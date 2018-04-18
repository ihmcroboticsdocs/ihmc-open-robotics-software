package us.ihmc.quadrupedRobotics.planning.stepStream;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitPlanner;
import us.ihmc.quadrupedRobotics.providers.YoQuadrupedXGaitSettings;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameYawPitchRoll;

import java.util.List;

public class QuadrupedXGaitStepStream
{
   private static int NUMBER_OF_PREVIEW_STEPS = 16;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleParameter initialStepDelayParameter = new DoubleParameter("initialStepDelay", registry, 0.5);
   private final DoubleParameter minimumStepClearanceParameter = new DoubleParameter("minimumStepClearance", registry, 0.075);

   private final YoQuadrupedXGaitSettings xGaitSettings;
   private final FramePoint3D supportCentroid;
   private final ReferenceFrame supportFrame;
   private final ReferenceFrame bodyZUpFrame;
   private final double controlDT;
   private final YoDouble timestamp, previousTimestamp;
   private final Vector3D desiredPlanarVelocity = new Vector3D();

   private final YoDouble bodyYaw;
   private final YoFrameYawPitchRoll bodyOrientation;
   private final QuadrupedXGaitPlanner xGaitStepPlanner;
   private final QuadrupedPlanarFootstepPlan footstepPlan;

   public QuadrupedXGaitStepStream(YoQuadrupedXGaitSettings xGaitSettings, QuadrupedReferenceFrames referenceFrames, YoDouble timestamp,
                                   YoVariableRegistry parentRegistry)
   {
      this(xGaitSettings, referenceFrames, Double.NaN, timestamp, parentRegistry);
   }

   public QuadrupedXGaitStepStream(YoQuadrupedXGaitSettings xGaitSettings, QuadrupedReferenceFrames referenceFrames, double controlDT, YoDouble timestamp,
                                   YoVariableRegistry parentRegistry)
   {
      this.xGaitSettings = xGaitSettings;
      this.supportCentroid = new FramePoint3D();
      this.supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      this.bodyZUpFrame = referenceFrames.getBodyZUpFrame();
      this.controlDT = controlDT;
      this.timestamp = timestamp;
      this.previousTimestamp = new YoDouble("previousTimestamp", registry);

      this.bodyYaw = new YoDouble("bodyYaw", registry);
      this.bodyOrientation = new YoFrameYawPitchRoll("bodyOrientation", worldFrame, registry);
      this.xGaitStepPlanner = new QuadrupedXGaitPlanner();
      this.footstepPlan = new QuadrupedPlanarFootstepPlan(NUMBER_OF_PREVIEW_STEPS);

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }
   }

   private void updateXGaitSettings()
   {
      // increase stance dimensions as a function of velocity to prevent self collisions
      double strideRotation = desiredPlanarVelocity.getZ() * xGaitSettings.getStepDuration();
      double strideLength = Math.abs(2 * desiredPlanarVelocity.getX() * xGaitSettings.getStepDuration());
      double strideWidth = Math.abs(2 * desiredPlanarVelocity.getY() * xGaitSettings.getStepDuration());
      strideLength += Math.abs(xGaitSettings.getStanceWidth() / 2 * Math.sin(2 * strideRotation));
      strideWidth += Math.abs(xGaitSettings.getStanceLength() / 2 * Math.sin(2 * strideRotation));
      xGaitSettings.setStanceLength(Math.max(xGaitSettings.getStanceLength(), strideLength / 2 + minimumStepClearanceParameter.getValue()));
      xGaitSettings.setStanceWidth(Math.max(xGaitSettings.getStanceWidth(), strideWidth / 2 + minimumStepClearanceParameter.getValue()));
   }

   public void onEntry()
   {
      // initialize body orientation
      bodyOrientation.setFromReferenceFrame(bodyZUpFrame);
      bodyYaw.set(bodyOrientation.getYaw().getDoubleValue());
      previousTimestamp.set(timestamp.getDoubleValue());

      // initialize step queue
      updateXGaitSettings();
      supportCentroid.setToZero(supportFrame);
      double initialYaw = bodyYaw.getDoubleValue();
      double initialTime = timestamp.getDoubleValue() + initialStepDelayParameter.getValue();
      RobotQuadrant initialQuadrant = (xGaitSettings.getEndPhaseShift() < 90) ? RobotQuadrant.HIND_LEFT : RobotQuadrant.FRONT_LEFT;
      xGaitStepPlanner.computeInitialPlan(footstepPlan, desiredPlanarVelocity, initialQuadrant, supportCentroid, initialTime, initialYaw, xGaitSettings);
      footstepPlan.initializeCurrentStepsFromPlannedSteps();
      this.process();
   }

   public void process()
   {
      double currentTime = timestamp.getDoubleValue();

      // update body orientation
      if(Double.isNaN(controlDT))
      {
         double effectiveDT = timestamp.getDoubleValue() - previousTimestamp.getDoubleValue();
         previousTimestamp.set(timestamp.getDoubleValue());
         updateBodyOrientation(effectiveDT);
      }
      else
      {
         updateBodyOrientation(controlDT);
      }

      // update xgait current steps
      footstepPlan.updateCurrentSteps(timestamp.getDoubleValue());

      // update xgait preview steps
      supportCentroid.setToZero(supportFrame);
      supportCentroid.changeFrame(worldFrame);

      updateXGaitSettings();
      double currentYaw = bodyYaw.getDoubleValue();
      xGaitStepPlanner.computeOnlinePlan(footstepPlan, desiredPlanarVelocity, currentTime, currentYaw, supportCentroid.getZ(), xGaitSettings);
   }

   public void setDesiredPlanarVelocity(double desiredVelocityX, double desiredVelocityY, double desiredVelocityYaw)
   {
      desiredPlanarVelocity.set(desiredVelocityX, desiredVelocityY, desiredVelocityYaw);
   }

   private void updateBodyOrientation(double dt)
   {
      bodyYaw.add(desiredPlanarVelocity.getZ() * dt);
      bodyOrientation.setYawPitchRoll(bodyYaw.getDoubleValue(), 0.0, 0.0);
   }

   public List<? extends QuadrupedTimedStep> getSteps()
   {
      return footstepPlan.getCompleteStepSequence(timestamp.getDoubleValue());
   }

   public QuadrupedPlanarFootstepPlan getFootstepPlan()
   {
      return footstepPlan;
   }
}
