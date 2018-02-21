package us.ihmc.quadrupedRobotics.controller.forceDevelopment.states;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedBalanceManager;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedControlManagerFactory;
import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFeetManager;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.*;
import us.ihmc.quadrupedRobotics.estimator.GroundPlaneEstimator;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedStepCrossoverProjection;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedContactSequence;
import us.ihmc.quadrupedRobotics.planning.YoQuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.stepStream.QuadrupedStepStream;
import us.ihmc.quadrupedRobotics.planning.trajectory.PiecewiseReverseDcmTrajectory;
import us.ihmc.quadrupedRobotics.planning.trajectory.QuadrupedPiecewiseConstantCopTrajectory;
import us.ihmc.quadrupedRobotics.planning.trajectory.ThreeDoFMinimumJerkTrajectory;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPostureInputProviderInterface;
import us.ihmc.quadrupedRobotics.util.YoPreallocatedList;
import us.ihmc.robotics.dataStructures.parameter.DoubleArrayParameter;
import us.ihmc.robotics.dataStructures.parameter.DoubleParameter;
import us.ihmc.robotics.dataStructures.parameter.ParameterFactory;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.OrientationFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedStepController implements QuadrupedController, QuadrupedStepTransitionCallback
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static int STEP_SEQUENCE_CAPACITY = 100;
   private final QuadrupedPostureInputProviderInterface postureProvider;
   private final QuadrupedStepStream stepStream;
   private final YoDouble robotTimestamp;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   // parameters
   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final DoubleArrayParameter bodyOrientationProportionalGainsParameter = parameterFactory
         .createDoubleArray("bodyOrientationProportionalGains", 5000, 5000, 5000);
   private final DoubleArrayParameter bodyOrientationDerivativeGainsParameter = parameterFactory
         .createDoubleArray("bodyOrientationDerivativeGains", 750, 750, 750);
   private final DoubleArrayParameter bodyOrientationIntegralGainsParameter = parameterFactory.createDoubleArray("bodyOrientationIntegralGains", 0, 0, 0);
   private final DoubleParameter bodyOrientationMaxIntegralErrorParameter = parameterFactory.createDouble("bodyOrientationMaxIntegralError", 0);
   private final DoubleArrayParameter comForceCommandWeightsParameter = parameterFactory.createDoubleArray("comForceCommandWeights", 1, 1, 1);
   private final DoubleArrayParameter comTorqueCommandWeightsParameter = parameterFactory.createDoubleArray("comTorqueCommandWeights", 1, 1, 1);
   private final DoubleParameter dcmPositionStepAdjustmentGainParameter = parameterFactory.createDouble("dcmPositionStepAdjustmentGain", 1.5);
   private final DoubleParameter jointDampingParameter = parameterFactory.createDouble("jointDamping", 1);
   private final DoubleParameter jointPositionLimitDampingParameter = parameterFactory.createDouble("jointPositionLimitDamping", 10);
   private final DoubleParameter jointPositionLimitStiffnessParameter = parameterFactory.createDouble("jointPositionLimitStiffness", 100);
   private final DoubleParameter haltTransitionDurationParameter = parameterFactory.createDouble("haltTransitionDuration", 1.0);
   private final DoubleParameter minimumStepClearanceParameter = parameterFactory.createDouble("minimumStepClearance", 0.075);
   private final DoubleParameter maximumStepStrideParameter = parameterFactory.createDouble("maximumStepStride", 1.0);
   private final DoubleParameter coefficientOfFrictionParameter = parameterFactory.createDouble("coefficientOfFriction", 0.5);

   // frames

   // feedback controllers
   private final FramePoint3D dcmPositionEstimate;
   private final QuadrupedBodyOrientationController.Setpoints bodyOrientationControllerSetpoints;
   private final QuadrupedBodyOrientationController bodyOrientationController;

   // managers
   private final QuadrupedFeetManager feetManager;
   private final QuadrupedBalanceManager balanceManager;

   // task space controller
   private final QuadrupedTaskSpaceEstimates taskSpaceEstimates;
   private final QuadrupedTaskSpaceEstimator taskSpaceEstimator;
   private final QuadrupedTaskSpaceController.Commands taskSpaceControllerCommands;
   private final QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings;
   private final QuadrupedTaskSpaceController taskSpaceController;

   // step planner
   private static int MAXIMUM_STEP_QUEUE_SIZE = 100;
   private final GroundPlaneEstimator groundPlaneEstimator;
   private final QuadrantDependentList<YoFramePoint> groundPlanePositions;

   private final QuadrupedTimedContactSequence timedContactSequence;
   private final ThreeDoFMinimumJerkTrajectory dcmTransitionTrajectory;

   private final YoFrameVector instantaneousStepAdjustment;
   private final YoFrameVector accumulatedStepAdjustment;
   private final QuadrupedStepCrossoverProjection crossoverProjection;

   private final FrameQuaternion bodyOrientationReference;
   private final OrientationFrame bodyOrientationReferenceFrame;

   private final FramePoint3D stepGoalPosition;
   private final YoPreallocatedList<YoQuadrupedTimedStep> stepSequence;

   // inputs
   private final YoDouble haltTime = new YoDouble("haltTime", registry);
   private final YoBoolean haltFlag = new YoBoolean("haltFlag", registry);
   private final YoBoolean onLiftOffTriggered = new YoBoolean("onLiftOffTriggered", registry);
   private final YoBoolean onTouchDownTriggered = new YoBoolean("onTouchDownTriggered", registry);

   public QuadrupedStepController(QuadrupedForceControllerToolbox controllerToolbox, QuadrupedControlManagerFactory controlManagerFactory,
                                  QuadrupedPostureInputProviderInterface postureProvider, QuadrupedStepStream stepStream,
                                  YoVariableRegistry parentRegistry)
   {
      this.postureProvider = postureProvider;
      this.stepStream = stepStream;
      this.robotTimestamp = controllerToolbox.getRuntimeEnvironment().getRobotTimestamp();

      // frames
      QuadrupedReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();

      // feedback controllers
      dcmPositionEstimate = new FramePoint3D();
      bodyOrientationControllerSetpoints = new QuadrupedBodyOrientationController.Setpoints();
      bodyOrientationController = controllerToolbox.getBodyOrientationController();
      feetManager = controlManagerFactory.getOrCreateFeetManager();

      // task space controllers
      taskSpaceEstimates = new QuadrupedTaskSpaceEstimates();
      taskSpaceEstimator = controllerToolbox.getTaskSpaceEstimator();
      taskSpaceControllerCommands = new QuadrupedTaskSpaceController.Commands();
      taskSpaceControllerSettings = new QuadrupedTaskSpaceController.Settings();
      taskSpaceController = controllerToolbox.getTaskSpaceController();

      // step planner
      groundPlaneEstimator = controllerToolbox.getGroundPlaneEstimator();
      groundPlanePositions = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         groundPlanePositions.set(robotQuadrant, new YoFramePoint(robotQuadrant.getCamelCaseName() + "GroundPlanePosition", worldFrame, registry));
      }
      timedContactSequence = new QuadrupedTimedContactSequence(4, 2 * STEP_SEQUENCE_CAPACITY);
      dcmTransitionTrajectory = new ThreeDoFMinimumJerkTrajectory();
      instantaneousStepAdjustment = new YoFrameVector("instantaneousStepAdjustment", worldFrame, registry);
      accumulatedStepAdjustment = new YoFrameVector("accumulatedStepAdjustment", worldFrame, registry);
      crossoverProjection = new QuadrupedStepCrossoverProjection(referenceFrames.getBodyZUpFrame(), minimumStepClearanceParameter.get(),
            maximumStepStrideParameter.get());
      bodyOrientationReference = new FrameQuaternion();
      bodyOrientationReferenceFrame = new OrientationFrame(bodyOrientationReference);
      stepGoalPosition = new FramePoint3D();
      stepSequence = new YoPreallocatedList<>("stepSequence", registry, MAXIMUM_STEP_QUEUE_SIZE,
            new YoPreallocatedList.DefaultElementFactory<YoQuadrupedTimedStep>()
            {
               @Override
               public YoQuadrupedTimedStep createDefaultElement(String prefix, YoVariableRegistry registry)
               {
                  return new YoQuadrupedTimedStep(prefix, registry);
               }
            });

      balanceManager = new QuadrupedBalanceManager(controllerToolbox, stepSequence, postureProvider, registry, timedContactSequence,
                                                   dcmTransitionTrajectory, dcmPositionEstimate);

      parentRegistry.addChild(registry);
   }

   private void updateGains()
   {
      balanceManager.updateGains();

      bodyOrientationController.getGains().setProportionalGains(bodyOrientationProportionalGainsParameter.get());
      bodyOrientationController.getGains().setIntegralGains(bodyOrientationIntegralGainsParameter.get(), bodyOrientationMaxIntegralErrorParameter.get());
      bodyOrientationController.getGains().setDerivativeGains(bodyOrientationDerivativeGainsParameter.get());

      taskSpaceControllerSettings.getContactForceLimits().setCoefficientOfFriction(coefficientOfFrictionParameter.get());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointDamping(jointDampingParameter.get());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitDamping(jointPositionLimitDampingParameter.get());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitStiffness(jointPositionLimitStiffnessParameter.get());
      taskSpaceControllerSettings.getContactForceOptimizationSettings().setComForceCommandWeights(comForceCommandWeightsParameter.get());
      taskSpaceControllerSettings.getContactForceOptimizationSettings().setComTorqueCommandWeights(comTorqueCommandWeightsParameter.get());
   }

   private void updateSetpoints()
   {
      // trigger step events
      triggerStepEvents();

      // update ground plane estimate
      groundPlaneEstimator.clearContactPoints();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         groundPlaneEstimator.addContactPoint(groundPlanePositions.get(robotQuadrant));
      }
      groundPlaneEstimator.compute();

      // update step plan
      computeStepSequence();

      // update step adjustment
      computeStepAdjustment();

      // update desired horizontal com forces
      balanceManager.compute(taskSpaceControllerCommands.getComForce(), taskSpaceEstimates, taskSpaceControllerSettings);

      // update desired body orientation, angular velocity, and torque
      stepStream.getBodyOrientation(bodyOrientationReference);
      bodyOrientationReference.changeFrame(bodyOrientationReferenceFrame.getParent());
      bodyOrientationReferenceFrame.setOrientationAndUpdate(bodyOrientationReference);
      bodyOrientationControllerSetpoints.getBodyOrientation().changeFrame(bodyOrientationReferenceFrame);
      bodyOrientationControllerSetpoints.getBodyOrientation().set(postureProvider.getBodyOrientationInput());
      bodyOrientationControllerSetpoints.getBodyOrientation().changeFrame(worldFrame);
      double bodyOrientationYaw = bodyOrientationControllerSetpoints.getBodyOrientation().getYaw();
      double bodyOrientationPitch = bodyOrientationControllerSetpoints.getBodyOrientation().getPitch();
      double bodyOrientationRoll = bodyOrientationControllerSetpoints.getBodyOrientation().getRoll();
      bodyOrientationControllerSetpoints.getBodyOrientation()
            .setYawPitchRoll(bodyOrientationYaw, bodyOrientationPitch + groundPlaneEstimator.getPitch(bodyOrientationYaw), bodyOrientationRoll);
      bodyOrientationControllerSetpoints.getBodyAngularVelocity().setToZero();
      bodyOrientationControllerSetpoints.getComTorqueFeedforward().setToZero();
      bodyOrientationController.compute(taskSpaceControllerCommands.getComTorque(), bodyOrientationControllerSetpoints, taskSpaceEstimates);

      // update desired contact state and sole forces
      feetManager.compute(taskSpaceControllerCommands.getSoleForce(), taskSpaceEstimates);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         taskSpaceControllerSettings.setContactState(robotQuadrant, feetManager.getContactState(robotQuadrant));
      }

      // update joint setpoints
      taskSpaceController.compute(taskSpaceControllerSettings, taskSpaceControllerCommands);

      // update accumulated step adjustment
      if (onLiftOffTriggered.getBooleanValue())
      {
         onLiftOffTriggered.set(false);
      }
      if (onTouchDownTriggered.getBooleanValue())
      {
         onTouchDownTriggered.set(false);
         accumulatedStepAdjustment.add(instantaneousStepAdjustment);
         accumulatedStepAdjustment.setZ(0);
      }
   }

   private void triggerStepEvents()
   {
      for (int i = 0; i < stepSequence.size(); i++)
      {
         RobotQuadrant robotQuadrant = stepSequence.get(i).getRobotQuadrant();
         double currentTime = robotTimestamp.getDoubleValue();
         double startTime = stepSequence.get(i).getTimeInterval().getStartTime();
         double endTime = stepSequence.get(i).getTimeInterval().getEndTime();
         if (startTime < currentTime && currentTime < endTime)
         {
            feetManager.triggerStep(robotQuadrant, stepSequence.get(i));
         }
      }
   }

   private void computeStepSequence()
   {
      // update step plan
      stepSequence.clear();
      for (int i = 0; i < stepStream.getSteps().size(); i++)
      {
         if (!haltFlag.getBooleanValue() || stepStream.getSteps().get(i).getTimeInterval().getEndTime() < haltTime.getDoubleValue())
         {
            stepSequence.add();
            stepSequence.get(stepSequence.size() - 1).set(stepStream.getSteps().get(i));
            stepSequence.get(stepSequence.size() - 1).getGoalPosition(stepGoalPosition);
            stepGoalPosition.changeFrame(worldFrame);
            stepGoalPosition.add(accumulatedStepAdjustment);
            stepSequence.get(stepSequence.size() - 1).setGoalPosition(stepGoalPosition);
         }
      }
   }

   private void computeStepAdjustment()
   {
      if (robotTimestamp.getDoubleValue() > dcmTransitionTrajectory.getEndTime())
      {
         // compute step adjustment for ongoing steps (proportional to dcm tracking error)
         FramePoint3D dcmPositionSetpoint = balanceManager.getDCMPositionSetpoint();
         dcmPositionSetpoint.changeFrame(instantaneousStepAdjustment.getReferenceFrame());
         dcmPositionEstimate.changeFrame(instantaneousStepAdjustment.getReferenceFrame());
         instantaneousStepAdjustment.set(dcmPositionEstimate);
         instantaneousStepAdjustment.sub(dcmPositionSetpoint);
         instantaneousStepAdjustment.scale(dcmPositionStepAdjustmentGainParameter.get());
         instantaneousStepAdjustment.setZ(0);

         // adjust nominal step goal positions in foot state machine
         for (int i = 0; i < stepSequence.size(); i++)
         {
            RobotQuadrant robotQuadrant = stepSequence.get(i).getRobotQuadrant();
            double currentTime = robotTimestamp.getDoubleValue();
            double startTime = stepSequence.get(i).getTimeInterval().getStartTime();
            double endTime = stepSequence.get(i).getTimeInterval().getEndTime();
            if (startTime < currentTime && currentTime < endTime)
            {
               stepSequence.get(i).getGoalPosition(stepGoalPosition);
               stepGoalPosition.changeFrame(worldFrame);
               stepGoalPosition.add(instantaneousStepAdjustment);
               crossoverProjection.project(stepGoalPosition, taskSpaceEstimates.getSolePosition(), robotQuadrant);
               groundPlaneEstimator.projectZ(stepGoalPosition);
               feetManager.adjustStep(robotQuadrant, stepGoalPosition);
            }
         }
      }
   }

   @Override
   public void onLiftOff(RobotQuadrant thisStepQuadrant)
   {
      // update ground plane estimate
      groundPlanePositions.get(thisStepQuadrant).setAndMatchFrame(taskSpaceEstimates.getSolePosition(thisStepQuadrant));
      onLiftOffTriggered.set(true);
   }

   @Override
   public void onTouchDown(RobotQuadrant thisStepQuadrant)
   {
      onTouchDownTriggered.set(true);
   }

   @Override
   public void onEntry()
   {
      // initialize step stream
      stepStream.onEntry();

      // initialize state
      haltFlag.set(false);
      onLiftOffTriggered.set(false);
      onTouchDownTriggered.set(false);
      accumulatedStepAdjustment.setToZero();

      // initialize estimates

      // update task space estimates
      taskSpaceEstimator.compute(taskSpaceEstimates);

      // initialize feedback controllers
      balanceManager.initialize(taskSpaceEstimates);

      bodyOrientationControllerSetpoints.initialize(taskSpaceEstimates);
      bodyOrientationController.reset();

      feetManager.registerStepTransitionCallback(this);
      feetManager.reset();
      feetManager.requestFullContact();

      // initialize task space controller
      taskSpaceControllerSettings.initialize();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         taskSpaceControllerSettings.getContactForceOptimizationSettings().setContactForceCommandWeights(robotQuadrant, 0.0, 0.0, 0.0);
         taskSpaceControllerSettings.setContactState(robotQuadrant, ContactState.IN_CONTACT);
      }
      taskSpaceController.reset();

      // initialize ground plane
      groundPlaneEstimator.clearContactPoints();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         groundPlanePositions.get(robotQuadrant).setAndMatchFrame(taskSpaceEstimates.getSolePosition(robotQuadrant));
         groundPlaneEstimator.addContactPoint(groundPlanePositions.get(robotQuadrant));
      }
      groundPlaneEstimator.compute();

      // initialize timed contact sequence
      timedContactSequence.initialize();

      // compute step plan
      computeStepSequence();

      // compute step adjustment
      computeStepAdjustment();

      balanceManager.initializeDcmSetpoints(taskSpaceEstimates, taskSpaceControllerSettings);
   }

   @Override
   public ControllerEvent process()
   {
      double currentTime = robotTimestamp.getDoubleValue();
      if (stepSequence.size() == 0 || stepSequence.get(stepSequence.size() - 1).getTimeInterval().getEndTime() < currentTime)
      {
         return ControllerEvent.DONE;
      }
      stepStream.process();

      updateGains();


      // update task space estimates
      taskSpaceEstimator.compute(taskSpaceEstimates);

      updateSetpoints();
      return null;
   }

   @Override
   public void onExit()
   {
      // clean up step stream
      stepStream.onExit();

      feetManager.registerStepTransitionCallback(null);
   }

   public void halt()
   {
      if (haltFlag.getBooleanValue() == false)
      {
         haltFlag.set(true);
         haltTime.set(robotTimestamp.getDoubleValue() + haltTransitionDurationParameter.get());
      }
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }
}
