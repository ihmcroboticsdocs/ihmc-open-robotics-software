package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import java.util.List;

import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentController.CentroidalMotionPlan;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentController.CentroidalMotionPlanGenerator;
import us.ihmc.commonWalkingControlModules.controlModules.flight.ContactState;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.math.trajectories.Trajectory;
import us.ihmc.robotics.math.trajectories.Trajectory3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * <p>Implements a SQP based centroidal motion planning approach to enable a larger variety of motions 
 * This models the robot as a point mass and a contact surface that can exert a restricted force on the
 * point mass. Constraints are imposed on plan generated to ensure that the force computed acts collinear 
 * to the CoP (constrained to be within the contact surface) and CoM. This ensures that the plan generated 
 * does not result in angular momentum changes. </p>
 * Essentially the mathematical equation being solved is <b>xddot = u (x - &n;) + g</b> where <b> x </b> is the CoM location,
 * <i> u  </i> is a scalar, <b>&nu;</b> is the CoP and <b> g </b> is the acceleration due to gravity
 * <p> All motion planning is done in the {@code ReferenceFrame#getWorldFrame()} </p>
 * @author Apoorv S
 *
 */
public class CollinearForceBasedCoMMotionPlanner implements CentroidalMotionPlanGenerator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int defaultQPIterationsToRun = 1;
   public static final int numberOfScalarTrajectoryCoefficients = 4;
   public static final int numberOfCoMTrajectoryCoefficients = 8;
   public static final int numberOfCoPTrajectoryCoefficients = 8;

   private final YoFramePoint3D initialCoMPosition;
   private final YoFramePoint3D initialCoPPosition;
   private final YoFrameVector3D initialCoMVelocity;
   private final YoFramePoint3D finalCoMPosition;
   private final YoFramePoint3D finalCoPPosition;
   private final YoFrameVector3D finalCoMVelocity;

   private final YoVariableRegistry registry;
   private final YoDouble nominalPlannerSegmentTime;
   private final YoDouble minPlannerSegmentTime;
   private final YoInteger numberOfContactStatesToPlan;
   private final YoInteger maxNumberOfSQPIterations;
   private final YoDouble consolidatedConvergenceThreshold;
   private final YoDouble individualAxisConvergenceThreshold;

   private final YoDouble dynamicsViolation;
   private final YoDouble[] axisDynamicsViolation = new YoDouble[Axis.values.length];

   private final YoInteger numberOfContactStates;
   private final YoInteger numberOfPlanningSegments;
   private final YoInteger numberOfElapsedSQPIterations;
   private final YoBoolean hasPlanConverged;
   private final YoBoolean hasPlannerFailed;
   private final YoBoolean hasInitialStateBeenSet;
   private final YoBoolean hasFinalStateBeenSet;

   private final RecyclingArrayList<CollinearForceMotionPlannerSegment> segmentList;
   private final RecyclingArrayList<ContactState> contactStateList;

   private final CollinearForceBasedPlannerResult sqpSolution;
   private final CollinearForceBasedPlannerOptimizationControlModule optimizationControlModule;
   private final CollinearForceBasedPlannerSeedSolutionGenerator initialSolutionGenerator;
   private final SmartContactStateProcessor contactStateProcessor;
   private final FramePoint3D tempPoint = new FramePoint3D();
   private final Point2D tempPoint2D = new Point2D();
   private final FrameVector3D tempVector = new FrameVector3D();

   public CollinearForceBasedCoMMotionPlanner(YoVariableRegistry parentRegistry)
   {
      String namePrefix = getClass().getSimpleName();
      registry = new YoVariableRegistry(namePrefix);

      initialCoMPosition = new YoFramePoint3D(namePrefix + "InitialCoMLocation", worldFrame, registry);
      initialCoPPosition = new YoFramePoint3D(namePrefix + "InitialCoPLocation", worldFrame, registry);
      initialCoMVelocity = new YoFrameVector3D(namePrefix + "InitialCoMVelocity", worldFrame, registry);
      finalCoMPosition = new YoFramePoint3D(namePrefix + "FinalCoMLocation", worldFrame, registry);
      finalCoPPosition = new YoFramePoint3D(namePrefix + "FinalCoPLocation", worldFrame, registry);
      finalCoMVelocity = new YoFrameVector3D(namePrefix + "FinalCoMVelocity", worldFrame, registry);

      dynamicsViolation = new YoDouble(namePrefix + "CummulativeDynamicsViolation", registry);
      for (Axis axis : Axis.values)
         axisDynamicsViolation[axis.ordinal()] = new YoDouble(namePrefix + axis.toString() + "DynamicsViolation", registry);
      hasPlanConverged = new YoBoolean(namePrefix + "HasPlannerConverged", registry);
      hasPlannerFailed = new YoBoolean(namePrefix + "HasPlannerFailed", registry);
      hasInitialStateBeenSet = new YoBoolean(namePrefix + "HasInitialStateBeenSet", registry);
      hasFinalStateBeenSet = new YoBoolean(namePrefix + "HasFinalStateBeenSet", registry);
      numberOfElapsedSQPIterations = new YoInteger(namePrefix + "NumberOfElapsedSQPIterations", registry);
      numberOfPlanningSegments = new YoInteger(namePrefix + "NumberOfPlanningSegments", registry);
      numberOfContactStates = new YoInteger(namePrefix + "NumberOfContactStates", registry);

      nominalPlannerSegmentTime = new YoDouble(namePrefix + "MaxPlannerSegmentTime", registry);
      minPlannerSegmentTime = new YoDouble(namePrefix + "MinPlannerSegmentTime", registry);
      numberOfContactStatesToPlan = new YoInteger(namePrefix + "NumberOfContactStatesToPlan", registry);
      maxNumberOfSQPIterations = new YoInteger(namePrefix + "MaxNumberOfSQPIterations", registry);
      consolidatedConvergenceThreshold = new YoDouble(namePrefix + "ConsolidatedConvergenceThreshold", registry);
      individualAxisConvergenceThreshold = new YoDouble(namePrefix + "IndividualAxisConvergenceThreshold", registry);

      sqpSolution = new CollinearForceBasedPlannerResult(registry);
      optimizationControlModule = new CollinearForceBasedPlannerOptimizationControlModule(sqpSolution, numberOfPlanningSegments, registry);
      initialSolutionGenerator = new CollinearForceBasedPlannerSeedSolutionGenerator(registry); // TODO set this up
      contactStateProcessor = new SmartContactStateProcessor(worldFrame, namePrefix, registry);
      contactStateList = new RecyclingArrayList<>(100, ContactState.class);
      segmentList = new RecyclingArrayList<>(100, CollinearForceMotionPlannerSegment.class);
      parentRegistry.addChild(registry);
      reset();
   }

   @Override
   public void initialize(MotionPlannerParameters plannerParameters, FrameVector3DReadOnly gravity)
   {
      if (plannerParameters instanceof CollinearForcePlannerParameters)
         initialize((CollinearForcePlannerParameters) plannerParameters, gravity);
      else
         throw new RuntimeException("Invalid planner parameters");
   }

   @Override
   public int getMaximumNumberOfContactStatesToPlan()
   {
      return maxNumberOfSQPIterations.getIntegerValue();
   }

   public void initialize(CollinearForcePlannerParameters parameters, FrameVector3DReadOnly gravity)
   {
      maxNumberOfSQPIterations.set(parameters.getMaxSQPIterations());
      consolidatedConvergenceThreshold.set(parameters.getConsolidatedConvergenceThreshold());
      individualAxisConvergenceThreshold.set(parameters.getIndividualAxisConvergenceThreshold());
      nominalPlannerSegmentTime.set(parameters.getNominalPlannerSegmentTime());
      minPlannerSegmentTime.set(parameters.getMinPlannerSegmentTime());
      numberOfContactStatesToPlan.set(parameters.getNumberOfContactStatesToPlan());

      sqpSolution.initialize(gravity, parameters.getRobotMass());
      initialSolutionGenerator.initialize(sqpSolution, gravity, parameters);
      optimizationControlModule.initialize(parameters, gravity);
      contactStateProcessor.initialize(parameters.getNumberOfSegmentsPerContactStateChange(), 15);
   }

   public void reset()
   {
      hasPlanConverged.set(false);
      hasPlannerFailed.set(false);
      numberOfElapsedSQPIterations.set(0);
      numberOfContactStates.set(0);
      numberOfPlanningSegments.set(0);
      contactStateList.clear();
      segmentList.clear();
      optimizationControlModule.reset();
      initialSolutionGenerator.reset();
      clearInitialState();
      clearFinalState();
      sqpSolution.reset();
   }

   public void clearInitialState()
   {
      hasInitialStateBeenSet.set(false);
      initialCoMPosition.setToNaN();
      initialCoMVelocity.setToNaN();
      initialCoPPosition.setToNaN();
   }

   public void clearFinalState()
   {
      hasFinalStateBeenSet.set(false);
      finalCoMPosition.setToNaN();
      finalCoMVelocity.setToNaN();
      finalCoPPosition.setToNaN();
   }

   @Override
   public void setInitialState(FramePoint3DReadOnly initialCoMLocation, FrameVector3DReadOnly initialCoMVelocity, FramePoint3DReadOnly initialCoPLocation)
   {
      hasInitialStateBeenSet.set(true);
      tempPoint.setIncludingFrame(initialCoMLocation);
      tempPoint.changeFrame(worldFrame);
      this.initialCoMPosition.set(tempPoint);

      tempPoint.setIncludingFrame(initialCoPLocation);
      tempPoint.changeFrame(worldFrame);
      this.initialCoPPosition.set(tempPoint);

      tempVector.setIncludingFrame(initialCoMVelocity);
      tempVector.changeFrame(worldFrame);
      this.initialCoMVelocity.set(tempVector);
   }

   @Override
   public void setFinalState(FramePoint3DReadOnly finalCoMLocation, FrameVector3DReadOnly finalCoMVelocity, FramePoint3DReadOnly finalCoPLocation)
   {
      hasFinalStateBeenSet.set(true);
      tempPoint.setIncludingFrame(finalCoMLocation);
      tempPoint.changeFrame(worldFrame);
      this.finalCoMPosition.set(tempPoint);

      tempPoint.setIncludingFrame(finalCoPLocation);
      tempPoint.changeFrame(worldFrame);
      this.finalCoPPosition.set(tempPoint);

      tempVector.setIncludingFrame(finalCoMVelocity);
      tempVector.changeFrame(worldFrame);
      this.finalCoMVelocity.set(tempVector);
   }

   public void processContactStateList()
   {
      contactStateProcessor.processContactStates(contactStateList, segmentList);
   }

   public void clearContactStateList()
   {
      reset();
   }

   @Override
   public void submitContactStateList(List<ContactState> contactStates)
   {
      contactStateList.clear();
      for (int i = 0; i < contactStates.size(); i++)
         appendContactStateToList(contactStates.get(i));
   }

   public void appendContactStateToList(ContactState contactStateToAppend)
   {
      if (isNodeValid(contactStateToAppend))
      {
         numberOfContactStates.increment();
         contactStateList.add().set(contactStateToAppend);
      }
   }

   public List<ContactState> getContactStateList()
   {
      return contactStateList;
   }

   private boolean checkIsDynamicsViolationBelowThresholds()
   {
      dynamicsViolation.set(0.0);
      boolean flag = true;
      flag &= !sqpSolution.didIterationConverge();

      for (Axis axis : Axis.values)
      {
         YoDouble axisViolation = axisDynamicsViolation[axis.ordinal()];
         axisViolation.set(Math.abs(sqpSolution.getViolation(axis)));
         dynamicsViolation.add(axisViolation);
         flag &= axisViolation.getDoubleValue() < individualAxisConvergenceThreshold.getDoubleValue();
      }
      flag &= dynamicsViolation.getDoubleValue() < consolidatedConvergenceThreshold.getDoubleValue();
      return flag;
   }

   private boolean isNodeValid(ContactState contactStateToCheck)
   {
      if (contactStateToCheck.getDuration() <= 0.0f)
         return false;
      for (RobotSide side : RobotSide.values)
      {
         if (contactStateToCheck.isFootInContact(side) && contactStateToCheck.getNumberOfSupportPolygonVertices(side) == 0)
            return false;
      }
      return true;
   }

   @Override
   public boolean compute()
   {
      runIterations(defaultQPIterationsToRun);
      return hasPlannerFailed();
   }

   @Override
   public void update()
   {
      throw new RuntimeException("Not implemented");
   }

   @Override
   public void prepareTransitionToNextContactState()
   {
      throw new RuntimeException("Not implemented");
   }

   public void runIterations(int numberOfSQPIterationsToRun)
   {
      if (isFirstQPRun())
      {
         processContactStateList();
         generateSeedSolution();
         setupOptimizationControlModule();
      }

      for (int i = 0; i < numberOfSQPIterationsToRun; i++)
      {
         if (!hasPlanConverged() && !hasPlannerFailed())
         {
            if (!optimizationControlModule.compute())
            {
               hasPlannerFailed.set(true);
               break;
            }
            optimizationControlModule.updateSolution();
            hasPlanConverged.set(checkIsDynamicsViolationBelowThresholds());
            numberOfElapsedSQPIterations.increment();
            hasPlannerFailed.set(numberOfElapsedSQPIterations.getIntegerValue() > maxNumberOfSQPIterations.getIntegerValue());
         }
         else
            break;
      }
   }

   private void setupOptimizationControlModule()
   {
      if (hasInitialStateBeenSet.getBooleanValue())
         optimizationControlModule.setDesiredInitialState(initialCoMPosition, initialCoPPosition, initialCoMVelocity);
      else
         throw new RuntimeException("Desired initial state not provided");
      if (hasFinalStateBeenSet.getBooleanValue())
         optimizationControlModule.setDesiredFinalState(finalCoMPosition, finalCoPPosition, finalCoMVelocity);
      else
         throw new RuntimeException("Desired final state not provided");
      optimizationControlModule.submitSegmentList(segmentList);
   }

   private void generateSeedSolution()
   {
      initialSolutionGenerator.submitSegmentList(segmentList);
      initialSolutionGenerator.setInitialState(initialCoMPosition, initialCoPPosition, initialCoMVelocity);
      initialSolutionGenerator.setFinalState(finalCoMPosition, finalCoPPosition, finalCoMVelocity);
      initialSolutionGenerator.computeSeedSolution();
   }

   private boolean isFirstQPRun()
   {
      return numberOfElapsedSQPIterations.getIntegerValue() == 0;
   }

   public boolean hasPlanConverged()
   {
      return hasPlanConverged.getBooleanValue();
   }

   public boolean hasPlannerFailed()
   {
      return hasPlannerFailed.getBooleanValue();
   }

   public List<CollinearForceMotionPlannerSegment> getSegmentList()
   {
      return segmentList;
   }

   public List<Trajectory3D> getCoMTrajectory()
   {
      return sqpSolution.comTrajectories;
   }

   public List<Trajectory3D> getCoPTrajectory()
   {
      return sqpSolution.copTrajectories;
   }

   public List<Trajectory> getScalarTrajectory()
   {
      return sqpSolution.scalarProfile;
   }

   @Override
   public CentroidalMotionPlan getMotionPlanReference()
   {
      return getSQPSolution();
   }

   public CollinearForceBasedPlannerResult getSQPSolution()
   {
      return sqpSolution;
   }
}
