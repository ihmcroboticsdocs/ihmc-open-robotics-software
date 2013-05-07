package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import org.ejml.data.DenseMatrix64F;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.MomentumOptimizerNative;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.MomentumOptimizerNativeInput;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.MomentumOptimizerNativeOutput;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ContactPointWrenchMatrixCalculator;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.exeptions.NoConvergenceException;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.*;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * @author twan
 *         Date: 4/25/13
 */
public class OptimizationMomentumControlModule implements MomentumControlModule
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final CentroidalMomentumHandler centroidalMomentumHandler;
   private final ExternalWrenchHandler externalWrenchHandler;
   private final HardMotionConstraintEnforcer hardMotionConstraintEnforcer = new HardMotionConstraintEnforcer();
   private final MotionConstraintHandler primaryMotionConstraintHandler;
   private final MotionConstraintHandler secondaryMotionConstraintHandler;
   private final ContactPointWrenchMatrixCalculator contactPointWrenchMatrixCalculator;
   private final MomentumOptimizerNativeInput momentumOptimizerNativeInput = new MomentumOptimizerNativeInput();
   private final MomentumOptimizerNative momentumOptimizerNative;
   private final MomentumOptimizationSettings momentumOptimizationSettings;

   private final BooleanYoVariable converged = new BooleanYoVariable("converged", registry);
   private final BooleanYoVariable hasNotConvergedInPast = new BooleanYoVariable("hasNotConvergedInPast", registry);
   private final InverseDynamicsJoint[] jointsToOptimizeFor;
   private final Map<ContactablePlaneBody, Wrench> wrenches = new LinkedHashMap<ContactablePlaneBody, Wrench>();
   private final MomentumRateOfChangeData momentumRateOfChangeData;

   public OptimizationMomentumControlModule(InverseDynamicsJoint rootJoint, ReferenceFrame centerOfMassFrame, double controlDT,
           YoVariableRegistry parentRegistry, InverseDynamicsJoint[] jointsToOptimizeFor, MomentumOptimizationSettings momentumOptimizationSettings,
           double gravityZ)
   {
      this.centroidalMomentumHandler = new CentroidalMomentumHandler(rootJoint, centerOfMassFrame, controlDT, registry);
      this.externalWrenchHandler = new ExternalWrenchHandler(gravityZ, centerOfMassFrame, rootJoint);
      this.primaryMotionConstraintHandler = new MotionConstraintHandler(jointsToOptimizeFor);
      this.secondaryMotionConstraintHandler = new MotionConstraintHandler(jointsToOptimizeFor);
      this.contactPointWrenchMatrixCalculator = new ContactPointWrenchMatrixCalculator(centerOfMassFrame, MomentumOptimizerNative.nSupportVectors,
              MomentumOptimizerNative.rhoSize);

      this.momentumOptimizerNative = new MomentumOptimizerNative(ScrewTools.computeDegreesOfFreedom(jointsToOptimizeFor), MomentumOptimizerNative.rhoSize); // TODO: reduce rhoSize?
      this.momentumOptimizationSettings = momentumOptimizationSettings;

      this.jointsToOptimizeFor = jointsToOptimizeFor;

      this.momentumRateOfChangeData = new MomentumRateOfChangeData(centerOfMassFrame);

      parentRegistry.addChild(registry);
      reset();
   }

   public void initialize()
   {
      centroidalMomentumHandler.initialize();
   }

   public void reset()
   {
      primaryMotionConstraintHandler.reset();
      secondaryMotionConstraintHandler.reset();
   }

   public void compute(LinkedHashMap<ContactablePlaneBody, ? extends PlaneContactState> contactStates, RobotSide upcomingSupportLeg)
   {
      centroidalMomentumHandler.compute();
      primaryMotionConstraintHandler.compute();

      hardMotionConstraintEnforcer.compute(centroidalMomentumHandler.getCentroidalMomentumMatrixPart(jointsToOptimizeFor),
            primaryMotionConstraintHandler.getJacobian(),
              primaryMotionConstraintHandler.getRightHandSide());

      DenseMatrix64F momentumDotEquationRightHandSide = centroidalMomentumHandler.getMomentumDotEquationRightHandSide(momentumRateOfChangeData);
      hardMotionConstraintEnforcer.constrainMomentumEquationRightHandSide(momentumDotEquationRightHandSide);

      momentumOptimizerNativeInput.setCentroidalMomentumMatrix(hardMotionConstraintEnforcer.getConstrainedCentroidalMomentumMatrix());
      momentumOptimizerNativeInput.setMomentumDotEquationRightHandSide(momentumDotEquationRightHandSide);


//      momentumOptimizerNativeInput.setCentroidalMomentumMatrix(centroidalMomentumHandler.getCentroidalMomentumMatrixPart(jointsToOptimizeFor));
//      momentumOptimizerNativeInput.setMomentumDotEquationRightHandSide(centroidalMomentumHandler.getMomentumDotEquationRightHandSide(momentumRateOfChangeData));

      momentumOptimizerNativeInput.setRhoMin(contactPointWrenchMatrixCalculator.getRhoMin(contactStates.values(),
              momentumOptimizationSettings.getRhoMinScalar()));

      contactPointWrenchMatrixCalculator.computeMatrix(contactStates.values());
      momentumOptimizerNativeInput.setContactPointWrenchMatrix(contactPointWrenchMatrixCalculator.getMatrix());
      DenseMatrix64F wrenchEquationRightHandSide = externalWrenchHandler.computeWrenchEquationRightHandSide(centroidalMomentumHandler.getCentroidalMomentumConvectiveTerm());
      hardMotionConstraintEnforcer.constrainWrenchEquationRightHandSide(wrenchEquationRightHandSide);
      momentumOptimizerNativeInput.setWrenchEquationRightHandSide(
            wrenchEquationRightHandSide);

      momentumOptimizerNativeInput.setMomentumDotWeight(momentumOptimizationSettings.getMomentumDotWeight(momentumRateOfChangeData.getMomentumSubspace()));
      momentumOptimizerNativeInput.setJointAccelerationRegularization(
          momentumOptimizationSettings.getDampedLeastSquaresFactorMatrix(ScrewTools.computeDegreesOfFreedom(jointsToOptimizeFor)));

      secondaryMotionConstraintHandler.compute();
      momentumOptimizerNativeInput.setSecondaryConstraintJacobian(secondaryMotionConstraintHandler.getJacobian());
      momentumOptimizerNativeInput.setSecondaryConstraintRightHandSide(secondaryMotionConstraintHandler.getRightHandSide());
      momentumOptimizerNativeInput.setNullspaceMatrix(secondaryMotionConstraintHandler.getNullspaceMatrix());
      momentumOptimizerNativeInput.setNullspaceMultipliers(secondaryMotionConstraintHandler.getNullspaceMultipliers());
      momentumOptimizerNativeInput.setSecondaryConstraintWeight(secondaryMotionConstraintHandler.getWeightMatrix());

      momentumOptimizerNativeInput.setGroundReactionForceRegularization(momentumOptimizationSettings.getGroundReactionForceRegularization());

      optimize(momentumOptimizerNativeInput);

      MomentumOptimizerNativeOutput output = momentumOptimizerNative.getOutput();

      contactPointWrenchMatrixCalculator.computeWrenches(contactStates.values(), output.getRho());

      for (ContactablePlaneBody contactablePlaneBody : contactStates.keySet())
      {
         Wrench wrench = contactPointWrenchMatrixCalculator.getWrench(contactStates.get(contactablePlaneBody));
         ReferenceFrame bodyFixedFrame = contactablePlaneBody.getRigidBody().getBodyFixedFrame();
         wrench.changeBodyFrameAttachedToSameBody(bodyFixedFrame);
         wrench.changeFrame(bodyFixedFrame);
         wrenches.put(contactablePlaneBody, wrench);
      }

      DenseMatrix64F jointAccelerations = hardMotionConstraintEnforcer.computeConstrainedJointAccelerations(output.getJointAccelerations());

      ScrewTools.setDesiredAccelerations(jointsToOptimizeFor, jointAccelerations);

//      for (InverseDynamicsJoint inverseDynamicsJoint : jointsToOptimizeFor)
//      {
//         System.out.print(inverseDynamicsJoint.getName() + ": ");
//         DenseMatrix64F desiredAccelerationMatrix = new DenseMatrix64F(inverseDynamicsJoint.getDegreesOfFreedom(), 1);
//         inverseDynamicsJoint.packDesiredAccelerationMatrix(desiredAccelerationMatrix, 0);
//         for (int i = 0; i < desiredAccelerationMatrix.getNumRows(); i++)
//         {
//            System.out.print(desiredAccelerationMatrix.get(i) + ", ");
//         }
//         System.out.println();
//      }

      centroidalMomentumHandler.computeCentroidalMomentumRate(jointsToOptimizeFor, jointAccelerations);
   }

   private void optimize(MomentumOptimizerNativeInput momentumOptimizerNativeInput)
   {
      try
      {
         momentumOptimizerNative.solve(momentumOptimizerNativeInput);
         converged.set(true);
      }
      catch (NoConvergenceException e)
      {
         if (!hasNotConvergedInPast.getBooleanValue())
         {
            e.printStackTrace();
            System.err.println("WARNING: Only showing the stack trace of the first " + e.getClass().getSimpleName()
                               + ". This may be happening more than once. See value of YoVariable " + converged.getName() + ".");
         }

         converged.set(false);
         hasNotConvergedInPast.set(true);
      }
   }

   public void resetGroundReactionWrenchFilter()
   {
      // empty for now
   }

   public void setDesiredJointAcceleration(InverseDynamicsJoint joint, DenseMatrix64F jointAcceleration)
   {
      primaryMotionConstraintHandler.setDesiredJointAcceleration(joint, jointAcceleration, Double.POSITIVE_INFINITY); // weight is arbitrary, actually
   }

   public void setDesiredSpatialAcceleration(GeometricJacobian jacobian, TaskspaceConstraintData taskspaceConstraintData)
   {
      primaryMotionConstraintHandler.setDesiredSpatialAcceleration(jacobian, taskspaceConstraintData, Double.POSITIVE_INFINITY); // weight is arbitrary, actually
   }

   public void setDesiredRateOfChangeOfMomentum(MomentumRateOfChangeData momentumRateOfChangeData)
   {
      this.momentumRateOfChangeData.set(momentumRateOfChangeData);
   }

   public void setDesiredJointAcceleration(InverseDynamicsJoint joint, DenseMatrix64F jointAcceleration, double weight)
   {
      secondaryMotionConstraintHandler.setDesiredJointAcceleration(joint, jointAcceleration, weight);
   }

   public void setDesiredSpatialAcceleration(GeometricJacobian jacobian, TaskspaceConstraintData taskspaceConstraintData, double weight)
   {
      secondaryMotionConstraintHandler.setDesiredSpatialAcceleration(jacobian, taskspaceConstraintData, weight);
   }

   public SpatialForceVector getDesiredCentroidalMomentumRate()
   {
      return centroidalMomentumHandler.getCentroidalMomentumRate();
   }

   public Map<ContactablePlaneBody, Wrench> getExternalWrenches()
   {
      return wrenches;
   }
}
