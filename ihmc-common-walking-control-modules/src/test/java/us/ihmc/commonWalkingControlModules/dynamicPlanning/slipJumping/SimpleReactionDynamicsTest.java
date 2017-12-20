package us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.testing.JUnitTools;

import java.util.Random;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.*;

public class SimpleReactionDynamicsTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testDynamics()
   {
      double mass = 10.0;
      double gravity = 9.81;
      double deltaT = 0.01;
      double flightDuration = 1.7;

      Vector3D boxSize = new Vector3D(0.3, 0.3, 0.5);
      SimpleReactionDynamics dynamics = new SimpleReactionDynamics(deltaT, mass, gravity);
      dynamics.setFlightDuration(flightDuration);

      Vector3D inertia = new Vector3D();
      inertia.setX(boxSize.getY() * boxSize.getY() + boxSize.getZ() * boxSize.getZ());
      inertia.setY(boxSize.getX() * boxSize.getX() + boxSize.getZ() * boxSize.getZ());
      inertia.setZ(boxSize.getY() * boxSize.getY() + boxSize.getX() * boxSize.getX());
      inertia.scale(mass / 12.0);

      FrameVector3D desiredLinearAcceleration = new FrameVector3D(ReferenceFrame.getWorldFrame(), 2.5, 3.5, 4.5);
      FrameVector3D desiredAngularAcceleration = new FrameVector3D(ReferenceFrame.getWorldFrame(), 5.5, 6.5, 7.5);

      Random random = new Random(1738L);
      DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize, 1);
      DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);

      currentControl.set(x, 0, desiredLinearAcceleration.getX() * mass);
      currentControl.set(y, 0, desiredLinearAcceleration.getY() * mass);
      currentControl.set(z, 0, (desiredLinearAcceleration.getZ() + gravity) * mass);
      currentControl.set(thetaX, 0, desiredAngularAcceleration.getX() * inertia.getX());
      currentControl.set(thetaY, 0, desiredAngularAcceleration.getY() * inertia.getY());
      currentControl.set(thetaZ, 0, desiredAngularAcceleration.getZ() * inertia.getZ());

      DenseMatrix64F nextState = new DenseMatrix64F(currentState);
      DenseMatrix64F nextStateExpected = new DenseMatrix64F(currentState);

      dynamics.getNextState(STANCE, currentState, currentControl, nextState);

      nextStateExpected.set(x, currentState.get(x) + deltaT * currentState.get(xDot) + 0.5 * deltaT * deltaT * desiredLinearAcceleration.getX());
      nextStateExpected.set(y, currentState.get(y) + deltaT * currentState.get(yDot) + 0.5 * deltaT * deltaT * desiredLinearAcceleration.getY());
      nextStateExpected.set(z, currentState.get(z) + deltaT * currentState.get(zDot) + 0.5 * deltaT * deltaT * desiredLinearAcceleration.getZ());
      nextStateExpected.set(thetaX, currentState.get(thetaX) + deltaT * currentState.get(thetaXDot) + 0.5 * deltaT * deltaT * desiredAngularAcceleration.getX());
      nextStateExpected.set(thetaY, currentState.get(thetaY) + deltaT * currentState.get(thetaYDot) + 0.5 * deltaT * deltaT * desiredAngularAcceleration.getY());
      nextStateExpected.set(thetaZ, currentState.get(thetaZ) + deltaT * currentState.get(thetaZDot) + 0.5 * deltaT * deltaT * desiredAngularAcceleration.getZ());
      nextStateExpected.set(xDot, currentState.get(xDot) + deltaT * desiredLinearAcceleration.getX());
      nextStateExpected.set(yDot, currentState.get(yDot) + deltaT * desiredLinearAcceleration.getY());
      nextStateExpected.set(zDot, currentState.get(zDot) + deltaT * desiredLinearAcceleration.getZ());
      nextStateExpected.set(thetaXDot, currentState.get(thetaXDot) + deltaT * desiredAngularAcceleration.getX());
      nextStateExpected.set(thetaYDot, currentState.get(thetaYDot) + deltaT * desiredAngularAcceleration.getY());
      nextStateExpected.set(thetaZDot, currentState.get(thetaZDot) + deltaT * desiredAngularAcceleration.getZ());

      JUnitTools.assertMatrixEquals(nextStateExpected, nextState, 1e-7);

      dynamics.getNextState(FLIGHT, currentState, currentControl, nextState);

      nextStateExpected.zero();

      nextStateExpected.set(x, currentState.get(x) + flightDuration * currentState.get(xDot));
      nextStateExpected.set(y, currentState.get(y) + flightDuration * currentState.get(yDot));
      nextStateExpected.set(z, currentState.get(z) + flightDuration * currentState.get(zDot) - 0.5 * flightDuration * flightDuration * gravity);
      nextStateExpected.set(thetaX, currentState.get(thetaX) + flightDuration * currentState.get(thetaXDot));
      nextStateExpected.set(thetaY, currentState.get(thetaY) + flightDuration * currentState.get(thetaYDot));
      nextStateExpected.set(thetaZ, currentState.get(thetaZ) + flightDuration * currentState.get(thetaZDot));
      nextStateExpected.set(xDot, currentState.get(xDot));
      nextStateExpected.set(yDot, currentState.get(yDot));
      nextStateExpected.set(zDot, currentState.get(zDot) - flightDuration * gravity);
      nextStateExpected.set(thetaXDot, currentState.get(thetaXDot));
      nextStateExpected.set(thetaYDot, currentState.get(thetaYDot));
      nextStateExpected.set(thetaZDot, currentState.get(thetaZDot));

      JUnitTools.assertMatrixEquals(nextStateExpected, nextState, 1e-7);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testDynamicsStateGradient()
   {
      double mass = 10.0;
      double gravity = 9.81;
      double deltaT = 0.01;
      double flightDuration = 1.7;

      Vector3D boxSize = new Vector3D(0.3, 0.3, 0.5);
      SimpleReactionDynamics dynamics = new SimpleReactionDynamics(deltaT, mass, gravity);
      dynamics.setFlightDuration(flightDuration);

      Vector3D inertia = new Vector3D();
      inertia.setX(boxSize.getY() * boxSize.getY() + boxSize.getZ() * boxSize.getZ());
      inertia.setY(boxSize.getX() * boxSize.getX() + boxSize.getZ() * boxSize.getZ());
      inertia.setZ(boxSize.getY() * boxSize.getY() + boxSize.getX() * boxSize.getX());
      inertia.scale(mass / 12.0);

      Random random = new Random(1738L);
      DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize, 1);
      DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);

      DenseMatrix64F gradient = new DenseMatrix64F(stateVectorSize, stateVectorSize);
      DenseMatrix64F gradientExpected = new DenseMatrix64F(stateVectorSize, stateVectorSize);

      dynamics.getDynamicsStateGradient(STANCE, currentState, currentControl, gradient);

      gradientExpected.zero();
      CommonOps.setIdentity(gradientExpected);
      gradientExpected.set(x, xDot, deltaT);
      gradientExpected.set(y, yDot, deltaT);
      gradientExpected.set(z, zDot, deltaT);
      gradientExpected.set(thetaX, thetaXDot, deltaT);
      gradientExpected.set(thetaY, thetaYDot, deltaT);
      gradientExpected.set(thetaZ, thetaZDot, deltaT);

      JUnitTools.assertMatrixEquals(gradientExpected, gradient, 1e-7);

      dynamics.getDynamicsStateGradient(FLIGHT, currentState, currentControl, gradient);

      gradientExpected.zero();
      CommonOps.setIdentity(gradientExpected);
      gradientExpected.set(x, xDot, flightDuration);
      gradientExpected.set(y, yDot, flightDuration);
      gradientExpected.set(z, zDot, flightDuration);
      gradientExpected.set(thetaX, thetaXDot, flightDuration);
      gradientExpected.set(thetaY, thetaYDot, flightDuration);
      gradientExpected.set(thetaZ, thetaZDot, flightDuration);

      JUnitTools.assertMatrixEquals(gradientExpected, gradient, 1e-7);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testDynamicsControlGradient()
   {
      double mass = 10.0;
      double gravity = 9.81;
      double deltaT = 0.01;
      double flightDuration = 1.7;

      Vector3D boxSize = new Vector3D(0.3, 0.3, 0.5);
      SimpleReactionDynamics dynamics = new SimpleReactionDynamics(deltaT, mass, gravity);
      dynamics.setFlightDuration(flightDuration);

      Vector3D inertia = new Vector3D();
      inertia.setX(boxSize.getY() * boxSize.getY() + boxSize.getZ() * boxSize.getZ());
      inertia.setY(boxSize.getX() * boxSize.getX() + boxSize.getZ() * boxSize.getZ());
      inertia.setZ(boxSize.getY() * boxSize.getY() + boxSize.getX() * boxSize.getX());
      inertia.scale(mass / 12.0);

      Random random = new Random(1738L);
      DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize, 1);
      DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);

      DenseMatrix64F gradient = new DenseMatrix64F(stateVectorSize, controlVectorSize);
      DenseMatrix64F gradientExpected = new DenseMatrix64F(stateVectorSize, controlVectorSize);

      dynamics.getDynamicsControlGradient(STANCE, currentState, currentControl, gradient);

      gradientExpected.set(x, fx, deltaT / mass);
      gradientExpected.set(y, fy, deltaT / mass);
      gradientExpected.set(z, fz, deltaT / mass);
      gradientExpected.set(thetaX, tauX, deltaT / inertia.getX());
      gradientExpected.set(thetaY, tauY, deltaT / inertia.getY());
      gradientExpected.set(thetaZ, tauZ, deltaT / inertia.getZ());
      gradientExpected.set(xDot, fx, deltaT / mass);
      gradientExpected.set(yDot, fy, deltaT / mass);
      gradientExpected.set(zDot, fz, deltaT / mass);
      gradientExpected.set(thetaXDot, tauX, deltaT / inertia.getX());
      gradientExpected.set(thetaYDot, tauY, deltaT / inertia.getY());
      gradientExpected.set(thetaZDot, tauZ, deltaT / inertia.getZ());

      JUnitTools.assertMatrixEquals(gradientExpected, gradient, 1e-7);

      dynamics.getDynamicsControlGradient(FLIGHT, currentState, currentControl, gradient);

      gradientExpected.zero();
      JUnitTools.assertMatrixEquals(gradientExpected, gradient, 1e-7);
   }
}
