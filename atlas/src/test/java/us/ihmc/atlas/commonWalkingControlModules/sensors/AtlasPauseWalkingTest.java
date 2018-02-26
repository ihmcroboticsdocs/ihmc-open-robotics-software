package us.ihmc.atlas.commonWalkingControlModules.sensors;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.AvatarPauseWalkingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasPauseWalkingTest extends AvatarPauseWalkingTest
{
   AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return robotModel.getSimpleRobotName();
   }

   @Override
   public double getSwingTime()
   {
      return 0.6;
   }

   @Override
   public double getTransferTime()
   {
      return 0.2;
   }

   @Override
   public double getStepLength()
   {
      return 0.3;
   }

   @Override
   public double getStepWidth()
   {
      return 0.25;
   }

   @Override
   public double getTimeForPausing()
   {
      return 2.55;
   }

   @Override
   public double getTimeForResuming()
   {
      return 2.0;
   }

   @Override
   public int getNumberOfFootsteps()
   {
      return 5;
   }

   @ContinuousIntegrationTest(estimatedDuration = 100.0)
   @Test(timeout = 100000)
   @Override
   public void testPauseWalking() throws SimulationExceededMaximumTimeException
   {
      super.testPauseWalking();
   }

   @ContinuousIntegrationTest(estimatedDuration = 100.0)
   @Test(timeout = 100000)
   @Override
   public void testPauseWalkingForward() throws SimulationExceededMaximumTimeException
   {
      super.testPauseWalkingForward();
   }

   @ContinuousIntegrationTest(estimatedDuration = 100.0)
   @Test(timeout = 100000)
   @Override
   public void testPauseWalkingInitialTransfer() throws SimulationExceededMaximumTimeException
   {
      super.testPauseWalkingInitialTransfer();
   }

   @ContinuousIntegrationTest(estimatedDuration = 100.0)
   @Test(timeout = 100000)
   @Override
   public void testPauseWalkingForwardInitialTransfer() throws SimulationExceededMaximumTimeException
   {
      super.testPauseWalkingForwardInitialTransfer();
   }
}
