package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.bambooTools.BambooTools;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.net.PacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.packets.manipulation.HandPoseListPacket;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseListBehavior;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.SysoutTool;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public abstract class DRCHandPoseListBehaviorTest implements MultiRobotTestInterface
{
   private static final boolean KEEP_SCS_UP = true;
   private static final boolean DEBUG = true;

   private final double JOINT_POSITION_THRESHOLD = 0.007;
   private final double EXTRA_SIM_TIME_FOR_SETTLING = 2.0;

   private static final boolean createMovie = BambooTools.doMovieCreation();
   private static final boolean checkNothingChanged = BambooTools.getCheckNothingChanged();
   private static final boolean showGUI = KEEP_SCS_UP || createMovie;

   private final DRCDemo01NavigationEnvironment testEnvironment = new DRCDemo01NavigationEnvironment();
   private final PacketCommunicator controllerCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), 10,
         "DRCHandPoseBehaviorTestControllerCommunicator");

   private DRCSimulationTestHelper drcSimulationTestHelper;

   private DoubleYoVariable yoTime;

   private RobotDataReceiver robotDataReceiver;
   private ForceSensorDataHolder forceSensorDataHolder;

   private BehaviorCommunicationBridge communicationBridge;

   private FullRobotModel fullRobotModel;

   private ArmJointName[] armJointNames;
   private int numberOfArmJoints;
   private LinkedHashMap<ArmJointName, Integer> armJointIndices = new LinkedHashMap<ArmJointName, Integer>();

   @Before
   public void setUp()
   {
      if (NetworkConfigParameters.USE_BEHAVIORS_MODULE)
      {
         throw new RuntimeException("Must set NetworkConfigParameters.USE_BEHAVIORS_MODULE = false in order to perform this test!");
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      drcSimulationTestHelper = new DRCSimulationTestHelper(testEnvironment, controllerCommunicator, getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, checkNothingChanged, showGUI, createMovie, false, getRobotModel());

      Robot robotToTest = drcSimulationTestHelper.getRobot();
      yoTime = robotToTest.getYoTime();

      fullRobotModel = getRobotModel().createFullRobotModel();

      armJointNames = fullRobotModel.getRobotSpecificJointNames().getArmJointNames();
      numberOfArmJoints = armJointNames.length;

      for (int i = 0; i < numberOfArmJoints; i++)
      {
         armJointIndices.put(armJointNames[i], i);
      }

      forceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(fullRobotModel.getForceSensorDefinitions()));

      robotDataReceiver = new RobotDataReceiver(fullRobotModel, forceSensorDataHolder, true);
      controllerCommunicator.attachListener(RobotConfigurationData.class, robotDataReceiver);

      PacketCommunicator junkyObjectCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), 10,
            "DRCHandPoseBehaviorTestJunkyCommunicator");

      communicationBridge = new BehaviorCommunicationBridge(junkyObjectCommunicator, controllerCommunicator, robotToTest.getRobotsYoVariableRegistry());
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (KEEP_SCS_UP)
      {
         //         ThreadTools.sleepForever();
      }

      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Test(timeout = 300000)
   public void testMoveOneRandomJoint90Deg() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      final HandPoseListBehavior handPoseListBehavior = new HandPoseListBehavior(communicationBridge, yoTime);
      communicationBridge.attachGlobalListenerToController(handPoseListBehavior.getControllerGlobalPacketConsumer());

      double swingTrajectoryTime = 2.0;
      int numberOfArmPoses = 2;
      RobotSide robotSide = RobotSide.LEFT;

      double[][] armPoses = createArmPosesInitializedToRobot(numberOfArmPoses, robotSide);

      int randomInt = RandomTools.generateRandomInt(new Random(), 0, numberOfArmJoints - 1);
      ArmJointName randomJoint = armJointNames[randomInt];
      int poseNumber = 1;
      setSingleJoint(armPoses, poseNumber, robotSide, randomJoint, Math.PI / 2, true);

      HandPoseListPacket handPoseListPacket = new HandPoseListPacket(robotSide, armPoses, swingTrajectoryTime);
      handPoseListBehavior.setInput(handPoseListPacket);

      success &= executeBehavior(handPoseListBehavior, swingTrajectoryTime);

      assertRobotAchievedFinalDesiredArmPose(armPoses, robotSide);
      assertTrue(success);
      assertTrue(handPoseListBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   @Test(timeout = 300000)
   public void testTwoPosesAtCurrentArmPose() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      final HandPoseListBehavior handPoseListBehavior = new HandPoseListBehavior(communicationBridge, yoTime);
      communicationBridge.attachGlobalListenerToController(handPoseListBehavior.getControllerGlobalPacketConsumer());

      double swingTrajectoryTime = 2.0;
      int numberOfArmPoses = 2;
      RobotSide robotSide = RobotSide.LEFT;

      double[][] armPoses = createArmPosesInitializedToRobot(numberOfArmPoses, robotSide);

      HandPoseListPacket handPoseListPacket = new HandPoseListPacket(robotSide, armPoses, swingTrajectoryTime);
      handPoseListBehavior.setInput(handPoseListPacket);

      success &= executeBehavior(handPoseListBehavior, swingTrajectoryTime);

      assertRobotAchievedFinalDesiredArmPose(armPoses, robotSide);
      assertTrue(success);
      assertTrue(handPoseListBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   @Test(timeout = 300000)
   public void testMoveBothArms() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      final HandPoseListBehavior leftHandPoseListBehavior = new HandPoseListBehavior(communicationBridge, yoTime);
      communicationBridge.attachGlobalListenerToController(leftHandPoseListBehavior.getControllerGlobalPacketConsumer());

      final HandPoseListBehavior rightHandPoseListBehavior = new HandPoseListBehavior(communicationBridge, yoTime);
      communicationBridge.attachGlobalListenerToController(rightHandPoseListBehavior.getControllerGlobalPacketConsumer());

      double swingTrajectoryTime = 2.0;
      int numberOfArmPoses = 2;

      double[][] leftArmPoses = createRandomArmPoses(numberOfArmPoses, RobotSide.LEFT);
      double[][] rightArmPoses = createRandomArmPoses(numberOfArmPoses, RobotSide.RIGHT);

      HandPoseListPacket leftHandPoseListPacket = new HandPoseListPacket(RobotSide.LEFT, leftArmPoses, swingTrajectoryTime);
      leftHandPoseListBehavior.setInput(leftHandPoseListPacket);

      HandPoseListPacket rightHandPoseListPacket = new HandPoseListPacket(RobotSide.RIGHT, rightArmPoses, swingTrajectoryTime);
      rightHandPoseListBehavior.setInput(rightHandPoseListPacket);

      ArrayList<BehaviorInterface> behaviors = new ArrayList<BehaviorInterface>();
      behaviors.add(leftHandPoseListBehavior);
      behaviors.add(rightHandPoseListBehavior);

      success &= executeBehaviors(behaviors, swingTrajectoryTime);

      assertRobotAchievedFinalDesiredArmPose(leftArmPoses, RobotSide.LEFT);
      assertRobotAchievedFinalDesiredArmPose(rightArmPoses, RobotSide.RIGHT);

      assertTrue(success);

      assertTrue(leftHandPoseListBehavior.isDone());
      assertTrue(rightHandPoseListBehavior.isDone());

      BambooTools.reportTestFinishedMessage();
   }

   private void setSingleJoint(double[][] armPosesToPack, int poseNumber, RobotSide robotSide, ArmJointName armJointName, double desiredJointAngle,
         boolean clipDesiredQToJointLimits)
   {
      int armJointIndex = armJointIndices.get(armJointName);
      setSingleJoint(armPosesToPack, poseNumber, robotSide, armJointIndex, desiredJointAngle, clipDesiredQToJointLimits);
   }

   private void setSingleJoint(double[][] armPosesToPack, int poseNumber, RobotSide robotSide, int armJointIndex, double desiredJointAngle,
         boolean clipDesiredQToJointLimits)
   {
      double q = desiredJointAngle;
      ArmJointName armJointName = armJointNames[armJointIndex];

      if (clipDesiredQToJointLimits)
      {
         q = clipDesiredToJointLimits(robotSide, armJointName, desiredJointAngle);
      }

      armPosesToPack[armJointIndex][poseNumber] = q;
   }

   private double clipDesiredToJointLimits(RobotSide robotSide, ArmJointName armJointName, double desiredJointAngle)
   {
      double q;
      double qMin = fullRobotModel.getArmJoint(robotSide, armJointName).getJointLimitLower();
      double qMax = fullRobotModel.getArmJoint(robotSide, armJointName).getJointLimitUpper();

      if (qMin > qMax)
      {
         double temp = qMax;
         qMax = qMin;
         qMin = temp;
      }

      q = MathTools.clipToMinMax(desiredJointAngle, qMin, qMax);
      return q;
   }

   private double[] getCurrentArmPose(RobotSide robotSide)
   {
      double[] armPose = new double[numberOfArmJoints];

      for (int jointNum = 0; jointNum < numberOfArmJoints; jointNum++)
      {
         ArmJointName jointName = armJointNames[jointNum];
         double currentAngle = fullRobotModel.getArmJoint(robotSide, jointName).getQ();
         armPose[jointNum] = currentAngle;
      }

      return armPose;
   }

   private double[] getDesiredArmPose(double[][] armPoses, int poseNumber)
   {
      double[] desiredPose = new double[numberOfArmJoints];

      for (int jointNum = 0; jointNum < numberOfArmJoints; jointNum++)
      {
         desiredPose[jointNum] = armPoses[jointNum][poseNumber];
      }

      return desiredPose;
   }

   private double[] getFinalDesiredArmPose(double[][] armPoses)
   {
      int lastPoseIndex = armPoses[0].length - 1;

      double[] desiredPose = getDesiredArmPose(armPoses, lastPoseIndex);

      return desiredPose;
   }

   private double[][] createRandomArmPoses(int numberOfPoses, RobotSide robotSide)
   {
      double[][] armPoses = new double[numberOfArmJoints][numberOfPoses];

      double[] desiredArmPose = createRandomArmPose(robotSide);

      for (int i = 0; i < numberOfPoses; i++)
      {
         setArmPose(armPoses, i, desiredArmPose);
      }

      return armPoses;
   }

   private double[] createRandomArmPose(RobotSide robotSide)
   {
      double[] armPose = new double[numberOfArmJoints];

      for (int jointNum = 0; jointNum < numberOfArmJoints; jointNum++)
      {
         double qDesired = clipDesiredToJointLimits(robotSide, armJointNames[jointNum], RandomTools.generateRandomDouble(new Random(), 1.5));
         armPose[jointNum] = qDesired;
      }

      return armPose;
   }

   private double[][] createArmPosesInitializedToRobot(int numberOfPoses, RobotSide robotSide)
   {
      double[][] armPoses = new double[numberOfArmJoints][numberOfPoses];

      double[] currentArmPose = getCurrentArmPose(robotSide);

      for (int i = 0; i < numberOfPoses; i++)
      {
         setArmPose(armPoses, i, currentArmPose);
      }

      return armPoses;
   }

   private void setArmPose(double[][] armPosesToPack, int poseNumber, double[] armPose)
   {
      for (ArmJointName jointName : armJointNames)
      {
         int jointNum = armJointIndices.get(jointName);
         armPosesToPack[jointNum][poseNumber] = armPose[jointNum];
      }
   }

   private boolean executeBehavior(final BehaviorInterface behavior, double trajectoryTime) throws SimulationExceededMaximumTimeException
   {
      ArrayList<BehaviorInterface> behaviors = new ArrayList<BehaviorInterface>();
      behaviors.add(behavior);

      return executeBehaviors(behaviors, trajectoryTime);
   }

   private boolean executeBehaviors(final ArrayList<BehaviorInterface> behaviors, double trajectoryTime) throws SimulationExceededMaximumTimeException
   {
      final double simulationRunTime = trajectoryTime + EXTRA_SIM_TIME_FOR_SETTLING;

      for (BehaviorInterface behavior : behaviors)
      {
         if (DEBUG)
         {
            System.out.println("\n");
            SysoutTool.println("starting behavior: " + behavior.getName() + "   t = " + yoTime.getDoubleValue());
         }

         spawnBehaviorThread(behavior, simulationRunTime);

      }

      boolean ret = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationRunTime);

      for (BehaviorInterface behavior : behaviors)
      {
         if (DEBUG)
         {
            SysoutTool.println("done with behavior: " + behavior.getName() + "   t = " + yoTime.getDoubleValue());
         }
      }
      return ret;
   }

   private void spawnBehaviorThread(final BehaviorInterface behavior, final double simulationRunTime)
   {
      Thread behaviorThread = new Thread()
      {
         public void run()
         {
            {
               double startTime = Double.NaN;
               boolean simStillRunning = true;
               boolean initalized = false;

               while (simStillRunning)
               {
                  if (!initalized)
                  {
                     startTime = yoTime.getDoubleValue();
                     initalized = true;
                  }

                  double timeSpentSimulating = yoTime.getDoubleValue() - startTime;
                  simStillRunning = timeSpentSimulating < simulationRunTime;

                  behavior.doControl();
               }
            }
         }
      };

      behaviorThread.start();
   }

   private void assertRobotAchievedFinalDesiredArmPose(double[][] armPoses, RobotSide robotSide)
   {
      double[] desiredArmPose = getFinalDesiredArmPose(armPoses);
      double[] actualArmPose = getCurrentArmPose(robotSide);

      for (int i = 0; i < numberOfArmJoints; i++)
      {
         ArmJointName armJointName = armJointNames[i];

         double q_desired = desiredArmPose[i];
         double q_actual = actualArmPose[i];

         if (DEBUG)
         {
            SysoutTool.println(armJointName + " qDesired = " + q_desired + ".  qActual = " + q_actual + ".");
         }

         assertEquals(q_desired, q_actual, JOINT_POSITION_THRESHOLD);
      }
   }
}
