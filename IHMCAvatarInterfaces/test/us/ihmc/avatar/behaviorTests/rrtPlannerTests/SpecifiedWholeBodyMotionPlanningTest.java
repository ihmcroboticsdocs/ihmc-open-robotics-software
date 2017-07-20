package us.ihmc.avatar.behaviorTests.rrtPlannerTests;

import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.util.ArrayList;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.pushDoor.PushDoor;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.pushDoor.PushDoorPose;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.pushDoor.PushDoorTrajectory;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.specifiedspace.TaskNode3D;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.specifiedspace.TaskNodeTree;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.tools.TaskNodeTreeVisualizer;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.tools.WheneverWholeBodyKinematicsSolver;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanel;
import us.ihmc.manipulation.planning.trajectory.EndEffectorLinearTrajectory;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.DoorEnvironment;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class SpecifiedWholeBodyMotionPlanningTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private boolean isKinematicsToolboxVisualizerEnabled = false;
   private DRCBehaviorTestHelper drcBehaviorTestHelper;
   private KinematicsToolboxModule kinematicsToolboxModule;
   private PacketCommunicator toolboxCommunicator;
   
   private static final boolean visualize = !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

   SolarPanel solarPanel;
      
   private void setUpSolarPanel()
   {
      Pose3D poseSolarPanel = new Pose3D();
      Quaternion quaternionSolarPanel = new Quaternion();
      poseSolarPanel.setPosition(0.7, -0.2, 1.03);
      quaternionSolarPanel.appendYawRotation(Math.PI*0.00);
      quaternionSolarPanel.appendRollRotation(0.0);
      quaternionSolarPanel.appendPitchRotation(-0.380);
      poseSolarPanel.setOrientation(quaternionSolarPanel);
      
      solarPanel = new SolarPanel(poseSolarPanel, 0.6, 0.6);      
   }
   
   private void setupKinematicsToolboxModule() throws IOException
   {
      DRCRobotModel robotModel = getRobotModel();
      kinematicsToolboxModule = new KinematicsToolboxModule(robotModel, isKinematicsToolboxVisualizerEnabled);
      toolboxCommunicator = drcBehaviorTestHelper.createAndStartPacketCommunicator(NetworkPorts.KINEMATICS_TOOLBOX_MODULE_PORT, PacketDestination.KINEMATICS_TOOLBOX_MODULE);
   }
   
   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");      
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (visualize)
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcBehaviorTestHelper != null)
      {
         drcBehaviorTestHelper.closeAndDispose();
         drcBehaviorTestHelper = null;
      }

      if (kinematicsToolboxModule != null)
      {
         kinematicsToolboxModule.destroy();
         kinematicsToolboxModule = null;
      }

      if (toolboxCommunicator != null)
      {
         toolboxCommunicator.closeConnection();
         toolboxCommunicator = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Before
   public void setUp() throws IOException
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

//      CommonAvatarEnvironmentInterface environment = new FlatGroundEnvironment();
      CommonAvatarEnvironmentInterface environment = new DoorEnvironment();
      

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(environment, getSimpleRobotName(), null, simulationTestingParameters, getRobotModel());
//      drcBehaviorTestHelper = new DRCBehaviorTestHelper(environment, getSimpleRobotName(), DRCObstacleCourseStartingLocation.OFFSET_ONE_METER_X_AND_Y_ROTATED_PI, simulationTestingParameters, getRobotModel());
//      drcBehaviorTestHelper = new DRCBehaviorTestHelper(environment, getSimpleRobotName(), DRCObstacleCourseStartingLocation.OFFSET_ONE_METER_X_AND_Y, simulationTestingParameters, getRobotModel());

      setupKinematicsToolboxModule();
   }
   
   public ArrayList<Graphics3DObject> getXYZAxis(Pose3D pose)
   {      
      double axisHeight = 0.1;
      double axisRadius = 0.01;
      ArrayList<Graphics3DObject> ret = new ArrayList<Graphics3DObject>();

      Graphics3DObject retX = new Graphics3DObject();
      Graphics3DObject retY = new Graphics3DObject();
      Graphics3DObject retZ = new Graphics3DObject();

      Point3D centerPoint = new Point3D(pose.getPosition());

      retX.translate(centerPoint);
      retY.translate(centerPoint);
      retZ.translate(centerPoint);
      
      RotationMatrix axisOrientation = new RotationMatrix(pose.getOrientation());
      
      RotationMatrix axisX = new RotationMatrix(axisOrientation);
      RotationMatrix axisY = new RotationMatrix(axisOrientation);
      RotationMatrix axisZ = new RotationMatrix(axisOrientation);
      
      retZ.rotate(axisZ);
      retZ.addCylinder(axisHeight, axisRadius, YoAppearance.Blue());

      axisX.appendPitchRotation(Math.PI*0.5);            
      retX.rotate(axisX);
      retX.addCylinder(axisHeight, axisRadius, YoAppearance.Red());

      
      axisY.appendRollRotation(-Math.PI*0.5);
      retY.rotate(axisY);
      retY.addCylinder(axisHeight, axisRadius, YoAppearance.Green());

      ret.add(retX);
      ret.add(retY);
      ret.add(retZ);

      return ret;
   }
   
   private Graphics3DObject getGraphicsSphere(Pose3D pose)
   {  
      Graphics3DObject graphicsSphere = new Graphics3DObject();
      Point3D translation1 = new Point3D(pose.getPosition());
      graphicsSphere.translate(translation1);
      graphicsSphere.addSphere(0.02, YoAppearance.DarkGray());
      return graphicsSphere;
   }
   
   private void showUpFullRobotModelWithConfiguration(FullHumanoidRobotModel createdFullRobotModel) throws SimulationExceededMaximumTimeException
   {
      for (int i = 0; i < createdFullRobotModel.getOneDoFJoints().length; i++)
      {         
         double jointPosition = createdFullRobotModel.getOneDoFJoints()[i].getQ();
         Joint scsJoint = drcBehaviorTestHelper.getRobot().getJoint(createdFullRobotModel.getOneDoFJoints()[i].getName());
         
         if (scsJoint instanceof PinJoint)
         {
            PinJoint pinJoint = (PinJoint) scsJoint;
            pinJoint.setQ(jointPosition);
         }
         else
         {
            PrintTools.info(createdFullRobotModel.getOneDoFJoints()[i].getName() + " was not a PinJoint.");
         }
      }

      FloatingJoint scsRootJoint = drcBehaviorTestHelper.getRobot().getRootJoint();
      scsRootJoint.setQuaternion(new Quaternion(createdFullRobotModel.getRootJoint().getRotationForReading()));
      scsRootJoint.setPosition(new Point3D(createdFullRobotModel.getRootJoint().getTranslationForReading()));
      
      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(0.005);
   }
   
//   @Test
   public void testForEndEffectorTrajectory() throws SimulationExceededMaximumTimeException, IOException
   {
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
   
      drcBehaviorTestHelper.updateRobotModel();
       
      Pose3D pose1 = new Pose3D(new Point3D(1.0, 1.0, 1.0), new Quaternion());
      Pose3D pose2 = new Pose3D(new Point3D(1.0, 2.0, 1.0), new Quaternion());
      Pose3D pose3 = new Pose3D(new Point3D(1.0, 2.0, 2.0), new Quaternion());
      Pose3D pose4 = new Pose3D(new Point3D(2.0, 2.0, 2.0), new Quaternion());
       
      EndEffectorLinearTrajectory constrainedEndEffectorTrajectory = new EndEffectorLinearTrajectory();
       
      constrainedEndEffectorTrajectory.setInitialPose(pose1);
      constrainedEndEffectorTrajectory.addLinearTrajectory(pose2, 1.0);
      constrainedEndEffectorTrajectory.addLinearTrajectory(pose3, 1.0);
      constrainedEndEffectorTrajectory.addLinearTrajectory(pose4, 1.0);      
       
      System.out.println(constrainedEndEffectorTrajectory.getEndEffectorPose(-1.0));
      System.out.println(constrainedEndEffectorTrajectory.getEndEffectorPose(2.0));
      System.out.println(constrainedEndEffectorTrajectory.getEndEffectorPose(2.5));
       
      System.out.println(constrainedEndEffectorTrajectory.getEndEffectorPose(5.5));
   }
   
//   @Test
   public void testForWheneverWholeBodyKinematicsSolver() throws SimulationExceededMaximumTimeException, IOException
   {      
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      drcBehaviorTestHelper.updateRobotModel();            
      drcBehaviorTestHelper.getControllerFullRobotModel().updateFrames();
      
      FullHumanoidRobotModel createdFullRobotModel;
      HumanoidReferenceFrames createdReferenceFrames;
      
      FullHumanoidRobotModel sdfFullRobotModel = drcBehaviorTestHelper.getControllerFullRobotModel();
      sdfFullRobotModel.updateFrames();
     
      /*
       * test 1 
       */
      // construct
      WheneverWholeBodyKinematicsSolver wbikTester = new WheneverWholeBodyKinematicsSolver(getRobotModel());
      
      // create initial robot configuration
      wbikTester.updateRobotConfigurationData(FullRobotModelUtils.getAllJointsExcludingHands(sdfFullRobotModel), sdfFullRobotModel.getRootJoint());      
      
      wbikTester.initialize();      
      wbikTester.holdCurrentTrajectoryMessages();
      
      // Desired
      Quaternion desiredHandOrientation = new Quaternion();
      desiredHandOrientation.appendPitchRotation(Math.PI*30/180);
      wbikTester.setDesiredHandPose(RobotSide.RIGHT, new Pose3D(new Point3D(0.6, -0.4, 1.0), desiredHandOrientation));
      wbikTester.setHandSelectionMatrixFree(RobotSide.LEFT);
      
      Quaternion desiredChestOrientation = new Quaternion();
      desiredChestOrientation.appendPitchRotation(Math.PI*10/180);
      wbikTester.setDesiredChestOrientation(desiredChestOrientation);
            
      wbikTester.setDesiredPelvisHeight(0.75);
      
      wbikTester.putTrajectoryMessages();

      PrintTools.info(""+wbikTester.isSolved());      
      
      /*
       * reversible test
       */
      
      wbikTester.initialize();      
      wbikTester.holdCurrentTrajectoryMessages();
      
      // Desired
      desiredHandOrientation = new Quaternion();
      desiredHandOrientation.appendPitchRotation(Math.PI*30/180);
      wbikTester.setDesiredHandPose(RobotSide.RIGHT, new Pose3D(new Point3D(0.5, -0.6, 1.1), desiredHandOrientation));
      wbikTester.setHandSelectionMatrixFree(RobotSide.LEFT);
      
      desiredChestOrientation = new Quaternion();
      desiredChestOrientation.appendPitchRotation(Math.PI*10/180);
      wbikTester.setDesiredChestOrientation(desiredChestOrientation);
            
      wbikTester.setDesiredPelvisHeight(0.75);
      
      wbikTester.putTrajectoryMessages();

      PrintTools.info(""+wbikTester.isSolved());      
      
      /*
       * reversible test
       */
      
      wbikTester.initialize();      
      wbikTester.holdCurrentTrajectoryMessages();
      
      // Desired
      desiredHandOrientation = new Quaternion();
      desiredHandOrientation.appendPitchRotation(Math.PI*30/180);
      wbikTester.setDesiredHandPose(RobotSide.RIGHT, new Pose3D(new Point3D(0.6, -0.4, 1.0), desiredHandOrientation));
      wbikTester.setHandSelectionMatrixFree(RobotSide.LEFT);
      
      desiredChestOrientation = new Quaternion();
      desiredChestOrientation.appendPitchRotation(Math.PI*10/180);
      wbikTester.setDesiredChestOrientation(desiredChestOrientation);
            
      wbikTester.setDesiredPelvisHeight(0.75);
      
      wbikTester.putTrajectoryMessages();

      PrintTools.info(""+wbikTester.isSolved());   
      
      /*
       * Show up
       */
      createdFullRobotModel = wbikTester.getDesiredFullRobotModel();
      createdReferenceFrames = new HumanoidReferenceFrames(createdFullRobotModel);
      
      wbikTester.printOutRobotModel(createdFullRobotModel, createdReferenceFrames.getMidFootZUpGroundFrame());
      showUpFullRobotModelWithConfiguration(createdFullRobotModel);
            
      PrintTools.info("END");     
   } 
      
//   @Test
   public void testForTaskNodeTree() throws SimulationExceededMaximumTimeException, IOException
   {
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
   
      drcBehaviorTestHelper.updateRobotModel();
            
      TaskNode3D rootNode = new TaskNode3D();
      
      TaskNodeTree taskNodeTree = new TaskNodeTree(rootNode, "pelvisHeight", "chestYaw", "chestPitch");
      
      taskNodeTree.getTaskNodeRegion().setRandomRegion(0, 0.0, 10.0);
      taskNodeTree.getTaskNodeRegion().setRandomRegion(1, Math.PI*(-0.2), Math.PI*(0.2));
      taskNodeTree.getTaskNodeRegion().setRandomRegion(2, Math.PI*(-0.2), Math.PI*(0.2));
      taskNodeTree.getTaskNodeRegion().setRandomRegion(3, Math.PI*(-0.2), Math.PI*(0.2));
      
      System.out.println(taskNodeTree.getTrajectoryTime());
      
      taskNodeTree.expandTree(300);
      
      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();
      
      TaskNodeTreeVisualizer taskNodeTreeVisualizer = new TaskNodeTreeVisualizer(scs, taskNodeTree);
      taskNodeTreeVisualizer.visualize();
   
      taskNodeTree.saveNodes();
      
      PrintTools.info("END");     
   } 
   
   @Test
   public void testForposeForNode() throws SimulationExceededMaximumTimeException, IOException
   {
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      
      drcBehaviorTestHelper.updateRobotModel();
      FullHumanoidRobotModel sdfFullRobotModel = drcBehaviorTestHelper.getControllerFullRobotModel();
      sdfFullRobotModel.updateFrames();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(sdfFullRobotModel);
      referenceFrames.updateFrames();
      /*
       * send hand trajectory message for initial posture
       */
      double motionTime = 3.0;
      Point3D point3D = new Point3D(0.6, -0.35, 1.0);
      Quaternion quaternion = new Quaternion();
      
      
      HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, motionTime, point3D, quaternion, referenceFrames.getMidFootZUpGroundFrame());
      WholeBodyTrajectoryMessage wholebodyTrajectoryMessage = new WholeBodyTrajectoryMessage();
      
      wholebodyTrajectoryMessage.setHandTrajectoryMessage(handTrajectoryMessage);
//      drcBehaviorTestHelper.send(wholebodyTrajectoryMessage);
//      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(motionTime);
                  
      /*
       * Initialize tester.
       */
      sdfFullRobotModel.updateFrames();
      referenceFrames.updateFrames();
      WheneverWholeBodyKinematicsSolver wbikTester = new WheneverWholeBodyKinematicsSolver(getRobotModel());
      
      wbikTester.updateRobotConfigurationData(FullRobotModelUtils.getAllJointsExcludingHands(sdfFullRobotModel), sdfFullRobotModel.getRootJoint());
                  
      TaskNode3D.nodeTester = wbikTester;      
      TaskNode3D.midZUpFrame = referenceFrames.getMidFootZUpGroundFrame();
      /*
       * Define end effector trajectory.  
       */
      
      Pose3D pose1 = new Pose3D(new Point3D(0.6, -0.4, 0.9), new Quaternion());
      Pose3D pose2 = new Pose3D(new Point3D(0.6, -0.5, 0.9), new Quaternion());
      Pose3D pose3 = new Pose3D(new Point3D(0.6, -0.5, 1.1), new Quaternion());
      Pose3D pose4 = new Pose3D(new Point3D(0.6, -0.4, 1.1), new Quaternion());
      
      EndEffectorLinearTrajectory constrainedEndEffectorTrajectory = new EndEffectorLinearTrajectory();
      
      constrainedEndEffectorTrajectory.setInitialPose(pose1);
      constrainedEndEffectorTrajectory.addLinearTrajectory(pose2, 3.0);
      constrainedEndEffectorTrajectory.addLinearTrajectory(pose3, 3.0);
      constrainedEndEffectorTrajectory.addLinearTrajectory(pose4, 3.0); 
      constrainedEndEffectorTrajectory.setRobotSideOfEndEffector(RobotSide.RIGHT);
            
      TaskNode3D.endEffectorTrajectory = constrainedEndEffectorTrajectory;      
      
      /*
       * Tree expanding.
       */
      
      double initialPelvisHeight = sdfFullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint().getTransformToWorldFrame().getM23();
      
      TaskNode3D rootNode = new TaskNode3D(0.0, initialPelvisHeight, 0.0, 0.0);      
            
      TaskNode3D node1 = new TaskNode3D(2.0, initialPelvisHeight+0.02, Math.PI/180*10, Math.PI/180*10);
      TaskNode3D node2 = new TaskNode3D(5.0, initialPelvisHeight+0.03, Math.PI/180*10, Math.PI/180*10);
      TaskNode3D node3 = new TaskNode3D(3.0, initialPelvisHeight+0.03, Math.PI/180*10, Math.PI/180*10);
      
      TaskNode3D node4 = new TaskNode3D(3.5, initialPelvisHeight+0.03, Math.PI/180*10, Math.PI/180*10);
      TaskNode3D node5 = new TaskNode3D(4.0, initialPelvisHeight+0.03, Math.PI/180*10, Math.PI/180*10);
      TaskNode3D node6 = new TaskNode3D(5.0, initialPelvisHeight+0.03, Math.PI/180*10, Math.PI/180*10);
      
      node1.setParentNode(rootNode);
      node2.setParentNode(node1);
      node3.setParentNode(rootNode);
      node4.setParentNode(rootNode);
      node5.setParentNode(rootNode);
      node6.setParentNode(rootNode);
      
      System.out.println(rootNode.isValidNode());
      System.out.println(node1.isValidNode());
      System.out.println(node2.isValidNode());
      System.out.println(node3.isValidNode());
      System.out.println(node4.isValidNode());
      System.out.println(node5.isValidNode());
      System.out.println(node6.isValidNode());
            
      PrintTools.info("END");     
   }       
   
//   @Test
   public void testForPushDoorKinematics() throws SimulationExceededMaximumTimeException, IOException
   {
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
   
      drcBehaviorTestHelper.updateRobotModel();
            
      FullHumanoidRobotModel sdfFullRobotModel = drcBehaviorTestHelper.getControllerFullRobotModel();
      sdfFullRobotModel.updateFrames();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(sdfFullRobotModel);
      referenceFrames.updateFrames();
      
      Point3D pushDoorLocation = new Point3D(0.5, -0.6, 0.0);
      Quaternion pushDoorOrientation = new Quaternion();
      pushDoorOrientation.appendYawRotation(Math.PI/180*10);
      FramePose pushDoorFramePose = new FramePose(referenceFrames.getMidFootZUpGroundFrame(), new Pose3D(pushDoorLocation, pushDoorOrientation));
      PushDoor pushDoor = new PushDoor(pushDoorFramePose, 0.8, 0.9);
      PushDoorTrajectory pushDoorTrajectory = new PushDoorTrajectory(pushDoor, 8.0, -30*Math.PI/180);
      pushDoorTrajectory.setRobotSideOfEndEffector(RobotSide.LEFT);
      
      PushDoorPose pushDoorPose1 = new PushDoorPose(pushDoor, RobotSide.LEFT, 0.0*Math.PI/180, 0.0*Math.PI/180);
      PushDoorPose pushDoorPose2 = new PushDoorPose(pushDoor, RobotSide.LEFT, -10.0*Math.PI/180, 0.0*Math.PI/180);
      PushDoorPose pushDoorPose3 = new PushDoorPose(pushDoor, RobotSide.LEFT, -20.0*Math.PI/180, 0.0*Math.PI/180);
      PushDoorPose pushDoorPose4 = new PushDoorPose(pushDoor, RobotSide.LEFT, -20.0*Math.PI/180, 10.0*Math.PI/180);
      PushDoorPose pushDoorPose5 = new PushDoorPose(pushDoor, RobotSide.LEFT, -20.0*Math.PI/180, -20.0*Math.PI/180);
      
      
      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();
      
      scs.addStaticLinkGraphics(pushDoor.getGraphics());
      scs.addStaticLinkGraphics(getXYZAxis(pushDoorPose1.getEndEffectorPose()));
      scs.addStaticLinkGraphics(getXYZAxis(pushDoorPose2.getEndEffectorPose()));
      scs.addStaticLinkGraphics(getXYZAxis(pushDoorPose3.getEndEffectorPose()));
      scs.addStaticLinkGraphics(getXYZAxis(pushDoorPose4.getEndEffectorPose()));
      scs.addStaticLinkGraphics(getXYZAxis(pushDoorPose5.getEndEffectorPose()));
                  
      /*
       * PushDoorPose IK Test
       */
      
      FullHumanoidRobotModel createdFullRobotModel;
            
      WheneverWholeBodyKinematicsSolver wbikTester = new WheneverWholeBodyKinematicsSolver(getRobotModel());
      
      // create initial robot configuration
      wbikTester.updateRobotConfigurationData(FullRobotModelUtils.getAllJointsExcludingHands(sdfFullRobotModel), sdfFullRobotModel.getRootJoint());      
      
      wbikTester.initialize();      
      wbikTester.holdCurrentTrajectoryMessages();
      
      // Desired
      Pose3D desiredPose = pushDoorPose5.getEndEffectorPose();
      FramePoint desiredPointToWorld = new FramePoint(referenceFrames.getWorldFrame(), desiredPose.getPosition());
      FrameOrientation desiredOrientationToWorld = new FrameOrientation(referenceFrames.getWorldFrame(), desiredPose.getOrientation());
            
      FramePose desiredPoseToWorld = new FramePose(desiredPointToWorld, desiredOrientationToWorld);      
      
      desiredPoseToWorld.changeFrame(referenceFrames.getMidFootZUpGroundFrame());
      
      Pose3D desiredPoseToMidZUp = new Pose3D(new Point3D(desiredPoseToWorld.getPosition()), new Quaternion(desiredPoseToWorld.getOrientation()));
      
      wbikTester.setDesiredHandPose(RobotSide.LEFT, desiredPoseToMidZUp);
      wbikTester.setHandSelectionMatrixFree(RobotSide.RIGHT);
                  
      wbikTester.setDesiredPelvisHeight(0.75);
      
      wbikTester.putTrajectoryMessages();

      PrintTools.info(""+wbikTester.isSolved());   
      createdFullRobotModel = wbikTester.getDesiredFullRobotModel();
      
      showUpFullRobotModelWithConfiguration(createdFullRobotModel);
      
      
      /*
       * pick up from trajectory
       */
      
      wbikTester.initialize();      
      wbikTester.holdCurrentTrajectoryMessages();
      
      // Desired
      double localTime = 7.0;
      desiredPose = pushDoorTrajectory.getEndEffectorPose(localTime);
      desiredPointToWorld = new FramePoint(referenceFrames.getWorldFrame(), desiredPose.getPosition());
      desiredOrientationToWorld = new FrameOrientation(referenceFrames.getWorldFrame(), desiredPose.getOrientation());
            
      desiredPoseToWorld = new FramePose(desiredPointToWorld, desiredOrientationToWorld);      
      
      desiredPoseToWorld.changeFrame(referenceFrames.getMidFootZUpGroundFrame());
      
      desiredPoseToMidZUp = new Pose3D(new Point3D(desiredPoseToWorld.getPosition()), new Quaternion(desiredPoseToWorld.getOrientation()));
      wbikTester.setDesiredHandPose(RobotSide.LEFT, desiredPoseToMidZUp);
      wbikTester.setHandSelectionMatrixFree(RobotSide.RIGHT);
                  
      wbikTester.setDesiredPelvisHeight(0.75);
      
      wbikTester.putTrajectoryMessages();

      PrintTools.info(""+wbikTester.isSolved());   
      createdFullRobotModel = wbikTester.getDesiredFullRobotModel();
      
      showUpFullRobotModelWithConfiguration(createdFullRobotModel);
      
      
      PrintTools.info("END");     
   } 
   
//   @Test
   public void testForBasicPushDoorOpeningMotion() throws SimulationExceededMaximumTimeException, IOException
   {
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
   
      drcBehaviorTestHelper.updateRobotModel();
            
      FullHumanoidRobotModel sdfFullRobotModel = drcBehaviorTestHelper.getControllerFullRobotModel();
      sdfFullRobotModel.updateFrames();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(sdfFullRobotModel);
      referenceFrames.updateFrames();
      
      Point3D pushDoorLocation = new Point3D(0.5, -0.6, 0.0);
      Quaternion pushDoorOrientation = new Quaternion();
      pushDoorOrientation.appendYawRotation(Math.PI/180*0);
      FramePose pushDoorFramePose = new FramePose(referenceFrames.getMidFootZUpGroundFrame(), new Pose3D(pushDoorLocation, pushDoorOrientation));
      PushDoor pushDoor = new PushDoor(pushDoorFramePose, 1.0, 0.9);
      
      PushDoorTrajectory pushDoorTrajectory = new PushDoorTrajectory(pushDoor, 8.0, -30*Math.PI/180);
      pushDoorTrajectory.setRobotSideOfEndEffector(RobotSide.LEFT);
            
      WholeBodyTrajectoryMessage wholebodyTrajectoryMessage = new WholeBodyTrajectoryMessage();
      HandTrajectoryMessage handTrajectoryMessage = pushDoorTrajectory.getEndEffectorTrajectoryMessage(referenceFrames.getMidFootZUpGroundFrame());
      
      ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage(2);
      chestTrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrame(referenceFrames.getMidFootZUpGroundFrame());
      chestTrajectoryMessage.getFrameInformation().setDataReferenceFrame(referenceFrames.getMidFootZUpGroundFrame());
         
      chestTrajectoryMessage.setTrajectoryPoint(0, 5.0, new Quaternion(), new Vector3D(), referenceFrames.getMidFootZUpGroundFrame());
      Quaternion chestOrientation = new Quaternion();
      chestOrientation.appendYawRotation(-Math.PI*30/180);
      chestOrientation.appendPitchRotation(Math.PI*20/180);
      chestTrajectoryMessage.setTrajectoryPoint(1, 10.0, chestOrientation, new Vector3D(), referenceFrames.getMidFootZUpGroundFrame());
                  
      wholebodyTrajectoryMessage.setChestTrajectoryMessage(chestTrajectoryMessage);
      wholebodyTrajectoryMessage.setHandTrajectoryMessage(handTrajectoryMessage);
                        
      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();      
      scs.addStaticLinkGraphics(pushDoor.getGraphics());
      System.out.println(handTrajectoryMessage.getNumberOfTrajectoryPoints());
      for(int i=0;i<handTrajectoryMessage.getNumberOfTrajectoryPoints();i++)
      {
         Point3D point = new Point3D();
         handTrajectoryMessage.getTrajectoryPoints()[i].getPosition(point);
         
         Quaternion orientation = new Quaternion();
         handTrajectoryMessage.getTrajectoryPoints()[i].getOrientation(orientation);
         
         Pose3D pose = new Pose3D(point, orientation);
         scs.addStaticLinkGraphics(getXYZAxis(pose));
      }
            
      drcBehaviorTestHelper.send(wholebodyTrajectoryMessage);
      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(19.0);
      
      PrintTools.info("END");     
   } 
   
//   @Test
   public void testForPlannerPushDoorOpeningTask() throws SimulationExceededMaximumTimeException, IOException
   {
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      drcBehaviorTestHelper.updateRobotModel();
      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();
      /*
       * Motion planning node.
       * 
       * Input @param.
       * solar panel position and orientation.
       * cleaning path. - robot side, via points, type(linear, circular_not yet).
       * trajectory time for initial position to root node pose.
       * finding initial node.
       * 
       * 
       * Output @param.
       * wholebody trajectory message.
       * -> hand trajectory message(end effector) is obtained from end effector path.
       * -> chest and pelvis trajectory message is obtained from planner.
       */
      
      /*
       * Initialize tester.
       */      
      FullHumanoidRobotModel sdfFullRobotModel = drcBehaviorTestHelper.getControllerFullRobotModel();
      sdfFullRobotModel.updateFrames();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(sdfFullRobotModel);
      referenceFrames.updateFrames();
      
      WheneverWholeBodyKinematicsSolver wbikTester = new WheneverWholeBodyKinematicsSolver(getRobotModel());
      
      wbikTester.updateRobotConfigurationData(FullRobotModelUtils.getAllJointsExcludingHands(sdfFullRobotModel), sdfFullRobotModel.getRootJoint());
                  
      TaskNode3D.nodeTester = wbikTester;
      TaskNode3D.midZUpFrame = referenceFrames.getMidFootZUpGroundFrame();
      /*
       * Push Door Define
       */
      Point3D pushDoorLocation = new Point3D(0.5, -0.4, 0.0);
      Quaternion pushDoorOrientation = new Quaternion();
      pushDoorOrientation.appendYawRotation(Math.PI/180*0);
      FramePose pushDoorFramePose = new FramePose(referenceFrames.getMidFootZUpGroundFrame(), new Pose3D(pushDoorLocation, pushDoorOrientation));
      PushDoor pushDoor = new PushDoor(pushDoorFramePose, 0.83, 0.9);
      
      PushDoorTrajectory pushDoorTrajectory = new PushDoorTrajectory(pushDoor, 8.0, -20*Math.PI/180);
      pushDoorTrajectory.setRobotSideOfEndEffector(RobotSide.LEFT);
                  
      TaskNode3D.endEffectorTrajectory = pushDoorTrajectory;      
      
      /*
       * Debug.
       */
      scs.addStaticLinkGraphics(pushDoor.getGraphics());
      HandTrajectoryMessage handTrajectoryMessage = pushDoorTrajectory.getEndEffectorTrajectoryMessage(referenceFrames.getMidFootZUpGroundFrame());
      
      System.out.println(handTrajectoryMessage.getNumberOfTrajectoryPoints());
      for(int i=0;i<handTrajectoryMessage.getNumberOfTrajectoryPoints();i++)
      {
         Point3D point = new Point3D();
         handTrajectoryMessage.getTrajectoryPoints()[i].getPosition(point);
         
         Quaternion orientation = new Quaternion();
         handTrajectoryMessage.getTrajectoryPoints()[i].getOrientation(orientation);
         
         Pose3D pose = new Pose3D(point, orientation);
         scs.addStaticLinkGraphics(getXYZAxis(pose));
      }
      
      /*
       * Tree expanding.
       */
      
      double initialPelvisHeight = sdfFullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint().getTransformToWorldFrame().getM23();
      
      TaskNode3D rootNode = new TaskNode3D(0.0, initialPelvisHeight, 0.0, 0.0);     
      rootNode.setConfigurationJoints(sdfFullRobotModel);
      
      TaskNodeTree taskNodeTree = new TaskNodeTree(rootNode, "pelvisHeight", "chestYaw", "chestPitch");
      
      taskNodeTree.getTaskNodeRegion().setRandomRegion(0, 0.0, pushDoorTrajectory.getTrajectoryTime());
      taskNodeTree.getTaskNodeRegion().setRandomRegion(1, 0.75, 0.9);
      taskNodeTree.getTaskNodeRegion().setRandomRegion(2, Math.PI*(-0.2), Math.PI*(0.2));
      taskNodeTree.getTaskNodeRegion().setRandomRegion(3, Math.PI*(-0.2), Math.PI*(0.2));
      
      System.out.println(taskNodeTree.getTrajectoryTime());
            
      taskNodeTree.expandTree(500);
            
      TaskNodeTreeVisualizer taskNodeTreeVisualizer = new TaskNodeTreeVisualizer(scs, taskNodeTree);
      taskNodeTreeVisualizer.visualize();
   
      taskNodeTree.saveNodes();
      
      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      
      PrintTools.info("END"); 
   }
   

   
   
   
   
   
   
   
   
   
   
   
   
   
}
