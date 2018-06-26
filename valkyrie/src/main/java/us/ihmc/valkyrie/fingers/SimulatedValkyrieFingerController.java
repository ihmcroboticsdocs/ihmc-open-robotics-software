package us.ihmc.valkyrie.fingers;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.FingerMotorTrajectoryMessage;
import controller_msgs.msg.dds.FingerTrajectoryMessage;
import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import controller_msgs.msg.dds.HandJointAnglePacket;
import controller_msgs.msg.dds.TrajectoryPoint1DMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.subscribers.FingerTrajectoryMessageSubscriber;
import us.ihmc.humanoidRobotics.communication.subscribers.HandDesiredConfigurationMessageSubscriber;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPIDGains;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.simulationConstructionSetTools.robotController.MultiThreadedRobotControlElement;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizerInterface;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

/**
 * This FingerController is for simulating finger trajectory generation given by the FingerTrajectoryMessages.
 * Once OneDegreeOfFreedomJoint of finger joint is initiated in scs, Valkyrie can move finger by this FingerController.
 */
public class SimulatedValkyrieFingerController implements MultiThreadedRobotControlElement
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final YoLong lastEstimatorStartTime = new YoLong("nextExecutionTime", registry);

   private final long controlDTInNS;
   private final long estimatorDTInNS;

   private final ThreadDataSynchronizerInterface threadDataSynchronizer;
   private final YoDouble handControllerTime = new YoDouble("handControllerTime", registry);
   private final SimulatedValkyrieFingerJointAngleProducer jointAngleProducer;

   private final SideDependentList<HandDesiredConfigurationMessageSubscriber> handDesiredConfigurationMessageSubscribers = new SideDependentList<>();
   private final SideDependentList<FingerTrajectoryMessageSubscriber> fingerTrajectoryMessageSubscribers = new SideDependentList<>();

   private final SideDependentList<IndividualValkyrieFingerSetController> individualHandControllers = new SideDependentList<>();

   private final SideDependentList<List<ValkyrieFingerMotorName>> sideListsOfValkyrieFingerMotorNames = new SideDependentList<>();

   private final YoPIDGains gains = new YoPIDGains("Hand", registry);

   public SimulatedValkyrieFingerController(FloatingRootJointRobot simulatedRobot, ThreadDataSynchronizerInterface threadDataSynchronizer,
                                            RealtimeRos2Node realtimeRos2Node, CloseableAndDisposableRegistry closeableAndDisposableRegistry,
                                            DRCRobotModel robotModel, MessageTopicNameGenerator pubTopicNameGenerator,
                                            MessageTopicNameGenerator subTopicNameGenerator)
   {
      this.threadDataSynchronizer = threadDataSynchronizer;
      this.controlDTInNS = Conversions.secondsToNanoseconds(robotModel.getControllerDT());
      this.estimatorDTInNS = Conversions.secondsToNanoseconds(robotModel.getEstimatorDT());

      if (realtimeRos2Node != null)
      {
         IHMCRealtimeROS2Publisher<HandJointAnglePacket> jointAnglePublisher = ROS2Tools.createPublisher(realtimeRos2Node, HandJointAnglePacket.class,
                                                                                                         pubTopicNameGenerator);
         jointAngleProducer = new SimulatedValkyrieFingerJointAngleProducer(jointAnglePublisher, simulatedRobot, closeableAndDisposableRegistry);
      }
      else
      {
         jointAngleProducer = null;
      }

      gains.setKp(7.0);
      gains.setKi(3.0);
      gains.setKd(1.0);

      for (RobotSide robotSide : RobotSide.values)
      {
         List<ValkyrieFingerMotorName> valkyrieFingerMotorNames = new ArrayList<ValkyrieFingerMotorName>();
         valkyrieFingerMotorNames.add(ValkyrieFingerMotorName.IndexFingerMotorPitch1);
         valkyrieFingerMotorNames.add(ValkyrieFingerMotorName.MiddleFingerMotorPitch1);
         valkyrieFingerMotorNames.add(ValkyrieFingerMotorName.PinkyMotorPitch1);
         sideListsOfValkyrieFingerMotorNames.put(robotSide, valkyrieFingerMotorNames);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         HandDesiredConfigurationMessageSubscriber handDesiredConfigurationSubscriber = new HandDesiredConfigurationMessageSubscriber(robotSide);
         handDesiredConfigurationMessageSubscribers.put(robotSide, handDesiredConfigurationSubscriber);

         FingerTrajectoryMessageSubscriber fingerTrajectoryMessageSubscriber = new FingerTrajectoryMessageSubscriber(robotSide);
         fingerTrajectoryMessageSubscribers.put(robotSide, fingerTrajectoryMessageSubscriber);
         if (realtimeRos2Node != null)
         {
            ROS2Tools.createCallbackSubscription(realtimeRos2Node, HandDesiredConfigurationMessage.class, subTopicNameGenerator,
                                                 handDesiredConfigurationSubscriber);
            ROS2Tools.createCallbackSubscription(realtimeRos2Node, FingerTrajectoryMessage.class, subTopicNameGenerator, fingerTrajectoryMessageSubscriber);
         }

         IndividualValkyrieFingerSetController individualHandController = new IndividualValkyrieFingerSetController(robotSide, handControllerTime,
                                                                                                                    simulatedRobot, registry);
         individualHandControllers.put(robotSide, individualHandController);
      }
   }

   @Override
   public void initialize()
   {

   }

   @Override
   public void read(long currentClockTime)
   {
      long timestamp;
      if (threadDataSynchronizer != null)
      {
         timestamp = threadDataSynchronizer.getTimestamp();
         handControllerTime.set(Conversions.nanosecondsToSeconds(timestamp));
      }
      else
      {
         handControllerTime.add(Conversions.nanosecondsToSeconds(controlDTInNS));
      }

      if (jointAngleProducer != null)
      {
         jointAngleProducer.sendHandJointAnglesPacket();
      }
   }

   @Override
   public void run()
   {
      checkForNewHandDesiredConfigurationRequested();
      checkForNewFingerTrajectoryRequested();

      for (RobotSide robotSide : RobotSide.values)
      {
         individualHandControllers.get(robotSide).doControl();
      }
   }

   @Override
   public void write(long timestamp)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         individualHandControllers.get(robotSide).writeDesiredJointAngles();
      }
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return null;
   }

   @Override
   public long nextWakeupTime()
   {
      if (lastEstimatorStartTime.getLongValue() == Long.MIN_VALUE)
      {
         return Long.MIN_VALUE;
      }
      else
      {
         return lastEstimatorStartTime.getLongValue() + controlDTInNS + estimatorDTInNS;
      }
   }

   private void checkForNewHandDesiredConfigurationRequested()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (handDesiredConfigurationMessageSubscribers.get(robotSide).isNewDesiredConfigurationAvailable())
         {
            HandConfiguration handDesiredConfiguration = HandConfiguration.fromByte(handDesiredConfigurationMessageSubscribers.get(robotSide).pollMessage()
                                                                                                                              .getDesiredHandConfiguration());
         }
      }
   }

   private void checkForNewFingerTrajectoryRequested()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (fingerTrajectoryMessageSubscribers.get(robotSide).isNewDesiredConfigurationAvailable())
         {
            FingerTrajectoryMessage fingerTrajectoryMessage = fingerTrajectoryMessageSubscribers.get(robotSide).pollMessage();
            Object<FingerMotorTrajectoryMessage> fingerMotorTrajectories = fingerTrajectoryMessage.getFingerMotorTrajectories();

            for (int i = 0; i < fingerMotorTrajectories.size(); i++)
            {
               FingerMotorTrajectoryMessage fingerMotorTrajectoryMessage = fingerMotorTrajectories.get(i);

               double delayTime = fingerMotorTrajectoryMessage.getDelayTime();
               TrajectoryPoint1DMessage trajectorypointMessage = fingerMotorTrajectoryMessage.getDesiredPosition();
               double trajectoryTime = trajectorypointMessage.getTime();
               double desiredPosition = trajectorypointMessage.getPosition();

               individualHandControllers.get(robotSide).executeTrajectory(sideListsOfValkyrieFingerMotorNames.get(robotSide).get(i), trajectoryTime, delayTime,
                                                                          desiredPosition);
            }
         }
      }
   }
}