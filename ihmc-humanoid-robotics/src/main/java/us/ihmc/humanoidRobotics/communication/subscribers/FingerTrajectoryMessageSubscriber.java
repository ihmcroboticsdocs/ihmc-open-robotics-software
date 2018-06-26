package us.ihmc.humanoidRobotics.communication.subscribers;

import java.util.concurrent.ConcurrentLinkedQueue;

import controller_msgs.msg.dds.FingerTrajectoryMessage;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.NewMessageListener;

public class FingerTrajectoryMessageSubscriber implements NewMessageListener<FingerTrajectoryMessage>
{
   private final ConcurrentLinkedQueue<FingerTrajectoryMessage> messageQueue = new ConcurrentLinkedQueue<FingerTrajectoryMessage>();
   private RobotSide robotSide;

   public FingerTrajectoryMessageSubscriber(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   @Override
   public void onNewDataMessage(Subscriber<FingerTrajectoryMessage> subscriber)
   {
      receivedPacket(subscriber.takeNextData());
   }

   public void receivedPacket(FingerTrajectoryMessage ihmcMessage)
   {
      if (this.robotSide == null)
         messageQueue.add(ihmcMessage);
      else if (ihmcMessage.getRobotSide() == this.robotSide.toByte())
         messageQueue.add(ihmcMessage);
   }

   public FingerTrajectoryMessage pollMessage()
   {
      return messageQueue.poll();
   }

   public boolean isNewDesiredConfigurationAvailable()
   {
      return !messageQueue.isEmpty();
   }

   public RobotSide getSide()
   {
      return robotSide;
   }
}