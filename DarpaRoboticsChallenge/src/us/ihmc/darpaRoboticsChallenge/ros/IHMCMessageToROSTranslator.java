package us.ihmc.darpaRoboticsChallenge.ros;

import ihmc_msgs.FootstepDataListRosMessage;
import ihmc_msgs.FootstepDataRosMessage;
import ihmc_msgs.Point2dRosMessage;
import org.ros.internal.message.Message;
import org.ros.message.MessageFactory;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.utilities.ros.RosMessageGenerationTools;
import us.ihmc.utilities.ros.msgToPacket.converter.GenericRosMessageConverter;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;

public class IHMCMessageToROSTranslator
{
   private static final MessageFactory messageFactory = GenericRosMessageConverter.getMessageFactory();
   //   private static final MessageFactory messageFactory = NodeConfiguration.newPrivate().getTopicMessageFactory();

   public static Message convertToRosMessage(Packet<?> ihmcMessage) throws Exception
   {
      Class<? extends Packet> aClass = ihmcMessage.getClass();
      try
      {
         Method convertToRosMessageMethod = IHMCMessageToROSTranslator.class.getDeclaredMethod("customConvertToRosMessage", aClass);
         convertToRosMessageMethod.setAccessible(true);
         return (Message) convertToRosMessageMethod.invoke(null, ihmcMessage);
      }
      catch (NoSuchMethodException exception)
      {
         return GenericRosMessageConverter.convertIHMCMessageToRosMessage(ihmcMessage);
      }
   }

   public static Packet<?> convertToIHMCMessage(Message rosMessage) throws Exception
   {
      Class<?> aClass = Class.forName(rosMessage.toRawMessage().getType().replace("/", "."));

      try
      {
         Method convertToIHMCMessageMethod = IHMCMessageToROSTranslator.class.getDeclaredMethod("customConvertToIHMCMessage", aClass);
         convertToIHMCMessageMethod.setAccessible(true);
         return (Packet<?>) convertToIHMCMessageMethod.invoke(null, rosMessage);
      }
      catch (NoSuchMethodException exception)
      {
         return GenericRosMessageConverter.convertRosMessageToIHMCMessage(rosMessage);
      }
   }

   private static Packet customConvertToIHMCMessage(FootstepDataListRosMessage message) throws Exception
   {
      FootstepDataListMessage footsteps = new FootstepDataListMessage();

      footsteps.swingTime = message.getSwingTime();
      footsteps.transferTime = message.getTransferTime();
      footsteps.setUniqueId(message.getUniqueId());

      ArrayList<FootstepDataMessage> stepData = new ArrayList<>();
      for (FootstepDataRosMessage footstepDataRosMessage : message.getFootstepDataList())
      {
         stepData.add((FootstepDataMessage) convertToIHMCMessage(footstepDataRosMessage));
      }

      footsteps.footstepDataList = stepData;

      return footsteps;
   }

   private static Packet customConvertToIHMCMessage(FootstepDataRosMessage message) throws Exception
   {
      FootstepDataMessage ihmcMessage = new FootstepDataMessage();

      ihmcMessage.setOrigin(FootstepDataMessage.FootstepOrigin.values()[message.getOrigin()]);
      ihmcMessage.setRobotSide(RobotSide.values[message.getRobotSide()]);
      ihmcMessage.setLocation(new Point3d(GenericRosMessageConverter.convertVector3(message.getLocation())));
      ihmcMessage.setOrientation(new Quat4d(GenericRosMessageConverter.convertQuaternion(message.getOrientation())));
      ihmcMessage.setSwingHeight(message.getSwingHeight());
      ihmcMessage.setTrajectoryType(TrajectoryType.values()[message.getTrajectoryType()]);
      ihmcMessage.setUniqueId(message.getUniqueId());

      ArrayList<Point2d> predictedContactPoints = new ArrayList<>();
      for (Point2dRosMessage point2dRosMessage : message.getPredictedContactPoints())
      {
         predictedContactPoints.add(GenericRosMessageConverter.convertPoint2DRos(point2dRosMessage));
      }

      ihmcMessage.setPredictedContactPoints(predictedContactPoints);

      return ihmcMessage;
   }

   private static Message customConvertToRosMessage(FootstepDataMessage footstep) throws Exception
   {
      Class<? extends Packet> ihmcMessageClass = FootstepDataMessage.class;
      String rosMessageClassNameFromIHMCMessage = RosMessageGenerationTools.getRosMessageClassNameFromIHMCMessage(ihmcMessageClass.getSimpleName());
      RosMessagePacket rosAnnotation = ihmcMessageClass.getAnnotation(RosMessagePacket.class);

      FootstepDataRosMessage message = messageFactory.newFromType(rosAnnotation.rosPackage() + "/" + rosMessageClassNameFromIHMCMessage);

      message.setUniqueId(footstep.getUniqueId());
      message.setLocation(GenericRosMessageConverter.convertTuple3d(footstep.getLocation()));
      message.setOrientation(GenericRosMessageConverter.convertTuple4d(footstep.getOrientation()));
      message.setOrigin((byte) footstep.getOrigin().ordinal());
      message.setRobotSide((byte) footstep.getOrigin().ordinal());
      message.setSwingHeight(footstep.getSwingHeight());
      message.setTrajectoryType((byte) footstep.getTrajectoryType().ordinal());

      List<Point2dRosMessage> predictedContatcPointsRos = new ArrayList<>();
      if(footstep.predictedContactPoints != null)
      {
         for (Point2d predictedContactPoint : footstep.predictedContactPoints)
         {
            predictedContatcPointsRos.add(GenericRosMessageConverter.convertPoint2d(predictedContactPoint));
         }
      }

      message.setPredictedContactPoints(predictedContatcPointsRos);

      return message;
   }

   private static Message customConvertToRosMessage(FootstepDataListMessage footstepList) throws Exception
   {
      Class<? extends Packet> ihmcMessageClass = FootstepDataListMessage.class;
      String rosMessageClassNameFromIHMCMessage = RosMessageGenerationTools.getRosMessageClassNameFromIHMCMessage(ihmcMessageClass.getSimpleName());
      RosMessagePacket rosAnnotation = ihmcMessageClass.getAnnotation(RosMessagePacket.class);

      FootstepDataListRosMessage message = messageFactory.newFromType(rosAnnotation.rosPackage() + "/" + rosMessageClassNameFromIHMCMessage);

      message.setSwingTime(footstepList.swingTime);
      message.setTransferTime(footstepList.transferTime);
      message.setUniqueId(footstepList.getUniqueId());

      List<FootstepDataRosMessage> convertedFootsteps = new ArrayList<>();
      for (FootstepDataMessage footstepDataMessage : footstepList.footstepDataList)
      {
         convertedFootsteps.add((FootstepDataRosMessage) convertToRosMessage(footstepDataMessage));
      }

      message.setFootstepDataList(convertedFootsteps);

      return message;
   }

   public static void main(String[] args) throws Exception
   {
      ArrayList<FootstepDataMessage> steps = new ArrayList<>();
      for(int i = 0; i < 10; i++)
      {
         FootstepDataMessage data = new FootstepDataMessage(RobotSide.values[i % 2], new Point3d(0, 0, 0), new Quat4d(0, 0, 0, 1));
         ArrayList<Point2d> contactPoints = new ArrayList<>();
         contactPoints.add(new Point2d(0, 0));
         contactPoints.add(new Point2d(1, 0));
         contactPoints.add(new Point2d(1, 1));
         contactPoints.add(new Point2d(0, 1));
         data.setPredictedContactPoints(contactPoints);
         steps.add(data);
      }

      FootstepDataListMessage footstepDataMessages = new FootstepDataListMessage(steps, 1.0, 1.0);

      Message message = IHMCMessageToROSTranslator.convertToRosMessage(footstepDataMessages);

//      System.out.println("Message: " + message);

      Packet<?> packet = IHMCMessageToROSTranslator.convertToIHMCMessage(message);

//      System.out.println("Packet: " + packet);
   }
}
