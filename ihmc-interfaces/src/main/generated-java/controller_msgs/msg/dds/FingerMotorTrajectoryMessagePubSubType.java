package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FingerMotorTrajectoryMessage" defined in "FingerMotorTrajectoryMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FingerMotorTrajectoryMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FingerMotorTrajectoryMessage_.idl instead.
*
*/
public class FingerMotorTrajectoryMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.FingerMotorTrajectoryMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::FingerMotorTrajectoryMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.FingerMotorTrajectoryMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.FingerMotorTrajectoryMessage data) throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   public static int getMaxCdrSerializedSize()
   {
      return getMaxCdrSerializedSize(0);
   }

   public static int getMaxCdrSerializedSize(int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += controller_msgs.msg.dds.TrajectoryPoint1DMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FingerMotorTrajectoryMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FingerMotorTrajectoryMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += controller_msgs.msg.dds.TrajectoryPoint1DMessagePubSubType.getCdrSerializedSize(data.getDesiredPosition(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.FingerMotorTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_6(data.getDelayTime());

      controller_msgs.msg.dds.TrajectoryPoint1DMessagePubSubType.write(data.getDesiredPosition(), cdr);
   }

   public static void read(controller_msgs.msg.dds.FingerMotorTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setDelayTime(cdr.read_type_6());
      	
      controller_msgs.msg.dds.TrajectoryPoint1DMessagePubSubType.read(data.getDesiredPosition(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.FingerMotorTrajectoryMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_6("delay_time", data.getDelayTime());
      ser.write_type_a("desired_position", new controller_msgs.msg.dds.TrajectoryPoint1DMessagePubSubType(), data.getDesiredPosition());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.FingerMotorTrajectoryMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setDelayTime(ser.read_type_6("delay_time"));
      ser.read_type_a("desired_position", new controller_msgs.msg.dds.TrajectoryPoint1DMessagePubSubType(), data.getDesiredPosition());

   }

   public static void staticCopy(controller_msgs.msg.dds.FingerMotorTrajectoryMessage src, controller_msgs.msg.dds.FingerMotorTrajectoryMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.FingerMotorTrajectoryMessage createData()
   {
      return new controller_msgs.msg.dds.FingerMotorTrajectoryMessage();
   }
   @Override
   public int getTypeSize()
   {
      return us.ihmc.idl.CDR.getTypeSize(getMaxCdrSerializedSize());
   }

   @Override
   public java.lang.String getName()
   {
      return name;
   }
   
   public void serialize(controller_msgs.msg.dds.FingerMotorTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.FingerMotorTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.FingerMotorTrajectoryMessage src, controller_msgs.msg.dds.FingerMotorTrajectoryMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FingerMotorTrajectoryMessagePubSubType newInstance()
   {
      return new FingerMotorTrajectoryMessagePubSubType();
   }
}
