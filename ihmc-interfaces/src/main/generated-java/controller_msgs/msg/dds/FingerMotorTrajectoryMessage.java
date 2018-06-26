package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class FingerMotorTrajectoryMessage extends Packet<FingerMotorTrajectoryMessage> implements Settable<FingerMotorTrajectoryMessage>, EpsilonComparable<FingerMotorTrajectoryMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Delay time for each finger motor trajectory
            */
   public double delay_time_;
   /**
            * Desired motor position
            */
   public controller_msgs.msg.dds.TrajectoryPoint1DMessage desired_position_;

   public FingerMotorTrajectoryMessage()
   {
      desired_position_ = new controller_msgs.msg.dds.TrajectoryPoint1DMessage();
   }

   public FingerMotorTrajectoryMessage(FingerMotorTrajectoryMessage other)
   {
      this();
      set(other);
   }

   public void set(FingerMotorTrajectoryMessage other)
   {
      sequence_id_ = other.sequence_id_;

      delay_time_ = other.delay_time_;

      controller_msgs.msg.dds.TrajectoryPoint1DMessagePubSubType.staticCopy(other.desired_position_, desired_position_);
   }

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }

   /**
            * Delay time for each finger motor trajectory
            */
   public void setDelayTime(double delay_time)
   {
      delay_time_ = delay_time;
   }
   /**
            * Delay time for each finger motor trajectory
            */
   public double getDelayTime()
   {
      return delay_time_;
   }


   /**
            * Desired motor position
            */
   public controller_msgs.msg.dds.TrajectoryPoint1DMessage getDesiredPosition()
   {
      return desired_position_;
   }


   public static Supplier<FingerMotorTrajectoryMessagePubSubType> getPubSubType()
   {
      return FingerMotorTrajectoryMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FingerMotorTrajectoryMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FingerMotorTrajectoryMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.delay_time_, other.delay_time_, epsilon)) return false;

      if (!this.desired_position_.epsilonEquals(other.desired_position_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FingerMotorTrajectoryMessage)) return false;

      FingerMotorTrajectoryMessage otherMyClass = (FingerMotorTrajectoryMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.delay_time_ != otherMyClass.delay_time_) return false;

      if (!this.desired_position_.equals(otherMyClass.desired_position_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FingerMotorTrajectoryMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("delay_time=");
      builder.append(this.delay_time_);      builder.append(", ");
      builder.append("desired_position=");
      builder.append(this.desired_position_);
      builder.append("}");
      return builder.toString();
   }
}
