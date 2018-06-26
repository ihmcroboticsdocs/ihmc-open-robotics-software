package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message commands the controller to move fingers in motor control space to the desired motor position.
       */
public class FingerTrajectoryMessage extends Packet<FingerTrajectoryMessage> implements Settable<FingerTrajectoryMessage>, EpsilonComparable<FingerTrajectoryMessage>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Specifies the side of the robot that will execute the trajectory.
            */
   public byte robot_side_ = (byte) 255;
   /**
            * Trajectories for each finger motors.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.FingerMotorTrajectoryMessage>  finger_motor_trajectories_;

   public FingerTrajectoryMessage()
   {
      finger_motor_trajectories_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.FingerMotorTrajectoryMessage> (1, new controller_msgs.msg.dds.FingerMotorTrajectoryMessagePubSubType());

   }

   public FingerTrajectoryMessage(FingerTrajectoryMessage other)
   {
      this();
      set(other);
   }

   public void set(FingerTrajectoryMessage other)
   {
      sequence_id_ = other.sequence_id_;

      robot_side_ = other.robot_side_;

      finger_motor_trajectories_.set(other.finger_motor_trajectories_);
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
            * Specifies the side of the robot that will execute the trajectory.
            */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   /**
            * Specifies the side of the robot that will execute the trajectory.
            */
   public byte getRobotSide()
   {
      return robot_side_;
   }


   /**
            * Trajectories for each finger motors.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.FingerMotorTrajectoryMessage>  getFingerMotorTrajectories()
   {
      return finger_motor_trajectories_;
   }


   public static Supplier<FingerTrajectoryMessagePubSubType> getPubSubType()
   {
      return FingerTrajectoryMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FingerTrajectoryMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FingerTrajectoryMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (this.finger_motor_trajectories_.size() != other.finger_motor_trajectories_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.finger_motor_trajectories_.size(); i++)
         {  if (!this.finger_motor_trajectories_.get(i).epsilonEquals(other.finger_motor_trajectories_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FingerTrajectoryMessage)) return false;

      FingerTrajectoryMessage otherMyClass = (FingerTrajectoryMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if (!this.finger_motor_trajectories_.equals(otherMyClass.finger_motor_trajectories_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FingerTrajectoryMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("finger_motor_trajectories=");
      builder.append(this.finger_motor_trajectories_);
      builder.append("}");
      return builder.toString();
   }
}
