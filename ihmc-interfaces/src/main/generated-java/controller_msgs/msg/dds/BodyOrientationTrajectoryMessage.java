package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * This message is part of the IHMC quadruped controller API.
 * This message commands the controller to move in taskspace the body to the desired orientation while going through the specified trajectory points.
 * A Hermite based curve (third order) is used to interpolate the orientations.
 * This message allows controlling the body orientation without interfering with position that will still be controlled to maintain the current desired capture point position.
 * To execute a normal trajectory to reach a desired body orientation, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time.
 */
public class BodyOrientationTrajectoryMessage extends Packet<BodyOrientationTrajectoryMessage>
      implements Settable<BodyOrientationTrajectoryMessage>, EpsilonComparable<BodyOrientationTrajectoryMessage>
{
   /**
    * Unique ID used to identify this message, should preferably be consecutively increasing.
    */
   public long sequence_id_;
   /**
    * Whether the pelvis orientation is allowed to be controlled by the user when the robot is walking.
    */
   public boolean enable_user_body_control_during_walking_;
   /**
    * The orientation trajectory information.
    */
   public controller_msgs.msg.dds.SO3TrajectoryMessage so3_trajectory_;

   public BodyOrientationTrajectoryMessage()
   {
      so3_trajectory_ = new controller_msgs.msg.dds.SO3TrajectoryMessage();
   }

   public BodyOrientationTrajectoryMessage(BodyOrientationTrajectoryMessage other)
   {
      this();
      set(other);
   }

   public void set(BodyOrientationTrajectoryMessage other)
   {
      sequence_id_ = other.sequence_id_;

      enable_user_body_control_during_walking_ = other.enable_user_body_control_during_walking_;

      controller_msgs.msg.dds.SO3TrajectoryMessagePubSubType.staticCopy(other.so3_trajectory_, so3_trajectory_);
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
    * Whether the pelvis orientation is allowed to be controlled by the user when the robot is walking.
    */
   public void setEnableUserBodyControlDuringWalking(boolean enable_user_body_control_during_walking)
   {
      enable_user_body_control_during_walking_ = enable_user_body_control_during_walking;
   }

   /**
    * Whether the pelvis orientation is allowed to be controlled by the user when the robot is walking.
    */
   public boolean getEnableUserBodyControlDuringWalking()
   {
      return enable_user_body_control_during_walking_;
   }

   /**
    * The orientation trajectory information.
    */
   public controller_msgs.msg.dds.SO3TrajectoryMessage getSo3Trajectory()
   {
      return so3_trajectory_;
   }

   @Override
   public boolean epsilonEquals(BodyOrientationTrajectoryMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_user_body_control_during_walking_, other.enable_user_body_control_during_walking_, epsilon))
         return false;

      if (!this.so3_trajectory_.epsilonEquals(other.so3_trajectory_, epsilon))
         return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof BodyOrientationTrajectoryMessage))
         return false;

      BodyOrientationTrajectoryMessage otherMyClass = (BodyOrientationTrajectoryMessage) other;

      if (this.sequence_id_ != otherMyClass.sequence_id_)
         return false;

      if (this.enable_user_body_control_during_walking_ != otherMyClass.enable_user_body_control_during_walking_)
         return false;

      if (!this.so3_trajectory_.equals(otherMyClass.so3_trajectory_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BodyOrientationTrajectoryMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);
      builder.append(", ");
      builder.append("enable_user_body_control_during_walking=");
      builder.append(this.enable_user_body_control_during_walking_);
      builder.append(", ");
      builder.append("so3_trajectory=");
      builder.append(this.so3_trajectory_);
      builder.append("}");
      return builder.toString();
   }
}
