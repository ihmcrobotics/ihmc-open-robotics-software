package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * This message is part of the IHMC whole-body controller API.
 * This message commands the controller to start loading a foot that was unloaded to support the robot weight.
 * When the robot is performing a 'flamingo stance' (one foot in the air not actually walking) and the user wants the robot to switch back to double support.
 */
public class FootLoadBearingMessage extends Packet<FootLoadBearingMessage>
      implements Settable<FootLoadBearingMessage>, EpsilonComparable<FootLoadBearingMessage>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   public static final byte LOAD_BEARING_REQUEST_LOAD = (byte) 0;
   public static final byte LOAD_BEARING_REQUEST_UNLOAD = (byte) 1;
   /**
    * Needed to identify a side dependent end-effector.
    */
   public byte robot_side_ = (byte) 255;
   /**
    * Whether the end-effector should be loaded or unloaded.
    */
   public byte load_bearing_request_;
   /**
    * The time to delay this command on the controller side before being executed.
    */
   public double execution_delay_time_;

   public FootLoadBearingMessage()
   {

   }

   public FootLoadBearingMessage(FootLoadBearingMessage other)
   {
      set(other);
   }

   public void set(FootLoadBearingMessage other)
   {
      robot_side_ = other.robot_side_;

      load_bearing_request_ = other.load_bearing_request_;

      execution_delay_time_ = other.execution_delay_time_;
   }

   /**
    * Needed to identify a side dependent end-effector.
    */
   public byte getRobotSide()
   {
      return robot_side_;
   }

   /**
    * Needed to identify a side dependent end-effector.
    */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }

   /**
    * Whether the end-effector should be loaded or unloaded.
    */
   public byte getLoadBearingRequest()
   {
      return load_bearing_request_;
   }

   /**
    * Whether the end-effector should be loaded or unloaded.
    */
   public void setLoadBearingRequest(byte load_bearing_request)
   {
      load_bearing_request_ = load_bearing_request;
   }

   /**
    * The time to delay this command on the controller side before being executed.
    */
   public double getExecutionDelayTime()
   {
      return execution_delay_time_;
   }

   /**
    * The time to delay this command on the controller side before being executed.
    */
   public void setExecutionDelayTime(double execution_delay_time)
   {
      execution_delay_time_ = execution_delay_time;
   }

   @Override
   public boolean epsilonEquals(FootLoadBearingMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.load_bearing_request_, other.load_bearing_request_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.execution_delay_time_, other.execution_delay_time_, epsilon))
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
      if (!(other instanceof FootLoadBearingMessage))
         return false;

      FootLoadBearingMessage otherMyClass = (FootLoadBearingMessage) other;

      if (this.robot_side_ != otherMyClass.robot_side_)
         return false;

      if (this.load_bearing_request_ != otherMyClass.load_bearing_request_)
         return false;

      if (this.execution_delay_time_ != otherMyClass.execution_delay_time_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootLoadBearingMessage {");
      builder.append("robot_side=");
      builder.append(this.robot_side_);

      builder.append(", ");
      builder.append("load_bearing_request=");
      builder.append(this.load_bearing_request_);

      builder.append(", ");
      builder.append("execution_delay_time=");
      builder.append(this.execution_delay_time_);

      builder.append("}");
      return builder.toString();
   }
}