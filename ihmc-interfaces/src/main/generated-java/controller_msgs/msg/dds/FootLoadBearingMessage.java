package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * This message is part of the IHMC whole-body controller API. This message commands the controller
 * to start loading a foot that was unloaded to support the robot weight. When the robot is
 * performing a 'flamingo stance' (one foot in the air not actually walking) and the user wants the
 * robot to switch back to double support.
 */
public class FootLoadBearingMessage extends Packet<FootLoadBearingMessage>
      implements Settable<FootLoadBearingMessage>, EpsilonComparable<FootLoadBearingMessage>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   public static final byte LOAD_BEARING_REQUEST_LOAD = (byte) 0;
   public static final byte LOAD_BEARING_REQUEST_UNLOAD = (byte) 1;
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
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
      header_ = new std_msgs.msg.dds.Header();
   }

   public FootLoadBearingMessage(FootLoadBearingMessage other)
   {
      this();
      set(other);
   }

   public void set(FootLoadBearingMessage other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      robot_side_ = other.robot_side_;

      load_bearing_request_ = other.load_bearing_request_;

      execution_delay_time_ = other.execution_delay_time_;

   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   /**
    * Needed to identify a side dependent end-effector.
    */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }

   /**
    * Needed to identify a side dependent end-effector.
    */
   public byte getRobotSide()
   {
      return robot_side_;
   }

   /**
    * Whether the end-effector should be loaded or unloaded.
    */
   public void setLoadBearingRequest(byte load_bearing_request)
   {
      load_bearing_request_ = load_bearing_request;
   }

   /**
    * Whether the end-effector should be loaded or unloaded.
    */
   public byte getLoadBearingRequest()
   {
      return load_bearing_request_;
   }

   /**
    * The time to delay this command on the controller side before being executed.
    */
   public void setExecutionDelayTime(double execution_delay_time)
   {
      execution_delay_time_ = execution_delay_time;
   }

   /**
    * The time to delay this command on the controller side before being executed.
    */
   public double getExecutionDelayTime()
   {
      return execution_delay_time_;
   }

   @Override
   public boolean epsilonEquals(FootLoadBearingMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
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

      if (!this.header_.equals(otherMyClass.header_))
         return false;
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
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
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
