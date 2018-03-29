package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * Atlas specific message. This packet controls the stiffness of LegJoints. i.e., the maximum force
 * a joint puts out when it is (pushed) away from a desired position. This is useful to prevent
 * robot from falling when leg hit things unexpectedly. However, low stiffness (high compliance) can
 * lead to poor joint tracking in the presence of natural joint stiction. Finding a good balance is
 * application specific. In our hybrid velocity+force controller, most force comes from velocity
 * control, therefore, only parameter related to velocity control is exposed.
 */
public class LegCompliancePacket extends Packet<LegCompliancePacket> implements Settable<LegCompliancePacket>, EpsilonComparable<LegCompliancePacket>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   /**
    * maximum allowed force (ratio) from velocity control in the range of [0.0, 1.0]. 1.0 is the
    * maximum stiffness (default) value tuned for fast walking, 0.0 refers to zero velocity control
    * making the joint very compliant (only force control) but often bad tracking. On Atlas, 0.1-0.3
    * gives decent tracking for slow motion and yet still compliant. The numbers in the array
    * correspond to joints HPZ, HPX, HPY, KNY, AKY, AKX, respectively.
    */
   public us.ihmc.idl.IDLSequence.Float max_velocity_deltas_;
   public byte robot_side_ = (byte) 255;

   public LegCompliancePacket()
   {
      header_ = new std_msgs.msg.dds.Header();
      max_velocity_deltas_ = new us.ihmc.idl.IDLSequence.Float(100, "type_5");

   }

   public LegCompliancePacket(LegCompliancePacket other)
   {
      this();
      set(other);
   }

   public void set(LegCompliancePacket other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      max_velocity_deltas_.set(other.max_velocity_deltas_);
      robot_side_ = other.robot_side_;

   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   /**
    * maximum allowed force (ratio) from velocity control in the range of [0.0, 1.0]. 1.0 is the
    * maximum stiffness (default) value tuned for fast walking, 0.0 refers to zero velocity control
    * making the joint very compliant (only force control) but often bad tracking. On Atlas, 0.1-0.3
    * gives decent tracking for slow motion and yet still compliant. The numbers in the array
    * correspond to joints HPZ, HPX, HPY, KNY, AKY, AKX, respectively.
    */
   public us.ihmc.idl.IDLSequence.Float getMaxVelocityDeltas()
   {
      return max_velocity_deltas_;
   }

   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }

   public byte getRobotSide()
   {
      return robot_side_;
   }

   @Override
   public boolean epsilonEquals(LegCompliancePacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.max_velocity_deltas_, other.max_velocity_deltas_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon))
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
      if (!(other instanceof LegCompliancePacket))
         return false;

      LegCompliancePacket otherMyClass = (LegCompliancePacket) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (!this.max_velocity_deltas_.equals(otherMyClass.max_velocity_deltas_))
         return false;
      if (this.robot_side_ != otherMyClass.robot_side_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("LegCompliancePacket {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("max_velocity_deltas=");
      builder.append(this.max_velocity_deltas_);
      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);
      builder.append("}");
      return builder.toString();
   }
}
