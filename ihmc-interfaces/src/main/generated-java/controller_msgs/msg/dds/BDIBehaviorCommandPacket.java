package controller_msgs.msg.dds;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * Atlas specific message.
 */
public class BDIBehaviorCommandPacket implements Settable<BDIBehaviorCommandPacket>, EpsilonComparable<BDIBehaviorCommandPacket>
{
   public static final byte NONE = (byte) 0;
   public static final byte FREEZE = (byte) 1;
   public static final byte STAND_PREP = (byte) 2;
   public static final byte STAND = (byte) 3;
   public static final byte WALK = (byte) 4;
   public static final byte STEP = (byte) 5;
   public static final byte MANIPULATE = (byte) 6;
   public static final byte USER = (byte) 7;
   public static final byte CALIBRATE = (byte) 8;
   public static final byte SOFT_STOP = (byte) 9;
   private byte atlas_bdi_robot_behavior_ = (byte) 255;
   private boolean stop_;

   public BDIBehaviorCommandPacket()
   {

   }

   public BDIBehaviorCommandPacket(BDIBehaviorCommandPacket other)
   {
      set(other);
   }

   public void set(BDIBehaviorCommandPacket other)
   {
      atlas_bdi_robot_behavior_ = other.atlas_bdi_robot_behavior_;

      stop_ = other.stop_;
   }

   public byte getAtlasBdiRobotBehavior()
   {
      return atlas_bdi_robot_behavior_;
   }

   public void setAtlasBdiRobotBehavior(byte atlas_bdi_robot_behavior)
   {
      atlas_bdi_robot_behavior_ = atlas_bdi_robot_behavior;
   }

   public boolean getStop()
   {
      return stop_;
   }

   public void setStop(boolean stop)
   {
      stop_ = stop;
   }

   @Override
   public boolean epsilonEquals(BDIBehaviorCommandPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.atlas_bdi_robot_behavior_, other.atlas_bdi_robot_behavior_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.stop_, other.stop_, epsilon))
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
      if (!(other instanceof BDIBehaviorCommandPacket))
         return false;

      BDIBehaviorCommandPacket otherMyClass = (BDIBehaviorCommandPacket) other;

      if (this.atlas_bdi_robot_behavior_ != otherMyClass.atlas_bdi_robot_behavior_)
         return false;

      if (this.stop_ != otherMyClass.stop_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BDIBehaviorCommandPacket {");
      builder.append("atlas_bdi_robot_behavior=");
      builder.append(this.atlas_bdi_robot_behavior_);

      builder.append(", ");
      builder.append("stop=");
      builder.append(this.stop_);

      builder.append("}");
      return builder.toString();
   }
}