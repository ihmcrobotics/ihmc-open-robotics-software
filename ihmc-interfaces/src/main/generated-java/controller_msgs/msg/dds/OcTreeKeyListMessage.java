package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC robot environment awareness module.
       * This message contains a list of occupied cells in the octree
       * All keys are given at maximum depth
       */
public class OcTreeKeyListMessage extends Packet<OcTreeKeyListMessage> implements Settable<OcTreeKeyListMessage>, EpsilonComparable<OcTreeKeyListMessage>
{
   public int tree_depth_;
   public double tree_resolution_;
   public us.ihmc.idl.IDLSequence.Integer  x_keys_;
   public us.ihmc.idl.IDLSequence.Integer  y_keys_;
   public us.ihmc.idl.IDLSequence.Integer  z_keys_;

   public OcTreeKeyListMessage()
   {
      x_keys_ = new us.ihmc.idl.IDLSequence.Integer (5000, "type_2");

      y_keys_ = new us.ihmc.idl.IDLSequence.Integer (5000, "type_2");

      z_keys_ = new us.ihmc.idl.IDLSequence.Integer (5000, "type_2");

   }

   public OcTreeKeyListMessage(OcTreeKeyListMessage other)
   {
      this();
      set(other);
   }

   public void set(OcTreeKeyListMessage other)
   {
      tree_depth_ = other.tree_depth_;

      tree_resolution_ = other.tree_resolution_;

      x_keys_.set(other.x_keys_);
      y_keys_.set(other.y_keys_);
      z_keys_.set(other.z_keys_);
   }

   public void setTreeDepth(int tree_depth)
   {
      tree_depth_ = tree_depth;
   }
   public int getTreeDepth()
   {
      return tree_depth_;
   }

   public void setTreeResolution(double tree_resolution)
   {
      tree_resolution_ = tree_resolution;
   }
   public double getTreeResolution()
   {
      return tree_resolution_;
   }


   public us.ihmc.idl.IDLSequence.Integer  getXKeys()
   {
      return x_keys_;
   }


   public us.ihmc.idl.IDLSequence.Integer  getYKeys()
   {
      return y_keys_;
   }


   public us.ihmc.idl.IDLSequence.Integer  getZKeys()
   {
      return z_keys_;
   }


   public static Supplier<OcTreeKeyListMessagePubSubType> getPubSubType()
   {
      return OcTreeKeyListMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return OcTreeKeyListMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(OcTreeKeyListMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.tree_depth_, other.tree_depth_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.tree_resolution_, other.tree_resolution_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.x_keys_, other.x_keys_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.y_keys_, other.y_keys_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.z_keys_, other.z_keys_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof OcTreeKeyListMessage)) return false;

      OcTreeKeyListMessage otherMyClass = (OcTreeKeyListMessage) other;

      if(this.tree_depth_ != otherMyClass.tree_depth_) return false;

      if(this.tree_resolution_ != otherMyClass.tree_resolution_) return false;

      if (!this.x_keys_.equals(otherMyClass.x_keys_)) return false;
      if (!this.y_keys_.equals(otherMyClass.y_keys_)) return false;
      if (!this.z_keys_.equals(otherMyClass.z_keys_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("OcTreeKeyListMessage {");
      builder.append("tree_depth=");
      builder.append(this.tree_depth_);      builder.append(", ");
      builder.append("tree_resolution=");
      builder.append(this.tree_resolution_);      builder.append(", ");
      builder.append("x_keys=");
      builder.append(this.x_keys_);      builder.append(", ");
      builder.append("y_keys=");
      builder.append(this.y_keys_);      builder.append(", ");
      builder.append("z_keys=");
      builder.append(this.z_keys_);
      builder.append("}");
      return builder.toString();
   }
}
