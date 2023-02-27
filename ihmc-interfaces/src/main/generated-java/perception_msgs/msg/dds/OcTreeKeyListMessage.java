package perception_msgs.msg.dds;

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
   public int number_of_keys_;
   /**
            * See us.ihmc.robotEnvironmentAwareness.communication.converters.OcTreeMessageConvterter for info on the compression protocol
            */
   public us.ihmc.idl.IDLSequence.Byte  keys_;

   public OcTreeKeyListMessage()
   {
      keys_ = new us.ihmc.idl.IDLSequence.Byte (200000, "type_9");

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

      number_of_keys_ = other.number_of_keys_;

      keys_.set(other.keys_);
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

   public void setNumberOfKeys(int number_of_keys)
   {
      number_of_keys_ = number_of_keys;
   }
   public int getNumberOfKeys()
   {
      return number_of_keys_;
   }


   /**
            * See us.ihmc.robotEnvironmentAwareness.communication.converters.OcTreeMessageConvterter for info on the compression protocol
            */
   public us.ihmc.idl.IDLSequence.Byte  getKeys()
   {
      return keys_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_keys_, other.number_of_keys_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.keys_, other.keys_, epsilon)) return false;


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

      if(this.number_of_keys_ != otherMyClass.number_of_keys_) return false;

      if (!this.keys_.equals(otherMyClass.keys_)) return false;

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
      builder.append("number_of_keys=");
      builder.append(this.number_of_keys_);      builder.append(", ");
      builder.append("keys=");
      builder.append(this.keys_);
      builder.append("}");
      return builder.toString();
   }
}
