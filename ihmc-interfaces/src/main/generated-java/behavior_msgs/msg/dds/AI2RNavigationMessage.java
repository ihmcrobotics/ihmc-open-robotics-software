package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class AI2RNavigationMessage extends Packet<AI2RNavigationMessage> implements Settable<AI2RNavigationMessage>, EpsilonComparable<AI2RNavigationMessage>
{
   /**
          * SPATIAL RELATION TYPE
          */
   public static final byte DEFAULT = (byte) 0;
   public static final byte FRONT = (byte) 1;
   public static final byte BEHIND = (byte) 2;
   public static final byte LEFT = (byte) 3;
   public static final byte RIGHT = (byte) 4;
   /**
            * The type of spatial relation with the secondary reference object as defined above
            */
   public byte spatial_relation_;
   /**
            * Goto action - Secondary reference object used to defined the spatial relation with reference_frame_name
            */
   public java.lang.StringBuilder secondary_reference_frame_name_;
   /**
            * Goto action - Reference frame for the action
            */
   public java.lang.StringBuilder reference_frame_name_;
   /**
            * Goto action - The distance to the frame
            */
   public double distance_to_frame_;

   public AI2RNavigationMessage()
   {
      secondary_reference_frame_name_ = new java.lang.StringBuilder(255);
      reference_frame_name_ = new java.lang.StringBuilder(255);
   }

   public AI2RNavigationMessage(AI2RNavigationMessage other)
   {
      this();
      set(other);
   }

   public void set(AI2RNavigationMessage other)
   {
      spatial_relation_ = other.spatial_relation_;

      secondary_reference_frame_name_.setLength(0);
      secondary_reference_frame_name_.append(other.secondary_reference_frame_name_);

      reference_frame_name_.setLength(0);
      reference_frame_name_.append(other.reference_frame_name_);

      distance_to_frame_ = other.distance_to_frame_;

   }

   /**
            * The type of spatial relation with the secondary reference object as defined above
            */
   public void setSpatialRelation(byte spatial_relation)
   {
      spatial_relation_ = spatial_relation;
   }
   /**
            * The type of spatial relation with the secondary reference object as defined above
            */
   public byte getSpatialRelation()
   {
      return spatial_relation_;
   }

   /**
            * Goto action - Secondary reference object used to defined the spatial relation with reference_frame_name
            */
   public void setSecondaryReferenceFrameName(java.lang.String secondary_reference_frame_name)
   {
      secondary_reference_frame_name_.setLength(0);
      secondary_reference_frame_name_.append(secondary_reference_frame_name);
   }

   /**
            * Goto action - Secondary reference object used to defined the spatial relation with reference_frame_name
            */
   public java.lang.String getSecondaryReferenceFrameNameAsString()
   {
      return getSecondaryReferenceFrameName().toString();
   }
   /**
            * Goto action - Secondary reference object used to defined the spatial relation with reference_frame_name
            */
   public java.lang.StringBuilder getSecondaryReferenceFrameName()
   {
      return secondary_reference_frame_name_;
   }

   /**
            * Goto action - Reference frame for the action
            */
   public void setReferenceFrameName(java.lang.String reference_frame_name)
   {
      reference_frame_name_.setLength(0);
      reference_frame_name_.append(reference_frame_name);
   }

   /**
            * Goto action - Reference frame for the action
            */
   public java.lang.String getReferenceFrameNameAsString()
   {
      return getReferenceFrameName().toString();
   }
   /**
            * Goto action - Reference frame for the action
            */
   public java.lang.StringBuilder getReferenceFrameName()
   {
      return reference_frame_name_;
   }

   /**
            * Goto action - The distance to the frame
            */
   public void setDistanceToFrame(double distance_to_frame)
   {
      distance_to_frame_ = distance_to_frame;
   }
   /**
            * Goto action - The distance to the frame
            */
   public double getDistanceToFrame()
   {
      return distance_to_frame_;
   }


   public static Supplier<AI2RNavigationMessagePubSubType> getPubSubType()
   {
      return AI2RNavigationMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AI2RNavigationMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AI2RNavigationMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.spatial_relation_, other.spatial_relation_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.secondary_reference_frame_name_, other.secondary_reference_frame_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.reference_frame_name_, other.reference_frame_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.distance_to_frame_, other.distance_to_frame_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AI2RNavigationMessage)) return false;

      AI2RNavigationMessage otherMyClass = (AI2RNavigationMessage) other;

      if(this.spatial_relation_ != otherMyClass.spatial_relation_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.secondary_reference_frame_name_, otherMyClass.secondary_reference_frame_name_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.reference_frame_name_, otherMyClass.reference_frame_name_)) return false;

      if(this.distance_to_frame_ != otherMyClass.distance_to_frame_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AI2RNavigationMessage {");
      builder.append("spatial_relation=");
      builder.append(this.spatial_relation_);      builder.append(", ");
      builder.append("secondary_reference_frame_name=");
      builder.append(this.secondary_reference_frame_name_);      builder.append(", ");
      builder.append("reference_frame_name=");
      builder.append(this.reference_frame_name_);      builder.append(", ");
      builder.append("distance_to_frame=");
      builder.append(this.distance_to_frame_);
      builder.append("}");
      return builder.toString();
   }
}
