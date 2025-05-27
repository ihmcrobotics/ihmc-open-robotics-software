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
            * Secondary reference frame (object) used as the point of view that defines the spatial relation with the object
            */
   public java.lang.StringBuilder pov_reference_frame_name_;
   /**
            * Reference frame (object) to go to
            */
   public java.lang.StringBuilder object_name_;
   /**
            * The distance to the reference frame (located at the object's centroid)
            */
   public double distance_to_object_;

   public AI2RNavigationMessage()
   {
      pov_reference_frame_name_ = new java.lang.StringBuilder(255);
      object_name_ = new java.lang.StringBuilder(255);
   }

   public AI2RNavigationMessage(AI2RNavigationMessage other)
   {
      this();
      set(other);
   }

   public void set(AI2RNavigationMessage other)
   {
      spatial_relation_ = other.spatial_relation_;

      pov_reference_frame_name_.setLength(0);
      pov_reference_frame_name_.append(other.pov_reference_frame_name_);

      object_name_.setLength(0);
      object_name_.append(other.object_name_);

      distance_to_object_ = other.distance_to_object_;

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
            * Secondary reference frame (object) used as the point of view that defines the spatial relation with the object
            */
   public void setPovReferenceFrameName(java.lang.String pov_reference_frame_name)
   {
      pov_reference_frame_name_.setLength(0);
      pov_reference_frame_name_.append(pov_reference_frame_name);
   }

   /**
            * Secondary reference frame (object) used as the point of view that defines the spatial relation with the object
            */
   public java.lang.String getPovReferenceFrameNameAsString()
   {
      return getPovReferenceFrameName().toString();
   }
   /**
            * Secondary reference frame (object) used as the point of view that defines the spatial relation with the object
            */
   public java.lang.StringBuilder getPovReferenceFrameName()
   {
      return pov_reference_frame_name_;
   }

   /**
            * Reference frame (object) to go to
            */
   public void setObjectName(java.lang.String object_name)
   {
      object_name_.setLength(0);
      object_name_.append(object_name);
   }

   /**
            * Reference frame (object) to go to
            */
   public java.lang.String getObjectNameAsString()
   {
      return getObjectName().toString();
   }
   /**
            * Reference frame (object) to go to
            */
   public java.lang.StringBuilder getObjectName()
   {
      return object_name_;
   }

   /**
            * The distance to the reference frame (located at the object's centroid)
            */
   public void setDistanceToObject(double distance_to_object)
   {
      distance_to_object_ = distance_to_object;
   }
   /**
            * The distance to the reference frame (located at the object's centroid)
            */
   public double getDistanceToObject()
   {
      return distance_to_object_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.pov_reference_frame_name_, other.pov_reference_frame_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.object_name_, other.object_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.distance_to_object_, other.distance_to_object_, epsilon)) return false;


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

      if (!us.ihmc.idl.IDLTools.equals(this.pov_reference_frame_name_, otherMyClass.pov_reference_frame_name_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.object_name_, otherMyClass.object_name_)) return false;

      if(this.distance_to_object_ != otherMyClass.distance_to_object_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AI2RNavigationMessage {");
      builder.append("spatial_relation=");
      builder.append(this.spatial_relation_);      builder.append(", ");
      builder.append("pov_reference_frame_name=");
      builder.append(this.pov_reference_frame_name_);      builder.append(", ");
      builder.append("object_name=");
      builder.append(this.object_name_);      builder.append(", ");
      builder.append("distance_to_object=");
      builder.append(this.distance_to_object_);
      builder.append("}");
      return builder.toString();
   }
}
