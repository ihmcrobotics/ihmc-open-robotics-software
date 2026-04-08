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
            * Point of view object. Secondary reference frame (object) used as the point of view that defines the spatial relation with the object
            */
   public java.lang.StringBuilder pov_object_;
   /**
            * Target reference frame (object) to go to
            */
   public java.lang.StringBuilder target_object_;
   /**
            * The distance to the reference frame (located at the object's centroid)
            */
   public double distance_to_object_;
   /**
            * The minimum distance to the obstacles considered in body path planning
            */
   public double obstacle_clearance_;

   public AI2RNavigationMessage()
   {
      pov_object_ = new java.lang.StringBuilder(255);
      target_object_ = new java.lang.StringBuilder(255);
   }

   public AI2RNavigationMessage(AI2RNavigationMessage other)
   {
      this();
      set(other);
   }

   public void set(AI2RNavigationMessage other)
   {
      spatial_relation_ = other.spatial_relation_;

      pov_object_.setLength(0);
      pov_object_.append(other.pov_object_);

      target_object_.setLength(0);
      target_object_.append(other.target_object_);

      distance_to_object_ = other.distance_to_object_;

      obstacle_clearance_ = other.obstacle_clearance_;

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
            * Point of view object. Secondary reference frame (object) used as the point of view that defines the spatial relation with the object
            */
   public void setPovObject(java.lang.String pov_object)
   {
      pov_object_.setLength(0);
      pov_object_.append(pov_object);
   }

   /**
            * Point of view object. Secondary reference frame (object) used as the point of view that defines the spatial relation with the object
            */
   public java.lang.String getPovObjectAsString()
   {
      return getPovObject().toString();
   }
   /**
            * Point of view object. Secondary reference frame (object) used as the point of view that defines the spatial relation with the object
            */
   public java.lang.StringBuilder getPovObject()
   {
      return pov_object_;
   }

   /**
            * Target reference frame (object) to go to
            */
   public void setTargetObject(java.lang.String target_object)
   {
      target_object_.setLength(0);
      target_object_.append(target_object);
   }

   /**
            * Target reference frame (object) to go to
            */
   public java.lang.String getTargetObjectAsString()
   {
      return getTargetObject().toString();
   }
   /**
            * Target reference frame (object) to go to
            */
   public java.lang.StringBuilder getTargetObject()
   {
      return target_object_;
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

   /**
            * The minimum distance to the obstacles considered in body path planning
            */
   public void setObstacleClearance(double obstacle_clearance)
   {
      obstacle_clearance_ = obstacle_clearance;
   }
   /**
            * The minimum distance to the obstacles considered in body path planning
            */
   public double getObstacleClearance()
   {
      return obstacle_clearance_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.pov_object_, other.pov_object_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.target_object_, other.target_object_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.distance_to_object_, other.distance_to_object_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.obstacle_clearance_, other.obstacle_clearance_, epsilon)) return false;


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

      if (!us.ihmc.idl.IDLTools.equals(this.pov_object_, otherMyClass.pov_object_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.target_object_, otherMyClass.target_object_)) return false;

      if(this.distance_to_object_ != otherMyClass.distance_to_object_) return false;

      if(this.obstacle_clearance_ != otherMyClass.obstacle_clearance_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AI2RNavigationMessage {");
      builder.append("spatial_relation=");
      builder.append(this.spatial_relation_);      builder.append(", ");
      builder.append("pov_object=");
      builder.append(this.pov_object_);      builder.append(", ");
      builder.append("target_object=");
      builder.append(this.target_object_);      builder.append(", ");
      builder.append("distance_to_object=");
      builder.append(this.distance_to_object_);      builder.append(", ");
      builder.append("obstacle_clearance=");
      builder.append(this.obstacle_clearance_);
      builder.append("}");
      return builder.toString();
   }
}
