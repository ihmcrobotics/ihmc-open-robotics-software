package vision_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * An object hypothesis that contains no pose information.
       * If you would like to define an array of ObjectHypothesis messages,
       * please see the Classification message type.
       */
public class ObjectHypothesis extends Packet<ObjectHypothesis> implements Settable<ObjectHypothesis>, EpsilonComparable<ObjectHypothesis>
{
   /**
            * The unique ID of the object class. To get additional information about
            * this ID, such as its human-readable class name, listeners should perform a
            * lookup in a metadata database. See vision_msgs/VisionInfo.msg for more detail.
            */
   public java.lang.StringBuilder class_id_;
   /**
            * The probability or confidence value of the detected object. By convention,
            * this value should lie in the range [0-1].
            */
   public double score_;

   public ObjectHypothesis()
   {
      class_id_ = new java.lang.StringBuilder(255);
   }

   public ObjectHypothesis(ObjectHypothesis other)
   {
      this();
      set(other);
   }

   public void set(ObjectHypothesis other)
   {
      class_id_.setLength(0);
      class_id_.append(other.class_id_);

      score_ = other.score_;

   }

   /**
            * The unique ID of the object class. To get additional information about
            * this ID, such as its human-readable class name, listeners should perform a
            * lookup in a metadata database. See vision_msgs/VisionInfo.msg for more detail.
            */
   public void setClassId(java.lang.String class_id)
   {
      class_id_.setLength(0);
      class_id_.append(class_id);
   }

   /**
            * The unique ID of the object class. To get additional information about
            * this ID, such as its human-readable class name, listeners should perform a
            * lookup in a metadata database. See vision_msgs/VisionInfo.msg for more detail.
            */
   public java.lang.String getClassIdAsString()
   {
      return getClassId().toString();
   }
   /**
            * The unique ID of the object class. To get additional information about
            * this ID, such as its human-readable class name, listeners should perform a
            * lookup in a metadata database. See vision_msgs/VisionInfo.msg for more detail.
            */
   public java.lang.StringBuilder getClassId()
   {
      return class_id_;
   }

   /**
            * The probability or confidence value of the detected object. By convention,
            * this value should lie in the range [0-1].
            */
   public void setScore(double score)
   {
      score_ = score;
   }
   /**
            * The probability or confidence value of the detected object. By convention,
            * this value should lie in the range [0-1].
            */
   public double getScore()
   {
      return score_;
   }


   public static Supplier<ObjectHypothesisPubSubType> getPubSubType()
   {
      return ObjectHypothesisPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ObjectHypothesisPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ObjectHypothesis other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.class_id_, other.class_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.score_, other.score_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ObjectHypothesis)) return false;

      ObjectHypothesis otherMyClass = (ObjectHypothesis) other;

      if (!us.ihmc.idl.IDLTools.equals(this.class_id_, otherMyClass.class_id_)) return false;

      if(this.score_ != otherMyClass.score_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ObjectHypothesis {");
      builder.append("class_id=");
      builder.append(this.class_id_);      builder.append(", ");
      builder.append("score=");
      builder.append(this.score_);
      builder.append("}");
      return builder.toString();
   }
}
