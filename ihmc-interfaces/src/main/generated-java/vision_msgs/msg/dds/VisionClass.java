package vision_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * A key value pair that maps an integer class_id to a string class label
       * in computer vision systems.
       */
public class VisionClass extends Packet<VisionClass> implements Settable<VisionClass>, EpsilonComparable<VisionClass>
{
   /**
            * The int value that identifies the class.
            * Elements identified with 65535, the maximum uint16 value are assumed
            * to belong to the "UNLABELED" class. For vision pipelines using less
            * than 255 classes the "UNLABELED" is the maximum value in the uint8
            * range.
            */
   public int class_id_;
   /**
            * The name of the class represented by the class_id
            */
   public java.lang.StringBuilder class_name_;

   public VisionClass()
   {
      class_name_ = new java.lang.StringBuilder(255);
   }

   public VisionClass(VisionClass other)
   {
      this();
      set(other);
   }

   public void set(VisionClass other)
   {
      class_id_ = other.class_id_;

      class_name_.setLength(0);
      class_name_.append(other.class_name_);

   }

   /**
            * The int value that identifies the class.
            * Elements identified with 65535, the maximum uint16 value are assumed
            * to belong to the "UNLABELED" class. For vision pipelines using less
            * than 255 classes the "UNLABELED" is the maximum value in the uint8
            * range.
            */
   public void setClassId(int class_id)
   {
      class_id_ = class_id;
   }
   /**
            * The int value that identifies the class.
            * Elements identified with 65535, the maximum uint16 value are assumed
            * to belong to the "UNLABELED" class. For vision pipelines using less
            * than 255 classes the "UNLABELED" is the maximum value in the uint8
            * range.
            */
   public int getClassId()
   {
      return class_id_;
   }

   /**
            * The name of the class represented by the class_id
            */
   public void setClassName(java.lang.String class_name)
   {
      class_name_.setLength(0);
      class_name_.append(class_name);
   }

   /**
            * The name of the class represented by the class_id
            */
   public java.lang.String getClassNameAsString()
   {
      return getClassName().toString();
   }
   /**
            * The name of the class represented by the class_id
            */
   public java.lang.StringBuilder getClassName()
   {
      return class_name_;
   }


   public static Supplier<VisionClassPubSubType> getPubSubType()
   {
      return VisionClassPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return VisionClassPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(VisionClass other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.class_id_, other.class_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.class_name_, other.class_name_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof VisionClass)) return false;

      VisionClass otherMyClass = (VisionClass) other;

      if(this.class_id_ != otherMyClass.class_id_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.class_name_, otherMyClass.class_name_)) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("VisionClass {");
      builder.append("class_id=");
      builder.append(this.class_id_);      builder.append(", ");
      builder.append("class_name=");
      builder.append(this.class_name_);
      builder.append("}");
      return builder.toString();
   }
}
