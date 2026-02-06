package vision_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Defines a classification result.
       * 
       * This result does not contain any position information. It is designed for
       * classifiers, which simply provide class probabilities given an instance of
       * source data (e.g., an image or a point cloud).
       */
public class Classification extends Packet<Classification> implements Settable<Classification>, EpsilonComparable<Classification>
{
   public std_msgs.msg.dds.Header header_;
   /**
            * A list of class probabilities. This list need not provide a probability for
            * every possible class, just ones that are nonzero, or above some
            * user-defined threshold.
            */
   public us.ihmc.idl.IDLSequence.Object<vision_msgs.msg.dds.ObjectHypothesis>  results_;

   public Classification()
   {
      header_ = new std_msgs.msg.dds.Header();
      results_ = new us.ihmc.idl.IDLSequence.Object<vision_msgs.msg.dds.ObjectHypothesis> (100, new vision_msgs.msg.dds.ObjectHypothesisPubSubType());

   }

   public Classification(Classification other)
   {
      this();
      set(other);
   }

   public void set(Classification other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      results_.set(other.results_);
   }


   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }


   /**
            * A list of class probabilities. This list need not provide a probability for
            * every possible class, just ones that are nonzero, or above some
            * user-defined threshold.
            */
   public us.ihmc.idl.IDLSequence.Object<vision_msgs.msg.dds.ObjectHypothesis>  getResults()
   {
      return results_;
   }


   public static Supplier<ClassificationPubSubType> getPubSubType()
   {
      return ClassificationPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ClassificationPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(Classification other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon)) return false;
      if (this.results_.size() != other.results_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.results_.size(); i++)
         {  if (!this.results_.get(i).epsilonEquals(other.results_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof Classification)) return false;

      Classification otherMyClass = (Classification) other;

      if (!this.header_.equals(otherMyClass.header_)) return false;
      if (!this.results_.equals(otherMyClass.results_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("Classification {");
      builder.append("header=");
      builder.append(this.header_);      builder.append(", ");
      builder.append("results=");
      builder.append(this.results_);
      builder.append("}");
      return builder.toString();
   }
}
