package us.ihmc.robotDataLogger;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ReferenceFrameInformation extends Packet<ReferenceFrameInformation> implements Settable<ReferenceFrameInformation>, EpsilonComparable<ReferenceFrameInformation>
{
   public us.ihmc.idl.IDLSequence.Integer  frameVariables_;
   public us.ihmc.idl.IDLSequence.Long  frameIndeces_;
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  frameNames_;

   public ReferenceFrameInformation()
   {
      frameVariables_ = new us.ihmc.idl.IDLSequence.Integer (1024, "type_3");

      frameIndeces_ = new us.ihmc.idl.IDLSequence.Long (1024, "type_4");

      frameNames_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (1024, "type_d");
   }

   public ReferenceFrameInformation(ReferenceFrameInformation other)
   {
      this();
      set(other);
   }

   public void set(ReferenceFrameInformation other)
   {
      frameVariables_.set(other.frameVariables_);
      frameIndeces_.set(other.frameIndeces_);
      frameNames_.set(other.frameNames_);
   }


   public us.ihmc.idl.IDLSequence.Integer  getFrameVariables()
   {
      return frameVariables_;
   }


   public us.ihmc.idl.IDLSequence.Long  getFrameIndeces()
   {
      return frameIndeces_;
   }


   public us.ihmc.idl.IDLSequence.StringBuilderHolder  getFrameNames()
   {
      return frameNames_;
   }


   public static Supplier<ReferenceFrameInformationPubSubType> getPubSubType()
   {
      return ReferenceFrameInformationPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ReferenceFrameInformationPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ReferenceFrameInformation other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.frameVariables_, other.frameVariables_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsLongSequence(this.frameIndeces_, other.frameIndeces_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.frameNames_, other.frameNames_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ReferenceFrameInformation)) return false;

      ReferenceFrameInformation otherMyClass = (ReferenceFrameInformation) other;

      if (!this.frameVariables_.equals(otherMyClass.frameVariables_)) return false;
      if (!this.frameIndeces_.equals(otherMyClass.frameIndeces_)) return false;
      if (!this.frameNames_.equals(otherMyClass.frameNames_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ReferenceFrameInformation {");
      builder.append("frameVariables=");
      builder.append(this.frameVariables_);      builder.append(", ");
      builder.append("frameIndeces=");
      builder.append(this.frameIndeces_);      builder.append(", ");
      builder.append("frameNames=");
      builder.append(this.frameNames_);
      builder.append("}");
      return builder.toString();
   }
}
