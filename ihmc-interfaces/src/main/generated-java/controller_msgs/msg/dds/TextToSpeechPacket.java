package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Use this message for the robot to speak.
       */
public class TextToSpeechPacket extends Packet<TextToSpeechPacket> implements Settable<TextToSpeechPacket>, EpsilonComparable<TextToSpeechPacket>
{

   public static final String FOOTSTEP_COMPLETED = "Finished Taking Footstep";


   public static final String FINISHED_WALKING = "Finished walking";


   public static final String WALKING_ABORTED = "walking aborted";


   public static final String STARTING_CONTROLLER = "starting controller";


   public static final String WALKING = "walking";


   public static final String MOVING_LEFT_ARM = "moving the left arm";


   public static final String MOVING_RIGHT_ARM = "moving the right arm";


   public static final String NETWORKPROCESSOR_ONLINE = "Reestablished Connection To The Network Processor";


   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public boolean speak_packet_;

   public boolean beep_ = true;

   public java.lang.StringBuilder text_to_speak_;

   public TextToSpeechPacket()
   {




      text_to_speak_ = new java.lang.StringBuilder(255);

   }

   public TextToSpeechPacket(TextToSpeechPacket other)
   {
      this();
      set(other);
   }

   public void set(TextToSpeechPacket other)
   {

      sequence_id_ = other.sequence_id_;


      speak_packet_ = other.speak_packet_;


      beep_ = other.beep_;


      text_to_speak_.setLength(0);
      text_to_speak_.append(other.text_to_speak_);

   }


   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }


   public void setSpeakPacket(boolean speak_packet)
   {
      speak_packet_ = speak_packet;
   }
   public boolean getSpeakPacket()
   {
      return speak_packet_;
   }


   public void setBeep(boolean beep)
   {
      beep_ = beep;
   }
   public boolean getBeep()
   {
      return beep_;
   }


   public void setTextToSpeak(java.lang.String text_to_speak)
   {
      text_to_speak_.setLength(0);
      text_to_speak_.append(text_to_speak);
   }

   public java.lang.String getTextToSpeakAsString()
   {
      return getTextToSpeak().toString();
   }
   public java.lang.StringBuilder getTextToSpeak()
   {
      return text_to_speak_;
   }


   public static Supplier<TextToSpeechPacketPubSubType> getPubSubType()
   {
      return TextToSpeechPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return TextToSpeechPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(TextToSpeechPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.speak_packet_, other.speak_packet_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.beep_, other.beep_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.text_to_speak_, other.text_to_speak_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof TextToSpeechPacket)) return false;

      TextToSpeechPacket otherMyClass = (TextToSpeechPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.speak_packet_ != otherMyClass.speak_packet_) return false;


      if(this.beep_ != otherMyClass.beep_) return false;


      if (!us.ihmc.idl.IDLTools.equals(this.text_to_speak_, otherMyClass.text_to_speak_)) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("TextToSpeechPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("speak_packet=");
      builder.append(this.speak_packet_);      builder.append(", ");

      builder.append("beep=");
      builder.append(this.beep_);      builder.append(", ");

      builder.append("text_to_speak=");
      builder.append(this.text_to_speak_);
      builder.append("}");
      return builder.toString();
   }
}
