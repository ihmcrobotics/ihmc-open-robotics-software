package controller_msgs.msg.dds;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * Use this message for the robot to speak.
 */
public class TextToSpeechPacket implements Settable<TextToSpeechPacket>, EpsilonComparable<TextToSpeechPacket>
{
   public static final String FOOTSTEP_COMPLETED = "Finished Taking Footstep";

   public static final String FINISHED_WALKING = "Finished walking";

   public static final String WALKING_ABORTED = "walking aborted";

   public static final String STARTING_CONTROLLER = "starting controller";

   public static final String WALKING = "walking";

   public static final String MOVING_LEFT_ARM = "moving the left arm";

   public static final String MOVING_RIGHT_ARM = "moving the right arm";

   public static final String NETWORKPROCESSOR_ONLINE = "Reestablished Connection To The Network Processor";
   private boolean speak_packet_;
   private boolean beep_ = true;
   private java.lang.StringBuilder text_to_speak_;

   public TextToSpeechPacket()
   {

      text_to_speak_ = new java.lang.StringBuilder(255);
   }

   public TextToSpeechPacket(TextToSpeechPacket other)
   {
      set(other);
   }

   public void set(TextToSpeechPacket other)
   {
      speak_packet_ = other.speak_packet_;

      beep_ = other.beep_;

      text_to_speak_.setLength(0);
      text_to_speak_.append(other.text_to_speak_);
   }

   public boolean getSpeakPacket()
   {
      return speak_packet_;
   }

   public void setSpeakPacket(boolean speak_packet)
   {
      speak_packet_ = speak_packet;
   }

   public boolean getBeep()
   {
      return beep_;
   }

   public void setBeep(boolean beep)
   {
      beep_ = beep;
   }

   public java.lang.String getTextToSpeakAsString()
   {
      return getTextToSpeak().toString();
   }

   public java.lang.StringBuilder getTextToSpeak()
   {
      return text_to_speak_;
   }

   public void setTextToSpeak(java.lang.String text_to_speak)
   {
      text_to_speak_.setLength(0);
      text_to_speak_.append(text_to_speak);
   }

   @Override
   public boolean epsilonEquals(TextToSpeechPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.speak_packet_, other.speak_packet_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.beep_, other.beep_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.text_to_speak_, other.text_to_speak_, epsilon))
         return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof TextToSpeechPacket))
         return false;

      TextToSpeechPacket otherMyClass = (TextToSpeechPacket) other;

      if (this.speak_packet_ != otherMyClass.speak_packet_)
         return false;

      if (this.beep_ != otherMyClass.beep_)
         return false;

      if (!us.ihmc.idl.IDLTools.equals(this.text_to_speak_, otherMyClass.text_to_speak_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("TextToSpeechPacket {");
      builder.append("speak_packet=");
      builder.append(this.speak_packet_);

      builder.append(", ");
      builder.append("beep=");
      builder.append(this.beep_);

      builder.append(", ");
      builder.append("text_to_speak=");
      builder.append(this.text_to_speak_);

      builder.append("}");
      return builder.toString();
   }
}