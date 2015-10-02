package us.ihmc.communication.packets;


public class TextToSpeechPacket extends Packet<TextToSpeechPacket>
{
   public static String STARTING_CONTROLLER = "starting controller";
   public static String WALKING = "walking";
   public static String MOVING_LEFT_ARM = "moving the left arm";
   public static String MOVING_RIGHT_ARM = "moving the right arm";
   
   public String textToSpeak;
   
   public TextToSpeechPacket()
   {
   }
   
   public TextToSpeechPacket(String textToSpeak)
   {
      this.textToSpeak = textToSpeak;
   }

   public void setTextToSpeak(String textToSpeak)
   {
      this.textToSpeak = textToSpeak;
   }

   public String getTextToSpeak()
   {
      return textToSpeak;
   }
   
   @Override
   public boolean epsilonEquals(TextToSpeechPacket other, double epsilon)
   {
      return textToSpeak.equals(other.textToSpeak);
   }
}
