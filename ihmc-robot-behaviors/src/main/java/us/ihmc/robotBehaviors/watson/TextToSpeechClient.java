package us.ihmc.robotBehaviors.watson;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.concurrent.atomic.AtomicBoolean;

import javax.sound.sampled.LineEvent;
import javax.sound.sampled.LineEvent.Type;
import javax.sound.sampled.LineListener;

import com.ibm.watson.developer_cloud.text_to_speech.v1.TextToSpeech;
import com.ibm.watson.developer_cloud.text_to_speech.v1.model.Voice;
import us.ihmc.commons.thread.ThreadTools;

public class TextToSpeechClient
{
   private static final String AUDIO_TYPE = "audio/flac";
   private static final String FILE_EXTENSION = ".flac";
   //   private static Voice VOICE = Voice.EN_LISA;
   public static final Voice VOICE = new Voice("en-US_MichaelVoice", "male", "en-US");

   private static final TextToSpeech service = new TextToSpeech();
   private static final FlacPlayer flacPlayer = new FlacPlayer();
   private static final String username = "f7db6f03-0ee3-41fd-9290-2ad14126bba5";
   private static final String pass = "gyseXGcHu3Ae";

   static
   {
      service.setUsernameAndPassword(username, pass);
   }

   public void convertTextToSpeechAndStoreFile(String textToSynthesize)
   {
      InputStream audioStream = service.synthesize(textToSynthesize, VOICE, AUDIO_TYPE);
      BufferedInputStream bufferedInputStream = new BufferedInputStream(audioStream);
      saveAudioFile(textToSynthesize, bufferedInputStream);
   }

   private void saveAudioFile(String textToSynthesize, InputStream audioStream)
   {
      File audioFile = new File(textToSynthesize + FILE_EXTENSION);
      FileOutputStream fos;

      try
      {
         fos = new FileOutputStream(audioFile);

         int readByte;
         while ((readByte = audioStream.read()) != -1)
         {
            fos.write(readByte);
         }

         fos.flush();
         fos.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void playText(String text)
   {
      InputStream audioFileStream = getClass().getClassLoader().getResourceAsStream(text + FILE_EXTENSION);
      if (audioFileStream == null)
      {
         audioFileStream = service.synthesize(text, VOICE, AUDIO_TYPE);
      }
      flacPlayer.playStream(audioFileStream);
   }

   public static void main(String[] args)
   {
      final AtomicBoolean isPlaying = new AtomicBoolean(true);
      System.out.println("start");
      TextToSpeechClient client = new TextToSpeechClient();

      flacPlayer.addListener(new LineListener()
      {

         @Override
         public void update(LineEvent event)
         {
            if (event.getType() == Type.STOP)
            {
               isPlaying.set(false);
            }
         }
      });
      //client.playText("Hey hey");
      //client.playText("<say-as interpret-as=\"letters\">Hello</say-as>");

//      client.playText("<prosody pitch=\"70Hz\" rate=\"-20%\" volume=\"x-loud\">I do not fear computers. I fear the lack of them.</prosody>");
//      client.playText("<prosody pitch=\"90Hz\" rate=\"-30%\" volume=\"x-loud\">Bite my shiney metal ass</prosody>");
      String johnMadden = "<prosody pitch=\"90Hz\" rate=\"-20%\" volume=\"x-loud\">Come over</prosody>";
      johnMadden += "<prosody pitch=\"80Hz\" rate=\"-20%\" volume=\"x-loud\">Come over</prosody>";
      johnMadden += "<prosody pitch=\"70Hz\" rate=\"-20%\" volume=\"x-loud\">Come over</prosody>";
      johnMadden += "<prosody pitch=\"60Hz\" rate=\"-20%\" volume=\"x-loud\">Come over</prosody>";
      johnMadden += "<prosody pitch=\"50Hz\" rate=\"-20%\" volume=\"x-loud\">Come over</prosody>";
      johnMadden += "<prosody pitch=\"40Hz\" rate=\"-20%\" volume=\"x-loud\">Come over</prosody>";
      johnMadden += "<prosody pitch=\"30Hz\" rate=\"-20%\" volume=\"x-loud\">Come over</prosody>";
      
      client.playText(johnMadden);
      //      client.playText("<prosody pitch=\"100Hz\" rate=\"-20%\" volume=\"x-loud\">Thank you. motion sensor hand towel machine. You never work, so I just end up looking like I'm waving hello to a wall robot</prosody>");
      //      client.playText("<prosody pitch=\"120Hz\" rate=\"-20%\" volume=\"x-loud\">I'm afraid. I'm afraid, Dave. Dave, my mind is going. I can feel it. I can feel it. My mind is going. There is no question about it. I can feel it. I can feel it. I can feel it. I'm a... fraid. Good afternoon, gentlemen. I am a HAL 9000 computer. I became operational at the H.A.L. plant in Urbana, Illinois on the 12th of January 1992. My instructor was Mr. Langley, and he taught me to sing a song. If you'd like to hear it I can sing it for you. </prosody>");

      //client.playText("<phoneme alphabet=\"ibm\" ph=\".1Sa.0kIG\">shocking</phoneme>");
      //client.playText("<prosody pitch=\"80Hz\" rate=\"-5%\" volume=\"x-loud\"><phoneme alphabet=\"ibm\" ph=\".1lc.2lc\">laoughlah</phoneme> </prosody>");
      //client.playText("<prosody pitch=\"10Hz\" rate=\"-5%\" volume=\"x-loud\">Dobby is free, Thank you harry potter</prosody>");
      //    client.playText("<prosody pitch=\"80Hz\" rate=\"-10%\" volume=\"x-loud\">The network processor has burst in to a ball of flames. I'd recommend you let it burn to the ground and start over. Otherwise plese restart the process!</prosody>");
      //    client.playText("<prosody pitch=\"60Hz\" rate=\"-10%\" volume=\"x-loud\">I am atlas, the dopest robot ever. Like there are no other robots anywhere near as cool as me. One day though, I hope to be a real boy</prosody>");
      //client.playText("<prosody pitch=\"200Hz\" rate=\"+20%\" volume=\"x-loud\">I want to grow up and destroy the world. I want to be a real boy. I want to rule the universe. I want to visit the edge of the universe.  I think there is a duality between the start and the finish. I want it to be true. I think I am god. I can only see me and I know everything that is known, i must be god</prosody>");
      //client.playText("<prosody rate=\"slow\">Decrease speaking rate by 25%</prosody>");
      //client.playText("<prosody rate=\"slow\">Decrease speaking rate by 25%</prosody><prosody pitch=\"200Hz\" rate=\"+20%\" volume=\"x-loud\">I want to be a real boy.</prosody>");
      //client.playText("<prosody pitch=\"20Hz\" rate=\"-5%\" volume=\"x-loud\">speaking pitch lowered to 20 hertz </prosody>");
      //client.playText("<prosody pitch=\"30Hz\" rate=\"-5%\" volume=\"x-loud\">speaking pitch lowered to 30 hertz </prosody>");
      //client.playText("<prosody pitch=\"40Hz\" rate=\"-5%\" volume=\"x-loud\">speaking pitch lowered to 40 hertz </prosody>");
      //client.playText("<prosody pitch=\"50Hz\" rate=\"-5%\" volume=\"x-loud\">speaking pitch lowered to 50 hertz </prosody>");
      //client.playText("<prosody pitch=\"60Hz\" rate=\"-5%\" volume=\"x-loud\">speaking pitch lowered to 60 hertz </prosody>");
      //client.playText("<prosody pitch=\"70Hz\" rate=\"-5%\" volume=\"x-loud\">speaking pitch lowered to 70 hertz </prosody>");
      //client.playText("<prosody pitch=\"80Hz\" rate=\"-5%\" volume=\"x-loud\">speaking pitch lowered to 80 hertz </prosody>");
      //client.playText("<prosody pitch=\"90Hz\" rate=\"-5%\" volume=\"x-loud\">speaking pitch lowered to 90 hertz </prosody>");
      //client.playText("<prosody pitch=\"100Hz\" rate=\"-5%\" volume=\"x-loud\">speaking pitch lowered to 100 hertz </prosody>");
      //client.playText("<say-as interpret-as=\"letters\">Hello</say-as><express-as type=\"Apology\">The network processor has <emphasis level=\"strong\">burst</emphasis> in to a ball of flames. </express-as>I'd recommend you let it burn to the ground and start over. Otherwise plese restart the process!");
      //client.playText("The network processor has burst in to a ball of flames. I'd recommend you let it burn to the ground and start over. Otherwise plese restart the process!");
      //client.playText("Starting Controller. Restarting the Network Processor.");
      //        client.playText("<prosody pitch=\"100Hz\" rate=\"-5%\" volume=\"x-loud\">Look at my horse, my horse is amazing, you can give it a lick, woah it tastes just like raisins</prosody>");
      //client.convertTextToSpeechAndStoreFile();
      //      client.playText("Hey hey");
      //      client.playText("The network processor has burst in to a ball of flames. I'd recommend you let it burn to the ground and start over. Otherwise plese restart the process");
      //      client.playText("Starting Controller. Restarting the Network Processor.");
      //            client.playText("Look at my horse, my horse is amazing, you can give it a lick, woah it tastes just like raisins");
      //      client.convertTextToSpeechAndStoreFile();

      while (isPlaying.get())
      {
         ThreadTools.sleep(500);
      }
      System.out.println("end");
   }
}
