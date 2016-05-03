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
import us.ihmc.tools.thread.ThreadTools;

public class TextToSpeechClient
{
   private static final String AUDIO_TYPE = "audio/flac";
   private static final String FILE_EXTENSION = ".flac";
   private static Voice VOICE = Voice.EN_LISA;
   
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
      if(audioFileStream == null)
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
            if(event.getType() == Type.STOP)
            {
               isPlaying.set(false);
            }
         }
      });
      
//      client.playText("Hey hey");
      client.playText("The network processor has burst in to a ball of flames. I'd recommend you let it burn to the ground and start over. Otherwise plese restart the process");
//      client.playText("Starting Controller. Restarting the Network Processor.");
            client.playText("Look at my horse, my horse is amazing, you can give it a lick, woah it tastes just like raisins");
//      client.convertTextToSpeechAndStoreFile();
      
      while(isPlaying.get())
      {
         ThreadTools.sleep(200);
      }
      System.out.println("end");
   }
}












