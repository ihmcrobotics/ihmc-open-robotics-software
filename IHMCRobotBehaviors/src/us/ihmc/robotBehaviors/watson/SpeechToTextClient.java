package us.ihmc.robotBehaviors.watson;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;

import org.apache.commons.io.IOUtils;

import com.ibm.watson.developer_cloud.speech_to_text.v1.SpeechToText;
import com.ibm.watson.developer_cloud.speech_to_text.v1.model.SpeechAlternative;
import com.ibm.watson.developer_cloud.speech_to_text.v1.model.SpeechResults;
import com.ibm.watson.developer_cloud.speech_to_text.v1.model.Transcript;

public class SpeechToTextClient
{
   private static final SpeechToText service = new SpeechToText();
   private static final String username = "a0b3ca38-386e-40ba-91ab-bb6ee24f7b0f";
   private static final String pass = "CJMZGZnnnHBC";
   
   private static final String PREFIX = "SpeechToTextClient";
   private static final String SUFFIX = ".flac";
   
   static
   {
      service.setUsernameAndPassword(username, pass);
   }
   
   public String convertAudioToText(InputStream stream)
   {
      File temporaryAudioFile = createTemporaryFileFromStream(stream);
      if(temporaryAudioFile == null)
      {
         System.err.println("Something was wrong with the stream. Don't cross the streams!");
         return "";
      }
      SpeechResults transcript = service.recognize(temporaryAudioFile, "audio/flac");
      Transcript firstTranscript = transcript.getResults().get(0);
      SpeechAlternative firstSpeechAlternative = firstTranscript.getAlternatives().get(0);
      String text = firstSpeechAlternative.getTranscript();
      return text;
   }
   
   public static void main(String[] args)
   {
      System.out.println("start");
      InputStream audioStream = SpeechToTextClient.class.getClassLoader().getResourceAsStream("Starting Control.flac");
      SpeechToTextClient client = new SpeechToTextClient();
      String result = client.convertAudioToText(audioStream);
      System.out.println(result);
      System.out.println("end");
   }

   public static File createTemporaryFileFromStream(InputStream in)
   {
      File tempFile = null;
      try
      {
         tempFile = File.createTempFile(PREFIX, SUFFIX);
         tempFile.deleteOnExit();
         try (FileOutputStream out = new FileOutputStream(tempFile))
         {
            IOUtils.copy(in, out);
         }
      }
      catch (IOException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
      
      return tempFile;
   }
}