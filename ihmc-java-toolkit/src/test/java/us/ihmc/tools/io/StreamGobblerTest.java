package us.ihmc.tools.io;

import static us.ihmc.robotics.Assert.*;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.PrintStream;

import org.junit.jupiter.api.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.commons.thread.ThreadTools;

public class StreamGobblerTest
{

	@Test
   public void testStreamGobblerWithASingleLine() throws IOException
   {
      String string = "This is some input! Yep, this is one fine line of input!";
      byte[] bytes = string.getBytes("UTF-8");
      InputStream inputStream = new ByteArrayInputStream(bytes);
      
      ByteArrayOutputStream byteArrayOutputStream = new ByteArrayOutputStream();
      PrintStream printStream = new PrintStream(byteArrayOutputStream);
      
      assertEquals(bytes.length, inputStream.available());
      
      StreamGobbler streamGobbler = new StreamGobbler(inputStream, printStream);
      streamGobbler.start();
      
      while(streamGobbler.isAlive())
      {
         ThreadTools.sleep(10);
      }
      
      byte[] returnBytes = byteArrayOutputStream.toByteArray();
      String returnString = new String(returnBytes, "UTF-8");
      
      assertTrue(returnString.startsWith(string)); 
      assertEquals(0, inputStream.available());
   }

	@Test
   public void testStreamGobblerToGobbleMultipleLines() throws IOException
   {
      String string = "This is some input! \nHere's a second line of input!";
      byte[] bytes = string.getBytes("UTF-8");
      InputStream inputStream = new ByteArrayInputStream(bytes);
      
      assertEquals(bytes.length, inputStream.available());

      StreamGobbler streamGobbler = new StreamGobbler(inputStream);
      streamGobbler.start();
     
      while(streamGobbler.isAlive())
      {
         ThreadTools.sleep(10);
      }
      assertEquals(0, inputStream.available());
   }

}
