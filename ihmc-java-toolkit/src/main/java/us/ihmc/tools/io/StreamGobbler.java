package us.ihmc.tools.io;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.PrintStream;

public class StreamGobbler extends Thread
{
   private final PrintStream outputStream;
   private final InputStream inputStream;

   public StreamGobbler(InputStream inputStream)
   {
      this(inputStream, null);
   }
   
   public StreamGobbler(InputStream inputStream, PrintStream outputStream)
   {
      this.inputStream = inputStream;
      this.outputStream = outputStream;
   }

   @Override
   public void run()
   {
      try
      {
         InputStreamReader inputStreamReader = new InputStreamReader(inputStream);
         BufferedReader bufferedReader = new BufferedReader(inputStreamReader);
         String line = null;
         while ((line = bufferedReader.readLine()) != null)
         {
            if (outputStream != null)
            {
               outputStream.println(line);
               outputStream.flush();
            }
         }
      }
      catch (IOException ioe)
      {
         ioe.printStackTrace();
      }
   }
}