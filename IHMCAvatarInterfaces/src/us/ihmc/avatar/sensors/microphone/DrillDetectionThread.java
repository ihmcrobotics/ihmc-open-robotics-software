package us.ihmc.avatar.sensors.microphone;

import java.io.InputStream;
import java.net.Authenticator;
import java.net.PasswordAuthentication;
import java.net.URL;

import sun.audio.AudioPlayer;
import us.ihmc.tools.thread.ThreadTools;

public abstract class DrillDetectionThread extends Thread
{
   // The annoying audio.cgi disconnects us every ~33 seconds
   private static final int reconnectPeriodSeconds = 30;
   private static final double checkForDrillFrequencyHz = 5.0;
   private static final int iterationsCount = (int)((double)reconnectPeriodSeconds * checkForDrillFrequencyHz);
   private static final long iterationSleep = (long)(1000.0 / checkForDrillFrequencyHz);

   private DrillDetectionAlgorithm algorithm = null;
   private boolean isRunning = false;

   public DrillDetectionThread(DrillDetectionAlgorithm algorithm)
   {
      // set webcam authentification
      Authenticator.setDefault(new Authenticator()
      {
         protected PasswordAuthentication getPasswordAuthentication() { return new PasswordAuthentication("admin", "unknownpw".toCharArray()); }
      });

      this.isRunning = true;
      this.algorithm = algorithm;
   }

   public boolean isRunning()
   {
      return isRunning;
   }

   public void shutdown()
   {
      isRunning = false;
   }

   private InputStream connectToStream()
   {
      try
      {
//         URL url = new URL("http://192.168.0.19:80/audio.cgi"); //Home
//         URL url = new URL("http://139.169.44.114:80/audio.cgi"); //JSC

         URL url = new URL("http://10.6.100.57:80/audio.cgi"); //Robotlab
         return url.openStream();
      }
      catch (Exception e)
      {
         e.printStackTrace();
         return null;
      }
   }

   @Override
   public void run()
   {
      System.out.println("Starting drill detection thread...");

      while (isRunning)
      {
         InputStream inputStream = connectToStream();
         if (inputStream == null)
         {
            ThreadTools.sleep(iterationSleep);
            continue;
         }

         System.out.println("Connected to the webcam. Opening the stream...");
         AudioPlayer.player.start(inputStream);

         for (int i = 0; i < iterationsCount; i++)
         {
            ThreadTools.sleep(iterationSleep);

            DrillDetectionResult result = algorithm.isDrillOn(inputStream);
            if (result != null) { onDrillDetectionResult(result); }

            if (!isRunning) { break; }
         }

         System.out.println("Closing the stream...");
         AudioPlayer.player.stop(inputStream);
         try { inputStream.close(); }
         catch (Exception ignored) { }

         System.out.println("Waiting for reconnect...");
      }

      System.out.println("Stopping drill detection thread...");
   }

   public abstract void onDrillDetectionResult(DrillDetectionResult result);
}