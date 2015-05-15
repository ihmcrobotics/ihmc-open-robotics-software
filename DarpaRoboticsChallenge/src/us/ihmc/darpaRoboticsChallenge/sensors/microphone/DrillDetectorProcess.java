package us.ihmc.darpaRoboticsChallenge.sensors.microphone;

import java.io.InputStream;
import java.net.Authenticator;
import java.net.PasswordAuthentication;
import java.net.URL;

import sun.audio.AudioPlayer;
import us.ihmc.utilities.ThreadTools;

/**
 * <p>Title: DrillDetector</p>
 * <p>Description: Detects a distinct sound by searching for a characteristic peak in FFT magnitude data of sound data from the Atlas Chest Webcam microphone around a given frequency</p>
 * 
 * @author Will
 * @author Igor
 */
public class DrillDetectorProcess
{
   // The annoying audio.cgi disconnects us every ~33 seconds
   private static final int reconnectPeriodSeconds = 30;
   private static final int checkForDrillFrequencyHz = 4;
   private static final int iterationsCount = reconnectPeriodSeconds * checkForDrillFrequencyHz;
   private static final long iterationSleep = 1000 / checkForDrillFrequencyHz;

   private static InputStream connectToStream()
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

   private static void startDrillDetectionLoop()
   {
      DrillDetector detector = new DrillDetector();
      boolean globalState = false;
      boolean lastDetectedState = false;

      while (true)
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

            boolean drillIsOn = detector.isDrillOn(inputStream);

            if (drillIsOn && lastDetectedState)
            {
                System.out.println(" - drill is ON");
                globalState = true;
            }
            else
            {
               if (globalState) { System.out.println(" - drill is off"); }
               globalState = false;
            }

            lastDetectedState = drillIsOn;
         }

         System.out.println("Closing the stream...");
         AudioPlayer.player.stop(inputStream);
         try { inputStream.close(); }
         catch (Exception ignored) { }

         System.out.println("Waiting for reconnect...");
      }
   }

   public static void main(String[] args)
   {
      // set webcam authentification
      Authenticator.setDefault(new Authenticator()
      {
         protected PasswordAuthentication getPasswordAuthentication() { return new PasswordAuthentication("admin", "unknownpw".toCharArray()); }
      });

      startDrillDetectionLoop();
   }
}