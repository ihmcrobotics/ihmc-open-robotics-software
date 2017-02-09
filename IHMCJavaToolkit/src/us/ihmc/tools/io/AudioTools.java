package us.ihmc.tools.io;

import java.io.InputStream;
import java.net.URL;

import javax.sound.sampled.AudioFormat;
import javax.sound.sampled.AudioInputStream;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.Clip;
import javax.sound.sampled.DataLine;
import javax.sound.sampled.LineUnavailableException;

import us.ihmc.tools.io.printing.PrintTools;

public class AudioTools
{
   /**
    * Used for playing a clip over and over again. Just load the same 
    * clip 20 times or so into an array and call this to play it over
    * and over again on top of itself.
    */
   public static void playFirstNotRunningClip(Clip[] clips)
   {
      for (int i = 0; i < clips.length; i++)
      {
         if (clips[i].isRunning())
         {
            continue;
         }
         else
         {
            clips[i].setFramePosition(0);
            clips[i].start();
            return;
         }
      }
   }

   /**
    * Play a clip once. Will do nothing if clip is already playing.
    */
   public static void playSoundOnce(Clip clip)
   {      
      if (clip == null || clip.isRunning())
         return;
      
      clip.setFramePosition(0);
      clip.start();
   }

   public static Clip loadSoundClip(InputStream is)
   {
      try
      {
         AudioInputStream stream;
         AudioFormat format;
         DataLine.Info info;
         Clip clip;

         stream = AudioSystem.getAudioInputStream(is);
         format = stream.getFormat();
         info = new DataLine.Info(Clip.class, format);
         clip = (Clip) AudioSystem.getLine(info);
         clip.open(stream);

         return clip;
      }
      catch (Exception e)
      {
         e.printStackTrace();
         return null;
      }
   }

   public static Clip loadSoundClip(URL url)
   {
      try
      {
         AudioInputStream stream;
         AudioFormat format;
         DataLine.Info info;
         Clip clip;

         stream = AudioSystem.getAudioInputStream(url);
         format = stream.getFormat();
         info = new DataLine.Info(Clip.class, format);
         clip = (Clip) AudioSystem.getLine(info);
         clip.open(stream);

         return clip;
      }
      catch (LineUnavailableException e)
      {
//         PrintTools.warn("Plug in a speaker to hear sounds.");
         return null;
      }
      catch (Exception e)
      {
         e.printStackTrace();
         return null;
      }
   }
}
