package us.ihmc.tools.inputDevices.ghostMouse;

import java.awt.AWTException;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.StringTokenizer;

import us.ihmc.tools.inputDevices.mouse.MouseButton;
import us.ihmc.tools.inputDevices.mouse.MousePressedOrReleased;

public class GhostMousePlayback
{
   private final ArrayList<GhostMousePlaybackEvent> playbackEvents = new ArrayList<GhostMousePlaybackEvent>();
   private final java.awt.Robot awtRobot;
   
   public GhostMousePlayback() throws AWTException
   {
      awtRobot = new java.awt.Robot();
   }

   public void addPlaybackEvent(GhostMousePlaybackEvent playbackEvent)
   {
      playbackEvents.add(playbackEvent);
   }

   public void addPlaybackEvent(String playbackEventString)
   {
      StringTokenizer tokenizer = new StringTokenizer(playbackEventString);
      String mouseType = tokenizer.nextToken("{ ");

      GhostMousePlaybackEvent playbackEvent = null;

      if (mouseType.equals("Move"))
      {
         int[] coordinates = getCoordinates(tokenizer);
         playbackEvent = new GhostMouseMovePlaybackEvent(coordinates);
      } else if (mouseType.equals("LMouse"))
      {
         playbackEvent = getPressedOrReleasedEvent(MouseButton.LEFT, tokenizer);
      } else if (mouseType.equals("MMouse"))
      {
         playbackEvent = getPressedOrReleasedEvent(MouseButton.MIDDLE, tokenizer);
      } else if (mouseType.equals("RMouse"))
      {
         playbackEvent = getPressedOrReleasedEvent(MouseButton.RIGHT, tokenizer);
      } else if (mouseType.equals("Delay"))
      {
         String delayInSecondsString = tokenizer.nextToken(" }");
         double delayInSeconds = Double.parseDouble(delayInSecondsString);

         playbackEvent = new GhostMouseDelayPlaybackEvent(delayInSeconds);
      } else
      {
         MousePressedOrReleased pressedOrReleased = nextTokenDownOrUp(tokenizer);

         if (pressedOrReleased == MousePressedOrReleased.PRESSED)
         {
            playbackEvent = new GhostMouseKeyPressPlaybackEvent(mouseType);
         } else
         {
            playbackEvent = new GhostMouseKeyReleasePlaybackEvent(mouseType);
         }
      }

      if (playbackEvent == null)
      {
         throw new RuntimeException("Cannot parse playbackEventString = " + playbackEventString);
      }

      addPlaybackEvent(playbackEvent);
   }

   private GhostMousePlaybackEvent getPressedOrReleasedEvent(MouseButton mouseButton, StringTokenizer tokenizer)
   {
      MousePressedOrReleased pressedOrReleased = nextTokenDownOrUp(tokenizer);
      int[] coordinates = getCoordinates(tokenizer);

      if (pressedOrReleased == MousePressedOrReleased.PRESSED)
      {
         return new GhostMouseButtonPressedPlaybackEvent(coordinates, mouseButton);
      } else if (pressedOrReleased == MousePressedOrReleased.RELEASED)
      {
         return new GhostMouseButtonReleasedPlaybackEvent(coordinates, mouseButton);
      } else
      {
         throw new RuntimeException("pressedOrReleased must be either Pressed or Released!");
      }
   }

   private MousePressedOrReleased nextTokenDownOrUp(StringTokenizer tokenizer)
   {
      String mouseDownOrUp = tokenizer.nextToken(" }");

      if (mouseDownOrUp.equals("down"))
         return MousePressedOrReleased.PRESSED;
      else if (mouseDownOrUp.equals("up"))
         return MousePressedOrReleased.RELEASED;
      else
         throw new RuntimeException("Neither down nor up!");
   }

   private int[] getCoordinates(StringTokenizer tokenizer)
   {
      String xString = tokenizer.nextToken(" (,");
      String yString = tokenizer.nextToken(" ,)");

      int x = Integer.parseInt(xString);
      int y = Integer.parseInt(yString);

      return new int[] { x, y };
   }

   public void playback()
   {
      playback(1.0);
   }
   
   public void playback(double playbackSpeed)
   {
      for (GhostMousePlaybackEvent playbackEvent : playbackEvents)
      {
         playbackEvent.playback(awtRobot, playbackSpeed);
      }
   }

   public void load(String filename)
   {
      File file = new File(filename);

      load(file);
   }

   public void load(File file)
   {
      try
      {
         BufferedReader reader = new BufferedReader(new FileReader(file));
         load(reader);
      } catch (FileNotFoundException e)
      {
         e.printStackTrace();
      }

   }

   private void load(BufferedReader reader)
   {
      while (true)
      {
         try
         {
            String playbackEventString = reader.readLine();
            if (playbackEventString == null)
               return;
            this.addPlaybackEvent(playbackEventString);
         } catch (IOException e)
         {
            e.printStackTrace();
            return;
         }
      }
   }

   @Override
   public String toString()
   {
      String ret = "";
      for (GhostMousePlaybackEvent playbackEvent : playbackEvents)
      {
         ret = ret + playbackEvent.toString() + "\n";
      }

      return ret;
   }
}
