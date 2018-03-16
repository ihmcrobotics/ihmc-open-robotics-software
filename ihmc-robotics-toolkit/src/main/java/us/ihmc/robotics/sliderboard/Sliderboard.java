package us.ihmc.robotics.sliderboard;

import java.util.Random;

import javax.sound.midi.InvalidMidiDataException;
import javax.sound.midi.MidiDevice;
import javax.sound.midi.MidiSystem;
import javax.sound.midi.MidiUnavailableException;
import javax.sound.midi.Receiver;
import javax.sound.midi.ShortMessage;
import javax.sound.midi.Transmitter;

import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;

/**
 * A lightweight sliderboard.
 * <p>
 * This class will automatically detect supported sliderboards if they are plugged in and available as MIDI devices.
 * To determine whether a slider board was connected successfully you may call {@link #isConnected()}. To close the
 * connection to any sliderboard call {@link #close()}.
 * </p>
 * <p>
 * To interact with the sliderboard listeners for sliders can be added and the value of a slider can be set through
 * {@link setSliderValue(double, int)}.
 * </p>
 *
 * @author Georg
 *
 */
public class Sliderboard
{
   private final SliderboardDataReciever sliderboardDataReciever = new SliderboardDataReciever();

   // Keep these around so they can be closed properly.
   private MidiDevice outDevice;
   private MidiDevice inDevice;
   private Receiver receiver;
   private Transmitter transmitter;

   public Sliderboard()
   {
      // Go through all system MIDI devices and find the ones for receiving from and sending to the sliderboard.
      for (MidiDevice.Info info : MidiSystem.getMidiDeviceInfo())
      {
         if (SliderboardTools.infoMatchesSliderboard(info))
         {
            try
            {
               MidiDevice current = MidiSystem.getMidiDevice(info);

               if (receiver == null && (receiver = SliderboardTools.getReceiver(current)) != null)
               {
                  outDevice = current;
               }

               if (transmitter == null && (transmitter = SliderboardTools.getTransmitter(current)) != null)
               {
                  inDevice = current;
               }
            }
            catch (MidiUnavailableException e)
            {
               continue;
            }
         }
      }

      // If we were not able to find the MIDI devices or they did not connect, close everything.
      if (!isConnected())
      {
         close();
         return;
      }

      // Setup the callback for receiving data from the slider board.
      transmitter.setReceiver(sliderboardDataReciever);
   }

   public void close()
   {
      if (receiver != null)
      {
         receiver.close();
         receiver = null;
      }
      if (transmitter != null)
      {
         transmitter.close();
         transmitter = null;
      }
      if (outDevice != null)
      {
         outDevice.close();
         outDevice = null;
      }
      if (inDevice != null)
      {
         inDevice.close();
         inDevice = null;
      }
   }

   public boolean isConnected()
   {
      return receiver != null && transmitter != null && outDevice != null && inDevice != null && outDevice.isOpen() && inDevice.isOpen();
   }

   /**
    * Attaches a listener to the specified slider that will get called whenever a slider is moved.
    *
    * @param sliderListener the listener to ass to the slider
    * @param slider the index of the slider
    */
   public void addListener(SliderboardListener sliderListener, int slider)
   {
      if (!isConnected())
      {
         return;
      }

      sliderboardDataReciever.addListener(sliderListener, slider);
   }

   /**
    * Sets the value of a slider.
    *
    * @param sliderPercent the new value for the slider between 0.0 and 1.0
    * @param slider the index of the slider
    */
   public void setSliderValue(double sliderPercent, int slider)
   {
      if (!isConnected())
      {
         return;
      }

      if (!SliderboardConfiguration.SLIDER_CONTROLLER_MAP.containsKey(slider))
      {
         PrintTools.info("Unknown slider index: " + slider);
         return;
      }

      try
      {
         ShortMessage message = new ShortMessage();
         byte data1 = SliderboardConfiguration.SLIDER_CONTROLLER_MAP.get(slider);
         byte data2 = SliderboardTools.toDataByte(sliderPercent);
         message.setMessage(SliderboardTools.STATUS, data1, data2);

         // Should this be done on a thread?
         receiver.send(message, -1);
      }
      catch (InvalidMidiDataException e)
      {
         PrintTools.info("Was unable to create slider board message.");
      }
   }

   public static void main(String[] args)
   {
      Sliderboard sliderboard = new Sliderboard();

      int[] sliders = SliderboardConfiguration.SLIDER_CONTROLLER_MAP.keys();
      for (int slider : sliders)
      {
         sliderboard.addListener(value -> PrintTools.info("Slider " + slider +" moved to: " + value), slider);
      }

      Random random = new Random();
      for (int i = 0; i < 10; i++)
      {
         sliderboard.setSliderValue(random.nextDouble(), sliders[random.nextInt(sliders.length)]);
         ThreadTools.sleep(100);
      }

      ThreadTools.sleep(10000);

      sliderboard.close();
   }
}
