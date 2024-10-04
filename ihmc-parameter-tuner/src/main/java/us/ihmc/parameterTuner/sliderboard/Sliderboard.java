package us.ihmc.parameterTuner.sliderboard;

import javax.sound.midi.InvalidMidiDataException;
import javax.sound.midi.MidiDevice;
import javax.sound.midi.MidiSystem;
import javax.sound.midi.MidiUnavailableException;
import javax.sound.midi.Receiver;
import javax.sound.midi.ShortMessage;
import javax.sound.midi.Transmitter;

import us.ihmc.log.LogTools;
import us.ihmc.parameterTuner.sliderboard.boards.SliderboardType;

/**
 * A lightweight sliderboard.
 * <p>
 * This class will automatically detect supported sliderboards if they are plugged in and available as MIDI devices.
 * To determine whether a slider board was connected successfully you may call {@link #isConnected()}. To close the
 * connection to any sliderboard call {@link #close()}.
 * </p>
 * <p>
 * To interact with the sliderboard listeners for sliders can be added and the value of a slider can be set through
 * {@link #setSliderValue(double, int)}.
 * </p>
 *
 * @author Georg
 *
 */
public class Sliderboard
{
   private static final SliderboardType sliderboardType = SliderboardType.findConnectedSliderBoard();

   private final MidiControlMap channelMapper;
   private final SliderboardDataReciever sliderboardDataReciever;

   // Keep these around so they can be closed properly.
   private MidiDevice outDevice;
   private MidiDevice inDevice;
   private Receiver receiver;
   private Transmitter transmitter;

   public Sliderboard()
   {
      if (sliderboardType == null)
      {
         channelMapper = null;
         sliderboardDataReciever = null;
         return;
      }

      channelMapper = sliderboardType.getChannelMapper();
      sliderboardDataReciever = new SliderboardDataReciever(channelMapper);

      // Go through all system MIDI devices and find the ones for receiving from and sending to the sliderboard.
      for (MidiDevice.Info info : MidiSystem.getMidiDeviceInfo())
      {
         if (SliderboardTools.infoMatchesSliderboard(info, sliderboardType.getStringIdentifier()))
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
    * @param sliderIndex the index of the slider
    */
   public boolean addListener(SliderboardListener sliderListener, int sliderIndex)
   {
      if (!isConnected())
      {
         return false;
      }

      return sliderboardDataReciever.addListener(sliderListener, sliderIndex);
   }

   /**
    * Attaches a listener to the specified button that will get called whenever a button is pressed.
    *
    * @param buttonListener the listener for the button
    * @param buttonIndex the index of the button
    */
   public boolean addListener(ButtonListener buttonListener, int buttonIndex)
   {
      if (!isConnected())
      {
         return false;
      }

      return sliderboardDataReciever.addListener(buttonListener, buttonIndex);
   }

   /**
    * Sets the value of a slider.
    *
    * @param sliderPercent the new value for the slider between 0.0 and 1.0
    * @param sliderIndex the index of the slider
    */
   public void setSliderValue(double sliderPercent, int sliderIndex)
   {
      if (!isConnected())
      {
         return;
      }

      int channel = channelMapper.getSliderChannel(sliderIndex);
      if (channel == -1)
      {
         LogTools.info("Unknown slider index: " + sliderIndex);
         return;
      }

      try
      {
         ShortMessage message = new ShortMessage();
         int data = SliderboardTools.toDataByte(sliderPercent);
         message.setMessage(SliderboardTools.STATUS, channel, data);

         // Should this be done on a thread?
         receiver.send(message, -1);
      }
      catch (InvalidMidiDataException e)
      {
         LogTools.info("Was unable to create slider board message.");
      }
   }

   public void setButtonValue(boolean status, int buttonIndex)
   {
      if (!isConnected())
      {
         return;
      }

      int channel = channelMapper.getButtonChannel(buttonIndex);
      if (channel == -1)
      {
         LogTools.info("Unknown button index: " + buttonIndex);
         return;
      }

      try
      {
         ShortMessage message = new ShortMessage();
         int data = SliderboardTools.toDataByte(status);
         message.setMessage(SliderboardTools.STATUS, channel, data);

         // Should this be done on a thread?
         receiver.send(message, -1);
      }
      catch (InvalidMidiDataException e)
      {
         LogTools.info("Was unable to create slider board message.");
      }
   }

   public void clearListeners()
   {
      sliderboardDataReciever.clearListeners();
   }

   public void removeListener(SliderboardListener sliderListener, int sliderIndex)
   {
      sliderboardDataReciever.removeListener(sliderListener, sliderIndex);
   }

   public void removeListener(ButtonListener buttonListener, int buttonIndex)
   {
      sliderboardDataReciever.removeListener(buttonListener, buttonIndex);
   }

}
