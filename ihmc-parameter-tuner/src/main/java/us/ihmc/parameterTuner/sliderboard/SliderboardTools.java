package us.ihmc.parameterTuner.sliderboard;

import javax.sound.midi.MidiDevice;
import javax.sound.midi.MidiUnavailableException;
import javax.sound.midi.Receiver;
import javax.sound.midi.Transmitter;

import us.ihmc.commons.MathTools;

public class SliderboardTools
{
   /**
    * Indicates that the MIDI command starting with this byte is a control command.
    */
   public static final byte CONTROL_BYTE = 0b1011;

   /**
    * Indicates the MIDI channel for a MIDI message.
    */
   public static final byte CHANNEL_ZERO = 0b0000;

   /**
    * MIDI message status for a control command on MIDI channel 0.
    */
   public static final short STATUS = (CONTROL_BYTE << 4) | CHANNEL_ZERO;

   /**
    * Maximum value for a valid data byte to be sent as part of a MIDI message.
    */
   public static final int MAX_VALUE = 0b01111111;

   public static double toSliderPercent(int dataByte)
   {
      return MathTools.clamp(((double) dataByte) / MAX_VALUE, 0.0, 1.0);
   }

   public static boolean toButtonStatus(int dataByte)
   {
      return dataByte == MAX_VALUE;
   }

   public static int toDataByte(double sliderPercent)
   {
      return (int) (MAX_VALUE * MathTools.clamp(sliderPercent, 0.0, 1.0));
   }

   public static int toDataByte(boolean status)
   {
      return status ? MAX_VALUE : 0;
   }

   public static boolean infoMatchesSliderboard(MidiDevice.Info info, String sliderBoardName)
   {
      String name = info.getName();
      String description = info.getDescription();
      return name.contains(sliderBoardName) || description.contains(sliderBoardName);
   }

   public static Transmitter getTransmitter(MidiDevice current)
   {
      if (current.getMaxTransmitters() != 0)
      {
         try
         {
            if (!current.isOpen())
            {
               current.open();
            }
         }
         catch (MidiUnavailableException e)
         {
            return null;
         }

         try
         {
            return current.getTransmitter();
         }
         catch (MidiUnavailableException e)
         {
            current.close();
         }
      }

      return null;
   }

   public static Receiver getReceiver(MidiDevice current)
   {
      if (current.getMaxReceivers() != 0)
      {
         try
         {
            if (!current.isOpen())
            {
               current.open();
            }
         }
         catch (MidiUnavailableException e)
         {
            return null;
         }

         try
         {
            return current.getReceiver();
         }
         catch (MidiUnavailableException e)
         {
            current.close();
         }
      }

      return null;
   }

}
