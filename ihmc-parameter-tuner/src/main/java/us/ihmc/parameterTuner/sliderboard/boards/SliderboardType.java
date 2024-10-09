package us.ihmc.parameterTuner.sliderboard.boards;

import javax.sound.midi.MidiDevice;
import javax.sound.midi.MidiSystem;

import us.ihmc.commons.PrintTools;
import us.ihmc.parameterTuner.sliderboard.MidiControlMap;

public enum SliderboardType
{
   BCF2000;

   public String getStringIdentifier()
   {
      switch (this)
      {
      case BCF2000:
         return "BCF2000";
      default:
         throw new RuntimeException("Unknown string identifier for sliderboard " + this + ".");
      }
   }

   public MidiControlMap getChannelMapper()
   {
      switch (this)
      {
      case BCF2000:
         return new BCF2000ControlMap();
      default:
         throw new RuntimeException("Unknown channel mapping for sliderboard " + this + ".");
      }
   }

   public static SliderboardType findConnectedSliderBoard()
   {
      for (MidiDevice.Info info : MidiSystem.getMidiDeviceInfo())
      {
         String name = info.getName();
         String description = info.getDescription();

         for (SliderboardType sliderboardType : SliderboardType.values())
         {
            String identifier = sliderboardType.getStringIdentifier();

            if (name.contains(identifier) || description.contains(identifier))
            {
               return sliderboardType;
            }
         }
      }

      PrintTools.info("Was not able to find a supported sliderboard.");
      return null;
   }
}
