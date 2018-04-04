package us.ihmc.robotics.sliderboard.boards;

import us.ihmc.robotics.sliderboard.MidiControlMap;

public class BCF2000ControlMap implements MidiControlMap
{
   @Override
   public int getKnobChannel(int knobIndex)
   {
      if (!MidiControlMap.isInRange(knobIndex, 0, 8))
      {
         return -1;
      }

      return knobIndex;
   }

   @Override
   public int getKnobIndex(int knobChannel)
   {
      if (!MidiControlMap.isInRange(knobChannel, 0, 8))
      {
         return -1;
      }

      return knobChannel;
   }

   @Override
   public int getSliderChannel(int sliderIndex)
   {
      if (!MidiControlMap.isInRange(sliderIndex, 0, 8))
      {
         return -1;
      }

      return sliderIndex + 80;
   }

   @Override
   public int getSliderIndex(int sliderChannel)
   {
      if (!MidiControlMap.isInRange(sliderChannel, 80, 88))
      {
         return -1;
      }

      return sliderChannel - 80;
   }
}
