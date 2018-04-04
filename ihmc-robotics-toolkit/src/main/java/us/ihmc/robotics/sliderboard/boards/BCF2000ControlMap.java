package us.ihmc.robotics.sliderboard.boards;

import us.ihmc.robotics.sliderboard.MidiControlMap;

public class BCF2000ControlMap implements MidiControlMap
{
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
