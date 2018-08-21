package us.ihmc.robotics.sliderboard.boards;

import us.ihmc.robotics.sliderboard.MidiControlMap;

public class BCF2000ControlMap implements MidiControlMap
{
   @Override
   public int getSliderChannel(int sliderIndex)
   {
      if (!MidiControlMap.isInRange(sliderIndex, 1, 8))
      {
         return -1;
      }

      return sliderIndex + 80;
   }

   @Override
   public int getSliderIndex(int sliderChannel)
   {
      if (!MidiControlMap.isInRange(sliderChannel, 81, 88))
      {
         return -1;
      }

      return sliderChannel - 80;
   }

   @Override
   public int getButtonChannel(int buttonIndex)
   {
      if (!MidiControlMap.isInRange(buttonIndex, 1, 16))
      {
         return -1;
      }

      return buttonIndex + 64;
   }

   @Override
   public int getButtonIndex(int buttonChannel)
   {
      if (!MidiControlMap.isInRange(buttonChannel, 65, 80))
      {
         return -1;
      }

      return buttonChannel - 64;
   }
}
