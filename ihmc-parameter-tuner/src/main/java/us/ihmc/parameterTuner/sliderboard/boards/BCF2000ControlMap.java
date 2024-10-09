package us.ihmc.parameterTuner.sliderboard.boards;

import us.ihmc.parameterTuner.sliderboard.MidiControlMap;

/**
 * Implementation of the {@link MidiControlMap} for the BCF2000 sliderboard.
 * <p>
 * Indexing of the buttons and sliders on the board is as follows:</br>
 *  - The sliders 1-8 map to the physical sliders on the board.</br>
 *  - The sliders 9-16 map to the knobs on the board.</br>
 *  - The buttons 1-16 map to the buttons below the knobs on the board.</br>
 *  - The buttons 17-20 map the buttons in the bottom right of the board.</br>
 *  - The buttons 21-28 map the knobs as they can be used as buttons as well.
 * </p>
 *
 * @author Georg Wiedebach
 *
 */
public class BCF2000ControlMap implements MidiControlMap
{
   @Override
   public int getSliderChannel(int sliderIndex)
   {
      if (MidiControlMap.isInRange(sliderIndex, 1, 8))
      {
         return sliderIndex + 80;
      }
      // the knobs
      if (MidiControlMap.isInRange(sliderIndex, 9, 16))
      {
         return sliderIndex - 8;
      }

      return INVALID;
   }

   @Override
   public int getSliderIndex(int sliderChannel)
   {
      if (MidiControlMap.isInRange(sliderChannel, 81, 88))
      {
         return sliderChannel - 80;
      }
      // the knobs
      if (MidiControlMap.isInRange(sliderChannel, 1, 8))
      {
         return sliderChannel + 8;
      }

      return INVALID;
   }

   @Override
   public int getButtonChannel(int buttonIndex)
   {
      if (MidiControlMap.isInRange(buttonIndex, 1, 16))
      {
         return buttonIndex + 64;
      }
      // bottom right buttons
      if (MidiControlMap.isInRange(buttonIndex, 17, 20))
      {
         return buttonIndex + 72;
      }
      // knobs
      if (MidiControlMap.isInRange(buttonIndex, 21, 28))
      {
         return buttonIndex + 12;
      }

      return INVALID;
   }

   @Override
   public int getButtonIndex(int buttonChannel)
   {
      if (MidiControlMap.isInRange(buttonChannel, 65, 80))
      {
         return buttonChannel - 64;
      }
      // bottom right buttons
      if (MidiControlMap.isInRange(buttonChannel, 89, 92))
      {
         return buttonChannel - 72;
      }
      // knobs
      if (MidiControlMap.isInRange(buttonChannel, 33, 40))
      {
         return buttonChannel - 12;
      }

      return INVALID;
   }

   @Override
   public int getDelayVariationChannel()
   {
      return 94;
   }
}
