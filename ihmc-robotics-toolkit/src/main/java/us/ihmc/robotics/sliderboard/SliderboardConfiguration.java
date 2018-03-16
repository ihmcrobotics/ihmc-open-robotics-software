package us.ihmc.robotics.sliderboard;

import javax.sound.midi.MidiDevice;
import javax.sound.midi.MidiSystem;

import gnu.trove.map.TByteIntMap;
import gnu.trove.map.TIntByteMap;
import gnu.trove.map.hash.TByteIntHashMap;
import gnu.trove.map.hash.TIntByteHashMap;
import us.ihmc.commons.PrintTools;

public class SliderboardConfiguration
{
   /**
    * Slider controller byte by slider index. There are up to 127 MIDI controllers and what
    * controller is attached to which slider is slider board specific.
    */
   public static final TIntByteMap SLIDER_CONTROLLER_MAP = new TIntByteHashMap();

   /**
    * Similar to {@link #SLIDER_CONTROLLER_MAP} this returns a map to retrieve the slider
    * given the MIDI controller byte.
    */
   public static final TByteIntMap CONTROLLER_SLIDER_MAP = new TByteIntHashMap();

   /**
    * A string identifier for a slider board that is plugged into this computer.
    */
   public static final String SLIDER_BOARD_NAME = findSliderBoard();

   private static String findSliderBoard()
   {
      String identifier = null;
      String bcf2000 = "BCF2000";

      for (MidiDevice.Info info : MidiSystem.getMidiDeviceInfo())
      {
         String name = info.getName();
         String description = info.getDescription();

         if (name.contains(bcf2000) || description.contains(bcf2000))
         {
            setupForBCF2000();
            identifier = bcf2000;
         }
      }

      SLIDER_CONTROLLER_MAP.forEachEntry((slider, controller) -> {
         CONTROLLER_SLIDER_MAP.put(controller, slider);
         return true;
      });

      if (identifier == null)
      {
         PrintTools.info("Was not able to find a supported sliderboard.");
      }
      else
      {
         PrintTools.info("Found sliderboard " + identifier);
      }
      return identifier;
   }

   private static void setupForBCF2000()
   {
      // This slider to MIDI controller map is for the BCF2000 slider board.
      SLIDER_CONTROLLER_MAP.put(0, (byte) 81); // sliders
      SLIDER_CONTROLLER_MAP.put(1, (byte) 82);
      SLIDER_CONTROLLER_MAP.put(2, (byte) 83);
      SLIDER_CONTROLLER_MAP.put(3, (byte) 84);
      SLIDER_CONTROLLER_MAP.put(4, (byte) 85);
      SLIDER_CONTROLLER_MAP.put(5, (byte) 86);
      SLIDER_CONTROLLER_MAP.put(6, (byte) 87);
      SLIDER_CONTROLLER_MAP.put(7, (byte) 88);

      SLIDER_CONTROLLER_MAP.put(8, (byte) 1); // knobs
      SLIDER_CONTROLLER_MAP.put(9, (byte) 2);
      SLIDER_CONTROLLER_MAP.put(10, (byte) 3);
      SLIDER_CONTROLLER_MAP.put(11, (byte) 4);
      SLIDER_CONTROLLER_MAP.put(12, (byte) 5);
      SLIDER_CONTROLLER_MAP.put(13, (byte) 6);
      SLIDER_CONTROLLER_MAP.put(14, (byte) 7);
      SLIDER_CONTROLLER_MAP.put(15, (byte) 8);
   }
}
