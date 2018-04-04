package us.ihmc.robotics.sliderboard;

/**
 * A MIDI connection can be used to send or receive control commands. 127 controllers can
 * be supported by a MIDI device. In the case of a sliderboard each input slider or knob is
 * mapped to a control channel. This mapping is sliderboard dependent and needs to be defined
 * for each supported sliderboard by implementing this interface.
 */
public interface MidiControlMap
{
   /**
    * Gets the MIDI control channel (between 0 and 127) associated with the given knob index for a
    * sliderboard.
    *
    * @param knobIndex the index of a knob on the sliderboard.
    * @return the MIDI control channel associated with the knob index.
    */
   int getKnobChannel(int knobIndex);

   /**
    * Gets the knob index of the provided MIDI control channel. This is the inverse function of
    * {@link #getKnobChannel(int)}.
    *
    * @param knobChannel the MIDI channel associated with a knob on the slider board.
    * @return the index of the knob corresponding to the provided MIDI channel.
    */
   int getKnobIndex(int knobChannel);

   /**
    * Gets the MIDI control channel (between 0 and 127) associated with the given slider index for a
    * sliderboard.
    *
    * @param sliderIndex the index of a slider on the sliderboard.
    * @return the MIDI control channel associated with the slider index.
    */
   int getSliderChannel(int sliderIndex);

   /**
    * Gets the slider index of the provided MIDI control channel. This is the inverse function of
    * {@link #getSliderChannel(int)}.
    *
    * @param sliderChannel the MIDI channel associated with a slider on the slider board.
    * @return the index of the slider corresponding to the provided MIDI channel.
    */
   int getSliderIndex(int sliderChannel);

   static boolean isInRange(int index, int min, int max)
   {
      return index >= 0 && index < max;
   }
}
