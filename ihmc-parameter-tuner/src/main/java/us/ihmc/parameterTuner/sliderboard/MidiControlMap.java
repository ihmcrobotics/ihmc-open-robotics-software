package us.ihmc.parameterTuner.sliderboard;

/**
 * A MIDI connection can be used to send or receive control commands. 127 controllers can
 * be supported by a MIDI device. In the case of a sliderboard each input slider or knob is
 * mapped to a control channel. This mapping is sliderboard dependent and needs to be defined
 * for each supported sliderboard by implementing this interface.
 */
public interface MidiControlMap
{
   public static int INVALID = -1;

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

   /**
    * Gets the MIDI control channel (between 0 and 127) associated with the given button index for a
    * sliderboard.
    *
    * @param buttonIndex the index of a button on the sliderboard.
    * @return the MIDI control channel associated with the button index.
    */
   int getButtonChannel(int buttonIndex);

   /**
    * Gets the button index of the provided MIDI control channel. This is the inverse function of
    * {@link #getButtonChannel(int)}.
    *
    * @param buttonChannel the MIDI channel associated with a button on the slider board.
    * @return the index of the button corresponding to the provided MIDI channel.
    */
   int getButtonIndex(int buttonChannel);

   /**
    * Gets the channel associated with the "Delay/Variation Send" status.
    *
    * @return the index of the "Delay/Variation Send" controller.
    */
   int getDelayVariationChannel();

   static boolean isInRange(int index, int min, int max)
   {
      return index >= min && index <= max;
   }
}
