package us.ihmc.simulationConstructionSetTools.util.inputdevices;

public class GenericChannelMapper implements MidiChannelMapper
{

   @Override
   public int getKnobChannel(int knob)
   {
      return knob;
   }

   @Override
   public int getSliderChannel(int slider)
   {
      return slider;
   }

   @Override
   public int getButtonChannel(int button)
   {
      return button;
   }

   @Override
   public int getKnobButtonChannel(int knob)
   {
      return knob;
   }

}
