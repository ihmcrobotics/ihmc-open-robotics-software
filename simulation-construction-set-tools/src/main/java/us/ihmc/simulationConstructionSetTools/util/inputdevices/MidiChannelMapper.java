package us.ihmc.simulationConstructionSetTools.util.inputdevices;

public interface MidiChannelMapper
{
   int getKnobChannel(int knob);
   int getSliderChannel(int slider);
   int getButtonChannel(int button);
   int getKnobButtonChannel(int knob);
}
