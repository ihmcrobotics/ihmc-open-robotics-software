package us.ihmc.simulationconstructionsettools.util.inputdevices;

public interface SliderBoardTransmitterInterface
{
   public abstract void moveControl(MidiControl midiControl);
   public abstract void moveControl(MidiControl midiControl, int sliderValue);
   public abstract void closeAndDispose();
}
