package us.ihmc.simulationconstructionset.util.inputdevices;

public interface SliderBoardTransmitterInterface {

	public void moveControl(MidiControl ctrl);
	public void moveControl(MidiControl ctrl, int sliderValue);

}
