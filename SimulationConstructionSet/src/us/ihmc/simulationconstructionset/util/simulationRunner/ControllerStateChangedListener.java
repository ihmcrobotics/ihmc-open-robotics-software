package us.ihmc.simulationconstructionset.util.simulationRunner;

public interface ControllerStateChangedListener
{
   public void controllerStateHasChanged(Enum<?> oldState, Enum<?> newState);
}
