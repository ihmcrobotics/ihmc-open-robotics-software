package us.ihmc.simulationConstructionSetTools.util.gui;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.simulationconstructionset.NewDataListener;

public interface YoVariableToggleContainer
{
   public abstract void processingStateChange(boolean endStateValue);

   public abstract void handleStateChange();

   public abstract void registerWithVariableChangedListener(VariableChangedListener variableChangedListener);

   public abstract NewDataListener getDataListener();
}
