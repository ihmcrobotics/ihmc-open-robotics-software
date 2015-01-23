package us.ihmc.simulationconstructionset.util.gui;

import us.ihmc.simulationconstructionset.NewDataListener;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;

public interface YoVariableToggleContainer
{
   public abstract void processingStateChange(boolean endStateValue);

   public abstract void handleStateChange();

   public abstract void registerWithVariableChangedListener(VariableChangedListener variableChangedListener);

   public abstract NewDataListener getDataListener();
}
