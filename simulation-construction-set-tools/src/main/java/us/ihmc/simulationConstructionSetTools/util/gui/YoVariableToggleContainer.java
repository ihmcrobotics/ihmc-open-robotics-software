package us.ihmc.simulationConstructionSetTools.util.gui;

import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.simulationconstructionset.NewDataListener;

public interface YoVariableToggleContainer
{
   public abstract void processingStateChange(boolean endStateValue);

   public abstract void handleStateChange();

   public abstract void registerWithVariableChangedListener(YoVariableChangedListener variableChangedListener);

   public abstract NewDataListener getDataListener();
}
