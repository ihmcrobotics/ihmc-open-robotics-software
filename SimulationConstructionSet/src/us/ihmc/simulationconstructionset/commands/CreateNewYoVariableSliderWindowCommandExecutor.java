package us.ihmc.simulationconstructionset.commands;

import us.ihmc.simulationconstructionset.gui.YoVariableSliderWindow;

/**
 * Created by Peter on 8/29/2017.
 */

public interface CreateNewYoVariableSliderWindowCommandExecutor
{
   //TODO: change this from Viewport to Slider GUI
   public abstract YoVariableSliderWindow createNewYoVariableSliderWindow();
   public abstract YoVariableSliderWindow createNewYoVariableSliderWindow(String viewportName);
   public abstract YoVariableSliderWindow createNewYoVariableSliderWindow(String viewportName, int screenID, boolean maximizeWindow);
   public abstract YoVariableSliderWindow getParameterSliderWindow(String windowName);
}
