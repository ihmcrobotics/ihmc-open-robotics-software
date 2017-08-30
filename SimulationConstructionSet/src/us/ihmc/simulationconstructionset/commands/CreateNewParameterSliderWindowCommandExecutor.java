package us.ihmc.simulationconstructionset.commands;

import us.ihmc.simulationconstructionset.gui.ParameterSliderWindow;
import us.ihmc.simulationconstructionset.gui.ViewportWindow;

/**
 * Created by Peter on 8/29/2017.
 */

public interface CreateNewParameterSliderWindowCommandExecutor
{
   //TODO: change this from Viewport to Slider GUI
   public abstract ParameterSliderWindow createNewParameterSliderWindow();
   public abstract ParameterSliderWindow createNewParameterSliderWindow(String viewportName);
   public abstract ParameterSliderWindow createNewParameterSliderWindow(String viewportName, int screenID, boolean maximizeWindow);
   public abstract ParameterSliderWindow getParameterSliderWindow(String windowName);
}
