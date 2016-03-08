package us.ihmc.simulationconstructionset.commands;

import us.ihmc.simulationconstructionset.gui.ViewportWindow;


public interface CreateNewViewportWindowCommandExecutor
{
   public abstract ViewportWindow createNewViewportWindow();
   public abstract ViewportWindow createNewViewportWindow(String viewportName);
   public abstract ViewportWindow createNewViewportWindow(String viewportName, int screenID, boolean maximizeWindow);
   public abstract ViewportWindow getViewportWindow(String windowName);

}
