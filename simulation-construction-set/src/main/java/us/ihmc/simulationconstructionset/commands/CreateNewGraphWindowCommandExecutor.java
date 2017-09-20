package us.ihmc.simulationconstructionset.commands;

import java.awt.Dimension;
import java.awt.Point;

import us.ihmc.simulationconstructionset.gui.GraphArrayWindow;

public interface CreateNewGraphWindowCommandExecutor
{
   public abstract GraphArrayWindow createNewGraphWindow();
   public abstract GraphArrayWindow createNewGraphWindow(String graphGroupName);
   public abstract GraphArrayWindow createNewGraphWindow(String graphGroupName, int screenID, Point windowLocation, Dimension windowSize, boolean maximizeWindow);
   public abstract GraphArrayWindow getGraphArrayWindow(String windowName);

}
