package us.ihmc.robotics.parameterGui.gui;

import java.awt.event.ActionListener;

import us.ihmc.yoVariables.parameters.xml.Registry;

public interface ParameterGuiNetworkManager
{
   public boolean createNewConnection(ActionListener listener);

   public Registry getParameterCopy();

   public void disconnect();
}
