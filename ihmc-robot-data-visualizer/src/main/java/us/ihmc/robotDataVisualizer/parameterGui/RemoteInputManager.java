package us.ihmc.robotDataVisualizer.parameterGui;

import java.util.List;

import javafx.scene.Node;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotics.parameterGui.GuiParameter;
import us.ihmc.robotics.parameterGui.GuiRegistry;
import us.ihmc.robotics.parameterGui.ParameterGuiInterface;

public class RemoteInputManager implements ParameterGuiInterface
{
   private final ParameterUpdateListener updateListener;

   public RemoteInputManager()
   {
      updateListener = new ParameterUpdateListener();
      YoVariableClient client = new YoVariableClient(updateListener);
      client.start();
   }

   @Override
   public boolean pollReloadAll()
   {
      return updateListener.hasNewGuiRegistry();
   }

   @Override
   public GuiRegistry getFullRegistryCopy()
   {
      return updateListener.pollGuiRegistry();
   }

   @Override
   public void submitChangedParameters(List<GuiParameter> changedParameters)
   {
      updateListener.changeVariables(changedParameters);
   }

   @Override
   public List<GuiParameter> pollUpdatedParameters()
   {
      return updateListener.getChangedParametersAndClear();
   }

   @Override
   public Node getInputManagerNode()
   {
      return null;
   }

   @Override
   public void shutdown()
   {
      updateListener.exitActionPerformed();
   }
}
