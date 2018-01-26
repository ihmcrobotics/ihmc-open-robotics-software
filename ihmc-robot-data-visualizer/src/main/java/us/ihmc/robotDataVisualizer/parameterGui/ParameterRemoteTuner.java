package us.ihmc.robotDataVisualizer.parameterGui;

import us.ihmc.robotics.parameterGui.ParameterGuiInterface;
import us.ihmc.robotics.parameterGui.gui.ParameterTuningApplication;

public class ParameterRemoteTuner extends ParameterTuningApplication
{
   @Override
   protected ParameterGuiInterface createInputManager()
   {
      return new RemoteInputManager();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
