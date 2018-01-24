package us.ihmc.robotics.parameterGui.gui;

import us.ihmc.robotics.parameterGui.ParameterGuiInterface;

public class ParameterFileTuner extends ParameterTuningApplication
{
   @Override
   protected ParameterGuiInterface createInputManager()
   {
      return new FileInputManager();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
