package us.ihmc.atlas.parameters;

import java.io.File;
import java.net.URISyntaxException;
import java.net.URL;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.parameterTuner.guiElements.main.ParameterGuiInterface;
import us.ihmc.parameterTuner.guiElements.main.ParameterTuningApplication;
import us.ihmc.parameterTuner.offline.FileInputManager;

public class AtlasParameterFileTuner extends ParameterTuningApplication
{
   @Override
   protected ParameterGuiInterface createInputManager()
   {
      URL parameterFile = AtlasRobotModel.class.getResource(AtlasRobotModel.getParameterFilePath());

      try
      {
         return new FileInputManager(new File(parameterFile.toURI()));
      }
      catch (URISyntaxException e)
      {
         throw new RuntimeException(e);
      }
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
