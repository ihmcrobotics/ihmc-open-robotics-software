package us.ihmc.atlas.parameters;

import java.nio.file.Path;
import java.nio.file.Paths;

import org.apache.commons.io.FilenameUtils;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.parameterTuner.guiElements.main.ParameterGuiInterface;
import us.ihmc.parameterTuner.guiElements.main.ParameterTuningApplication;
import us.ihmc.parameterTuner.offline.FileInputManager;

public class AtlasParameterFileTuner extends ParameterTuningApplication
{
   @Override
   protected ParameterGuiInterface createInputManager()
   {
      String relativeFilePath = FilenameUtils.separatorsToSystem(AtlasRobotModel.getParameterResourceName());
      Path filePath = Paths.get("src", "main", "resources", relativeFilePath);
      return new FileInputManager(filePath.toFile());
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
