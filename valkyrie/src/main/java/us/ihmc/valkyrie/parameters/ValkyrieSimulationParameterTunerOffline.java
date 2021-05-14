package us.ihmc.valkyrie.parameters;

import java.nio.file.Path;
import java.nio.file.Paths;

import org.apache.commons.io.FilenameUtils;

import us.ihmc.parameterTuner.guiElements.main.ParameterGuiInterface;
import us.ihmc.parameterTuner.guiElements.main.ParameterTuningApplication;
import us.ihmc.parameterTuner.offline.FileInputManager;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieSimulationParameterTunerOffline extends ParameterTuningApplication
{
   @Override
   protected ParameterGuiInterface createInputManager()
   {
      String relativeFilePath = FilenameUtils.separatorsToSystem(ValkyrieRobotModel.getSimulationParameterResourceName());
      Path filePath = Paths.get("src", "main", "resources", relativeFilePath);
      return new FileInputManager(filePath.toFile());
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
