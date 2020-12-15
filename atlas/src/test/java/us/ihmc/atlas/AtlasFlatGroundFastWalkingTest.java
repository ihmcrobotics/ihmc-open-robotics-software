package us.ihmc.atlas;

import java.io.File;
import java.io.InputStream;
import java.nio.file.Path;
import java.nio.file.Paths;

import org.apache.commons.io.FilenameUtils;
import org.junit.jupiter.api.Test;

import javafx.application.Application;
import us.ihmc.avatar.AvatarFlatGroundFastWalkingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.parameterTuner.guiElements.main.ParameterGuiInterface;
import us.ihmc.parameterTuner.guiElements.main.ParameterTuningApplication;
import us.ihmc.parameterTuner.offline.FileInputManager;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;

public class AtlasFlatGroundFastWalkingTest extends AvatarFlatGroundFastWalkingTest
{
   private static final String FAST_WALKING_PARAMETERS_XML = "/us/ihmc/atlas/fast_walking_parameters.xml";

   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS)
      {
         @Override
         public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes, boolean enableJointDamping)
         {
            return super.createHumanoidFloatingRootJointRobot(createCollisionMeshes, false);
         }

         @Override
         public InputStream getParameterOverwrites()
         {
            return AtlasFlatGroundFastWalkingTest.class.getResourceAsStream(FAST_WALKING_PARAMETERS_XML);
         }

         @Override
         public double getControllerDT()
         {
            return 0.002;
         }

         @Override
         public double getSimulateDT()
         {
            return 0.0005;
         }
      };
   }

   @Override
   public double getFastSwingTime()
   {
      return 0.4;
   }

   @Override
   public double getFastTransferTime()
   {
      return 0.05;
   }

   @Test
   @Override
   public void testForwardWalking() throws Exception
   {
      super.testForwardWalking();
   }

   public static void main(String[] args)
   {
      Application.launch(AtlasFastWalkingTunerOffline.class, args);
   }

   public static class AtlasFastWalkingTunerOffline extends ParameterTuningApplication
   {
      @Override
      protected ParameterGuiInterface createInputManager()
      {
         System.out.println(new File("."));
         String relativeFilePath = FilenameUtils.separatorsToSystem(FAST_WALKING_PARAMETERS_XML);
         Path filePath = Paths.get("resources", relativeFilePath);
         return new FileInputManager(filePath.toFile());
      }
   }
}
