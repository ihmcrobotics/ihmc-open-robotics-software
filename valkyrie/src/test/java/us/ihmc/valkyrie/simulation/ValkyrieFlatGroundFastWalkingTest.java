package us.ihmc.valkyrie.simulation;

import java.io.File;
import java.io.InputStream;
import java.nio.file.Path;
import java.nio.file.Paths;

import org.apache.commons.io.FilenameUtils;
import org.junit.jupiter.api.Test;

import javafx.application.Application;
import us.ihmc.avatar.AvatarFlatGroundFastWalkingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.parameterTuner.guiElements.main.ParameterGuiInterface;
import us.ihmc.parameterTuner.guiElements.main.ParameterTuningApplication;
import us.ihmc.parameterTuner.offline.FileInputManager;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieFlatGroundFastWalkingTest extends AvatarFlatGroundFastWalkingTest
{
   private static final String FAST_WALKING_PARAMETERS_XML = "/us/ihmc/valkyrie/fast_walking_parameters.xml";

   @Override
   public DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS)
      {
         @Override
         public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes, boolean enableJointDamping)
         {
            return super.createHumanoidFloatingRootJointRobot(createCollisionMeshes, false);
         }

         @Override
         public InputStream getParameterOverwrites()
         {
            return ValkyrieFlatGroundFastWalkingTest.class.getResourceAsStream(FAST_WALKING_PARAMETERS_XML);
         }

         @Override
         public double getSimulateDT()
         {
            return 2.5e-4;
         }
      };
   }

   @Override
   public double getFastSwingTime()
   {
      return 0.50;
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
      Application.launch(ValkyrieFastWalkingTunerOffline.class, args);
   }

   public static class ValkyrieFastWalkingTunerOffline extends ParameterTuningApplication
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
