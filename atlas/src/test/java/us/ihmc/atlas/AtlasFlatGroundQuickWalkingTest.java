package us.ihmc.atlas;

import java.io.File;
import java.io.InputStream;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Objects;

import org.apache.commons.io.FilenameUtils;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.parameters.AtlasCoPTrajectoryParameters;
import us.ihmc.atlas.parameters.AtlasSwingTrajectoryParameters;
import us.ihmc.atlas.parameters.AtlasToeOffParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.AvatarFlatGroundQuickWalkingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryParameters;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.javafx.ApplicationNoModule;
import us.ihmc.parameterTuner.guiElements.main.ParameterGuiInterface;
import us.ihmc.parameterTuner.guiElements.main.ParameterTuningApplication;
import us.ihmc.parameterTuner.offline.FileInputManager;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;

public class AtlasFlatGroundQuickWalkingTest extends AvatarFlatGroundQuickWalkingTest
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
            InputStream resourceAsStream = AtlasFlatGroundQuickWalkingTest.class.getResourceAsStream(FAST_WALKING_PARAMETERS_XML);
            Objects.requireNonNull(resourceAsStream);
            return resourceAsStream;
         }

         @Override
         public CoPTrajectoryParameters getCoPTrajectoryParameters()
         {
            return new AtlasCoPTrajectoryParameters()
            {
               @Override
               public int getMaxNumberOfStepsToConsider()
               {
                  return 5;
               }
            };
         }

         @Override
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return new AtlasWalkingControllerParameters(getTarget(), getJointMap(), getContactPointParameters())
            {
               @Override
               public boolean controlHeightWithMomentum()
               {
                  return false;
               }

               @Override
               public SwingTrajectoryParameters getSwingTrajectoryParameters()
               {
                  return new AtlasSwingTrajectoryParameters(getTarget(), getJointMap().getModelScale())
                  {
                     @Override
                     public double getDesiredTouchdownHeightOffset()
                     {
                        return -0.005;
                     }

                     @Override
                     public Tuple3DReadOnly getTouchdownVelocityWeight()
                     {
                        return new Vector3D(30.0, 30.0, 30.0);
                     }
                  };
               }

               @Override
               public ToeOffParameters getToeOffParameters()
               {
                  return new AtlasToeOffParameters(getJointMap())
                  {
                     @Override
                     public boolean doToeOffIfPossibleInSingleSupport()
                     {
                        return true;
                     }
                  };
               }
            };
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
      return 0.45;
   }

   @Override
   public double getFastTransferTime()
   {
      return 0.05;
   }

   @Override
   public double getMaxForwardStepLength()
   {
      return 0.525;
   }

   @Tag("humanoid-flat-ground")
   @Test
   @Override
   public void testForwardWalking() throws Exception
   {
      super.testForwardWalking();
   }

   public static void main(String[] args)
   {
      ApplicationNoModule.launch(AtlasFastWalkingTunerOffline.class, args);
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
