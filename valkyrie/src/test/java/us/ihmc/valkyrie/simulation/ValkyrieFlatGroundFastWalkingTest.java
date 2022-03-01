package us.ihmc.valkyrie.simulation;

import java.io.File;
import java.io.InputStream;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Objects;

import org.apache.commons.io.FilenameUtils;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.AvatarFlatGroundFastWalkingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryParameters;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.javafx.ApplicationNoModule;
import us.ihmc.parameterTuner.guiElements.main.ParameterGuiInterface;
import us.ihmc.parameterTuner.guiElements.main.ParameterTuningApplication;
import us.ihmc.parameterTuner.offline.FileInputManager;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.parameters.ValkyrieCoPTrajectoryParameters;
import us.ihmc.valkyrie.parameters.ValkyrieSwingTrajectoryParameters;
import us.ihmc.valkyrie.parameters.ValkyrieWalkingControllerParameters;

public class ValkyrieFlatGroundFastWalkingTest extends AvatarFlatGroundFastWalkingTest
{
   private static final String FAST_WALKING_PARAMETERS_XML = "/us/ihmc/valkyrie/simulation/fast_walking_parameters.xml";

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
            InputStream resourceAsStream = ValkyrieFlatGroundFastWalkingTest.class.getResourceAsStream(FAST_WALKING_PARAMETERS_XML);
            Objects.requireNonNull(resourceAsStream);
            return resourceAsStream;
         }

         @Override
         public CoPTrajectoryParameters getCoPTrajectoryParameters()
         {
            return new ValkyrieCoPTrajectoryParameters()
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
            return new ValkyrieWalkingControllerParameters(getJointMap(), getRobotPhysicalProperties(), getTarget())
            {
               @Override
               public SwingTrajectoryParameters getSwingTrajectoryParameters()
               {
                  return new ValkyrieSwingTrajectoryParameters(getRobotPhysicalProperties(), getTarget())
                  {
                     @Override
                     public Tuple3DReadOnly getTouchdownVelocityWeight()
                     {
                        return new Vector3D(30.0, 30.0, 30.0);
                     }
                  };
               }

               @Override
               public boolean controlHeightWithMomentum()
               {
                  return false;
               }
            };
         }

         @Override
         public double getSimulateDT()
         {
            return 2.5e-4;
         }

         @Override
         public double getControllerDT()
         {
            return 0.002;
         }

         @Override
         public double getEstimatorDT()
         {
            return 0.001;
         }
      };
   }

   @Override
   public double getFastSwingTime()
   {
      return 0.55;
   }

   @Override
   public double getFastTransferTime()
   {
      return 0.05;
   }

   @Override
   public double getMaxForwardStepLength()
   {
      return 0.55;
   }

   @Test
   @Override
   public void testForwardWalking() throws Exception
   {
      super.testForwardWalking();
   }

   public static void main(String[] args)
   {
      ApplicationNoModule.launch(ValkyrieFastWalkingTunerOffline.class, args);
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
