package us.ihmc.atlas;

import us.ihmc.avatar.DRCFlatGroundWalkingTrack;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.simpleWholeBodyWalking.SimpleControlManagerFactory;
import us.ihmc.simpleWholeBodyWalking.SimpleWalkingControllerStateFactory;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;

import java.io.InputStream;

public class SimpleAtlasFlatGroundWalkingTrack
{
   public static void main(String[] args)
   {

      DRCRobotModel model = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS,
                                                RobotTarget.SCS,
                                                false)
      {
         private static final String parameterFile = "/us/ihmc/atlas/parameters/experimental_controller_parameters.xml";

         @Override
         public String getParameterFileName()
         {
            return parameterFile;
         }

         @Override
         public InputStream getParameterOverwrites()
         {
            return null;
         }
      };

      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false);

      final double groundHeight = 0.0;
      GroundProfile3D groundProfile = new FlatGroundProfile(groundHeight);

      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, model.getSimulateDT());
      scsInitialSetup.setDrawGroundProfile(true);
      scsInitialSetup.setInitializeEstimatorToActual(true);

      double initialYaw = 0.3;
      DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = model.getDefaultRobotInitialSetup(groundHeight, initialYaw);

      boolean useVelocityAndHeadingScript = true;
      boolean cheatWithGroundHeightAtForFootstep = false;

      new DRCFlatGroundWalkingTrack(robotInitialSetup,
                                    guiInitialSetup,
                                    scsInitialSetup,
                                    useVelocityAndHeadingScript,
                                    cheatWithGroundHeightAtForFootstep,
                                    model,
                                    new SimpleWalkingControllerStateFactory());
   }

}
