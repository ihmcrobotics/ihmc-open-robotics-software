package us.ihmc.atlas;

import us.ihmc.atlas.parameters.AtlasPhysicalProperties;
import us.ihmc.atlas.parameters.AtlasSmoothCMPPlannerParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.DRCFlatGroundWalkingTrack;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeadingAndVelocityEvaluationScriptParameters;
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
         public InputStream getWholeBodyControllerParametersFile()
         {
            return getClass().getResourceAsStream(parameterFile);
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
      HeadingAndVelocityEvaluationScriptParameters walkingScriptParameters = new HeadingAndVelocityEvaluationScriptParameters();
      walkingScriptParameters.setMaxVelocity(0.45);
      walkingScriptParameters.setCruiseVelocity(0.35);
      
      double currentNominalHeightAboveAnkle = model.getWalkingControllerParameters().nominalHeightAboveAnkle();
      ((AtlasWalkingControllerParameters) model.getWalkingControllerParameters()).setNominalHeightAboveAnkle(currentNominalHeightAboveAnkle + 0.20);

      new DRCFlatGroundWalkingTrack(robotInitialSetup,
                                    guiInitialSetup,
                                    scsInitialSetup,
                                    useVelocityAndHeadingScript,
                                    cheatWithGroundHeightAtForFootstep,
                                    model,
                                    walkingScriptParameters,
                                    new SimpleWalkingControllerStateFactory());
   }

}
