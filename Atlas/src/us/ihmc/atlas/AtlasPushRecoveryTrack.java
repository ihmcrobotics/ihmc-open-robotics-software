package us.ihmc.atlas;

import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.darpaRoboticsChallenge.DRCFlatGroundWalkingTrack;
import us.ihmc.darpaRoboticsChallenge.DRCGuiInitialSetup;
import us.ihmc.darpaRoboticsChallenge.DRCSCSInitialSetup;
import us.ihmc.darpaRoboticsChallenge.controllers.DRCPushRobotController;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.validation.YoVariableThreadAccessValidator;
import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;

import com.martiansoftware.jsap.JSAPException;

public class AtlasPushRecoveryTrack
{
   private static final DRCRobotModel defaultModelForGraphicSelector = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.SCS, false);
   private final static boolean VISUALIZE_FORCE = true;

   public static void main(String[] args) throws JSAPException
   {
      DRCRobotModel model = null;
      final double groundHeight = 0.0;

      model = AtlasRobotModelFactory.selectSimulationModelFromFlag(args);

      if (model == null)
         model = AtlasRobotModelFactory.selectModelFromGraphicSelector(defaultModelForGraphicSelector);

      if (model == null)
         throw new RuntimeException("No robot model selected");

      GroundProfile3D groundProfile = new FlatGroundProfile(groundHeight);

      YoVariableThreadAccessValidator.registerAccessValidator();
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false);

      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, model.getSimulateDT());
      scsInitialSetup.setDrawGroundProfile(true);
      scsInitialSetup.setInitializeEstimatorToActual(true);

      double initialYaw = 0.3;
      DRCRobotInitialSetup<SDFHumanoidRobot> robotInitialSetup = model.getDefaultRobotInitialSetup(groundHeight, initialYaw);

      boolean useVelocityAndHeadingScript = true;
      boolean cheatWithGroundHeightAtForFootstep = false;

      DRCFlatGroundWalkingTrack track = new DRCFlatGroundWalkingTrack(robotInitialSetup, guiInitialSetup, scsInitialSetup, useVelocityAndHeadingScript,
            cheatWithGroundHeightAtForFootstep, model);

      SDFRobot robot = track.getDrcSimulation().getRobot();
      SDFFullHumanoidRobotModel fullRobotModel = model.createFullRobotModel();
      DRCPushRobotController pushRobotController = new DRCPushRobotController(robot, fullRobotModel);

      pushRobotController.addPushButtonToSCS(track.getSimulationConstructionSet());
      
      double defaultForceDurationInSeconds = 0.15;
      double defaultForceMagnitude = 400.0;
      Vector3d defaultForceDirection = new Vector3d(1.0, 0.0, 0.0);
      
      SimulationConstructionSet scs = track.getSimulationConstructionSet();
      
      BooleanYoVariable enable = (BooleanYoVariable) scs.getVariable("enablePushRecovery");
      // enable push recovery
      enable.set(true);
      
      if(VISUALIZE_FORCE)
      {
         scs.addYoGraphic(pushRobotController.getForceVisualizer());
      }
      
      pushRobotController.setPushDuration(defaultForceDurationInSeconds);
      pushRobotController.setPushForceMagnitude(defaultForceMagnitude);
      pushRobotController.setPushForceDirection(defaultForceDirection);
   }
}
