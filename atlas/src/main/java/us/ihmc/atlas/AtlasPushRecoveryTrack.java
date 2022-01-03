package us.ihmc.atlas;

import com.martiansoftware.jsap.JSAPException;

import us.ihmc.avatar.DRCFlatGroundWalkingTrack;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.initialSetup.RobotInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.validation.YoVariableThreadAccessValidator;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.simulationToolkit.controllers.PushRobotController;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;

public class AtlasPushRecoveryTrack
{
   private static final DRCRobotModel defaultModelForGraphicSelector = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
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
      RobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = model.getDefaultRobotInitialSetup(groundHeight, initialYaw);

      boolean useVelocityAndHeadingScript = true;
      boolean cheatWithGroundHeightAtForFootstep = false;

      DRCFlatGroundWalkingTrack track = new DRCFlatGroundWalkingTrack(robotInitialSetup, guiInitialSetup, scsInitialSetup, useVelocityAndHeadingScript,
            cheatWithGroundHeightAtForFootstep, model);

      FloatingRootJointRobot robot = track.getAvatarSimulation().getHumanoidFloatingRootJointRobot();
      FullHumanoidRobotModel fullRobotModel = model.createFullRobotModel();
      PushRobotController pushRobotController = new PushRobotController(robot, fullRobotModel);

      pushRobotController.addPushButtonToSCS(track.getSimulationConstructionSet());
      
      double defaultForceDurationInSeconds = 0.15;
      double defaultForceMagnitude = 400.0;
      Vector3D defaultForceDirection = new Vector3D(1.0, 0.0, 0.0);
      
      SimulationConstructionSet scs = track.getSimulationConstructionSet();
      
      YoBoolean enable = (YoBoolean) scs.findVariable("enablePushRecovery");
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
