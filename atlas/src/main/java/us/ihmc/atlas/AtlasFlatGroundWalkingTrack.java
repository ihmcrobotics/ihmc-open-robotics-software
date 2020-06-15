package us.ihmc.atlas;

import com.martiansoftware.jsap.JSAPException;

import us.ihmc.avatar.DRCFlatGroundWalkingTrack;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationToolkit.controllers.OscillateFeetPerturber;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.ground.BumpyGroundProfile;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;

public class AtlasFlatGroundWalkingTrack
{
   // Give a default model to use is one isn't input
   private static final DRCRobotModel defaultModelForGraphicSelector = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
<<<<<<< refs/remotes/origin/feature/SandboxState
   
   //Setting Parameters
   private static final boolean USE_BUMPY_GROUND = false;
=======

   private static final boolean USE_BUMPY_GROUND = true;
>>>>>>> Testing commits and push
   private static final boolean USE_FEET_PERTURBER = false;
   private static final boolean Ground_Profile_Visible = true;
   private static final boolean Draw_Plane_At_Zero_Height = false;
   
   
   public static void main(String[] args) throws JSAPException
   {
      
      // This main could be called with arguments, namely the default model to use, but if it is called 
      // empty then the default model will be filled in
      DRCRobotModel model = null;
      model = AtlasRobotModelFactory.selectSimulationModelFromFlag(args);

      if (model == null)
         model = AtlasRobotModelFactory.selectModelFromGraphicSelector(defaultModelForGraphicSelector);

      if (model == null)
         throw new RuntimeException("No robot model selected");
      
      // Setup GUI with a bunch of settings on how it should look
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(Ground_Profile_Visible, Draw_Plane_At_Zero_Height);
      
      //Define Ground Profile
      final double groundHeight = 0.0;
      GroundProfile3D groundProfile;
      if (USE_BUMPY_GROUND)
      {
         groundProfile = createBumpyGroundProfile();
      }
      else
      {
         groundProfile = new FlatGroundProfile(groundHeight);
      }
      
      //Create SCS setup object with proper settings
      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, model.getSimulateDT());
      scsInitialSetup.setDrawGroundProfile(true);
      scsInitialSetup.setInitializeEstimatorToActual(true);
      
      // create robot initial state
      double initialYaw = 0.3;
      DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = model.getDefaultRobotInitialSetup(groundHeight, initialYaw);

      boolean useVelocityAndHeadingScript = true;
      boolean cheatWithGroundHeightAtForFootstep = false;
      
      // Begin the DRC Flat Ground Walking Track Simulation (it is general so we pass in the robot model and all the initial settings we defined
      DRCFlatGroundWalkingTrack drcFlatGroundWalkingTrack = new DRCFlatGroundWalkingTrack(robotInitialSetup, guiInitialSetup, scsInitialSetup,
            useVelocityAndHeadingScript, cheatWithGroundHeightAtForFootstep, model);

      if (USE_FEET_PERTURBER)
         createOscillateFeetPerturber(drcFlatGroundWalkingTrack);
   }

   private static void createOscillateFeetPerturber(DRCFlatGroundWalkingTrack drcFlatGroundWalkingTrack)
   {
      SimulationConstructionSet simulationConstructionSet = drcFlatGroundWalkingTrack.getSimulationConstructionSet();
      HumanoidFloatingRootJointRobot robot = drcFlatGroundWalkingTrack.getAvatarSimulation().getHumanoidFloatingRootJointRobot();

      int ticksPerPerturbation = 10;
      OscillateFeetPerturber oscillateFeetPerturber = new OscillateFeetPerturber(robot, simulationConstructionSet.getDT() * ((double) ticksPerPerturbation));
      oscillateFeetPerturber.setTranslationMagnitude(new double[] { 0.01, 0.015, 0.005 });
      oscillateFeetPerturber.setRotationMagnitudeYawPitchRoll(new double[] { 0.017, 0.012, 0.011 });

      oscillateFeetPerturber.setTranslationFrequencyHz(RobotSide.LEFT, new double[] { 0.0, 0, 3.3 });
      oscillateFeetPerturber.setTranslationFrequencyHz(RobotSide.RIGHT, new double[] { 0.0, 0, 1.3 });

      oscillateFeetPerturber.setRotationFrequencyHzYawPitchRoll(RobotSide.LEFT, new double[] { 0.0, 0, 7.3 });
      oscillateFeetPerturber.setRotationFrequencyHzYawPitchRoll(RobotSide.RIGHT, new double[] { 0., 0, 1.11 });

      robot.setController(oscillateFeetPerturber, ticksPerPerturbation);
   }

   private static BumpyGroundProfile createBumpyGroundProfile()
   {
      double xAmp1 = 0.05, xFreq1 = 0.5, xAmp2 = 0.01, xFreq2 = 0.5;
      double yAmp1 = 0.01, yFreq1 = 0.07, yAmp2 = 0.05, yFreq2 = 0.37;
      BumpyGroundProfile groundProfile = new BumpyGroundProfile(xAmp1, xFreq1, xAmp2, xFreq2, yAmp1, yFreq1, yAmp2, yFreq2);
      return groundProfile;
   }
}
