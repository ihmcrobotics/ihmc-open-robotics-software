package us.ihmc.atlas.hikSim;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
//import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
//import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.AtlasWholeBodyIKSimController;
import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
//import us.ihmc.wholeBodyController.WholeBodyControlParameters;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
//import wholeBodyInverseKinematicsController.WholeBodyIkController;

public class AtlasWholeBodyIKSimulation
{
   private static final double DT = 0.0001;
   private static final double SimulationTime = 10;
   private static final int DataFreq = 50;
   private static final int BufferSize = (int) (SimulationTime / DataFreq / DT + 1);

   public static final boolean showGUI = false; // don't commit this to true, will break bamboo
   private final SimulationConstructionSet scs;

   private final YoVariableRegistry registry = new YoVariableRegistry("WholeBodyIKSimulation");

   private GroundProfile3D groundProfile = new FlatGroundProfile(-10.0, 1000, -6.0 / 2.0, 6.0 / 2.0, 0);

   private final SDFFullHumanoidRobotModel fullRobotModel;
   private final SDFRobot scsRobot;
//   private final AtlasRobotModel atlasRobotModel;

   public AtlasWholeBodyIKSimulation(AtlasRobotModel atlasRobotModel) throws Exception
   {
      scsRobot = atlasRobotModel.createSdfRobot(false);
      fullRobotModel = atlasRobotModel.createFullRobotModel();
//      this.atlasRobotModel = atlasRobotModel;
      AtlasWholeBodyIKSimController controller = new AtlasWholeBodyIKSimController(scsRobot, fullRobotModel, atlasRobotModel);

      scsRobot.setController(controller);

      scsRobot.setGravity(-9.81);
      
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(); 
      parameters.setCreateGUI(showGUI);
      parameters.setDataBufferSize(BufferSize);
      scs = new SimulationConstructionSet(scsRobot, parameters);

      if (showGUI)
      {
         // Rewrite this view if you need it
         // PersoView.setupPersoViews(scs, "cameraTrackingCoordX", "cameraTrackingCoordY", "cameraTrackingCoordZ");
      }

      scs.setDT(DT, DataFreq);
      // Update graphs less frequently for better simulation speed
      scs.setFastSimulate(true);
      // Setup camera configuration
      scs.setCameraPosition(0.0, -8.0, -0.5);
      scs.setCameraFix(0.0, 0.0, -0.5);
      // Run simulation
      Thread myThread = new Thread(scs);
      myThread.start();

      sleepForSeconds(2.0);

      scs.simulate(SimulationTime);
   }

   public void sleepForSeconds(double sleepInSeconds)
   {
      try
      {
         Thread.sleep((long) (sleepInSeconds * 1000));
      }
      catch (InterruptedException e)
      {
      }
   }

   public static void main(String[] args) throws Exception
   {
      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, DRCRobotModel.RobotTarget.SCS, false);
      new AtlasWholeBodyIKSimulation(atlasRobotModel);
   }
}