package us.ihmc.atlas;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.terrain.TerrainType;
import us.ihmc.darpaRoboticsChallenge.DRCSCSInitialSetup;
import us.ihmc.darpaRoboticsChallenge.DRCSimulationVisualizer;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.wholeBodyController.SimpleStanceController;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

public class AtlasSimpleStanceController
{
   public static void main(String[] args)
   {
      // Flag to set robot model
      JSAP jsap = new JSAP();
      FlaggedOption robotModel = new FlaggedOption("robotModel").setLongFlag("model").setShortFlag('m').setRequired(true).setStringParser(JSAP.STRING_PARSER);
      robotModel.setHelp("Robot models: " + AtlasRobotModelFactory.robotModelsToString());

      DRCRobotModel model;
      try
      {
         jsap.registerParameter(robotModel);

         JSAPResult config = jsap.parse(args);

         if (config.success())
         {
            model = AtlasRobotModelFactory.createDRCRobotModel(config.getString("robotModel"), DRCRobotModel.RobotTarget.SCS, false);
         }
         else
         {
            System.out.println("Enter a robot model.");
            return;
         }
      }
      catch (JSAPException e)
      {
         e.printStackTrace();
         return;
      }

      DRCRobotJointMap jointMap = model.getJointMap();
      SDFFullHumanoidRobotModel fullRobotModel = model.createFullRobotModel();
      SDFHumanoidRobot robot = model.createSdfRobot(false);
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);

      DRCRobotInitialSetup<SDFHumanoidRobot> intialSetup = model.getDefaultRobotInitialSetup(0, 0);
      intialSetup.initializeRobot(robot, jointMap);
      WalkingControllerParameters walkingControlParams = model.getWalkingControllerParameters();
      double footForward = walkingControlParams.getFootForwardOffset();
      double footBack = walkingControlParams.getFootBackwardOffset();
      double footWidth = walkingControlParams.getFootWidth();

      double controlDT = 0.005;
      InverseDynamicsJoint[] jointsToOptimize = SimpleStanceController.createJointsToOptimize(fullRobotModel);
      double gravityZ = -robot.getGravityZ();
      SimpleStanceController controller = new SimpleStanceController(robot, fullRobotModel, referenceFrames, controlDT, jointsToOptimize, gravityZ,
            footForward, footBack, footWidth);
      controller.initialize();

      YoGraphicsListRegistry yoGraphicsListsRegistry = new YoGraphicsListRegistry();
      new DRCSimulationVisualizer(robot, yoGraphicsListsRegistry);

      double simDT = 1e-4;
      int simulationTicksPerControlTick = (int) (controlDT / simDT);
      robot.setController(controller, simulationTicksPerControlTick);

      DRCSCSInitialSetup drcscsInitialSetup = new DRCSCSInitialSetup(TerrainType.FLAT_Z_ZERO, simDT);
      drcscsInitialSetup.initializeRobot(robot, model, yoGraphicsListsRegistry);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.setDT(simDT, 50);
      scs.addYoGraphicsListRegistry(yoGraphicsListsRegistry);

      scs.startOnAThread();
   }
}
