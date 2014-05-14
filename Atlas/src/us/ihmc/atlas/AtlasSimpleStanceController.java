package us.ihmc.atlas;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.referenceFrames.ReferenceFrames;
import us.ihmc.commonWalkingControlModules.terrain.TerrainType;
import us.ihmc.darpaRoboticsChallenge.DRCSCSInitialSetup;
import us.ihmc.darpaRoboticsChallenge.DRCSimulationVisualizer;
import us.ihmc.darpaRoboticsChallenge.controllers.SimpleStanceController;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPhysicalProperties;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class AtlasSimpleStanceController {

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
	            model = AtlasRobotModelFactory.createDRCRobotModel(config.getString("robotModel"), false, false);
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
	      DRCRobotPhysicalProperties physicalProperties = model.getPhysicalProperties();
	      SDFFullRobotModel fullRobotModel = model.createFullRobotModel();
	      SDFRobot robot = model.createSdfRobot(false);
	      ReferenceFrames referenceFrames = new ReferenceFrames(fullRobotModel, jointMap, physicalProperties.getAnkleHeight());

	      DRCRobotInitialSetup<SDFRobot> intialSetup = model.getDefaultRobotInitialSetup(0, 0);
	      intialSetup.initializeRobot(robot, jointMap);
	      WalkingControllerParameters walkingControlParams = model.getWalkingControlParameters();
	      double footForward =  walkingControlParams.getFootForwardOffset();
	      double footBack = walkingControlParams.getFootBackwardOffset();
	      double footWidth = walkingControlParams.getFootWidth();

	      double controlDT = 0.005;
	      InverseDynamicsJoint[] jointsToOptimize = SimpleStanceController.createJointsToOptimize(fullRobotModel);
	      double gravityZ = -robot.getGravityZ();
	      SimpleStanceController controller = new SimpleStanceController(robot, fullRobotModel, referenceFrames, controlDT, jointsToOptimize, gravityZ,
	                                             footForward, footBack, footWidth);
	      controller.initialize();



	      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListsRegistry = new DynamicGraphicObjectsListRegistry();
	      new DRCSimulationVisualizer(robot, dynamicGraphicObjectsListsRegistry);

	      double simDT = 1e-4;
	      int simulationTicksPerControlTick = (int) (controlDT / simDT);
	      robot.setController(controller, simulationTicksPerControlTick);

	      DRCSCSInitialSetup drcscsInitialSetup = new DRCSCSInitialSetup(TerrainType.FLAT_Z_ZERO, simDT);
	      drcscsInitialSetup.initializeRobot(robot, dynamicGraphicObjectsListsRegistry);

	      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
	      scs.setDT(simDT, 50);
	      dynamicGraphicObjectsListsRegistry.addDynamicGraphicsObjectListsToSimulationConstructionSet(scs);

	      scs.startOnAThread();
	   }

}
