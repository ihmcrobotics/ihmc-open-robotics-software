package us.ihmc.valkyrie;

import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;
import com.martiansoftware.jsap.Switch;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.HumanoidNetworkProcessor;
import us.ihmc.avatar.networkProcessor.HumanoidNetworkProcessorParameters;
import us.ihmc.avatar.networkProcessor.time.SimulationRosClockPPSTimestampOffsetProvider;
import us.ihmc.avatar.ros.RobotROSClockCalculator;
import us.ihmc.avatar.ros.RobotROSClockCalculatorFromPPSOffset;
import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.avatar.simulationStarter.DRCSimulationTools;
import us.ihmc.communication.net.LocalObjectCommunicator;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;

public class ValkyrieObstacleCourseNoUI {
	public static JSAP getArgumentParser() {
		JSAP argumentParser = new JSAP();
        Switch logDataSwitch = new Switch("logData").setShortFlag('l').setLongFlag("logData").setDefault("false");
        try {
        	argumentParser.registerParameter(logDataSwitch);
        } catch (JSAPException e1) {
        	System.err.println("Unable to register option parameters");
        	e1.printStackTrace();
        	System.exit(1);
        }
        return argumentParser;
	}
	
	public static void main(final String[] args) throws JSAPException {
		/* 
		 * Supported command-line arguments:
		 * -l/--logData: create a YoVariableServer and set up the robot model for logging
		 */
        JSAP argumentParser = getArgumentParser();
        JSAPResult arguments = argumentParser.parse(args);

        boolean logData = arguments.getBoolean("logData");
		
		/* 
		 * For this robot model and sim, we need to override the clock so that it provides sim time on the ROS1 /clock 
		 * topic for use with ROS1 nodes. The RobotROSClockCalculatorFromPPSOffset class is already available to do this; 
		 * we just need to make sure that the sim uses it.
		 */
		
		DRCRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS) {
			private RobotROSClockCalculatorFromPPSOffset rosSimClockCalculator = null;
			
			@Override
			public RobotROSClockCalculator getROSClockCalculator()
		    {
			       if (rosSimClockCalculator == null) {
			    	   rosSimClockCalculator = new RobotROSClockCalculatorFromPPSOffset(new SimulationRosClockPPSTimestampOffsetProvider());
			       }
			       return rosSimClockCalculator;
		    }
			
			@Override
			public DataServerSettings getLogSettings() {
				return new DataServerSettings(logData, "SimulationGUI");
			}
		};
		DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel,
				new DefaultCommonAvatarEnvironment()) {
			   protected void startNetworkProcessor(HumanoidNetworkProcessorParameters networkModuleParams)
			   {
			      if (networkModuleParams.isUseROSModule() || networkModuleParams.isUseSensorModule())
			      {
			         LocalObjectCommunicator simulatedSensorCommunicator = createSimulatedSensorsPacketCommunicator();
			         networkModuleParams.setSimulatedSensorCommunicator(simulatedSensorCommunicator);
			      }

			      networkProcessor = HumanoidNetworkProcessor.newFromParameters(robotModel, pubSubImplementation, networkModuleParams);
			      networkProcessor.start();
			   }			
		};

		simulationStarter.setRunMultiThreaded(true);
		simulationStarter.setCreateYoVariableServer(logData);
		DRCSimulationTools.startSimulationWithGraphicSelector(simulationStarter, null, null,
				DRCObstacleCourseStartingLocation.values());

		try {
			Thread.sleep(10000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}