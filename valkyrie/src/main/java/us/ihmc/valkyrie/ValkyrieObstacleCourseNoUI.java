package us.ihmc.valkyrie;

import com.martiansoftware.jsap.JSAPException;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.HumanoidNetworkProcessor;
import us.ihmc.avatar.networkProcessor.HumanoidNetworkProcessorParameters;
import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.avatar.simulationStarter.DRCSimulationTools;
import us.ihmc.communication.net.LocalObjectCommunicator;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;

public class ValkyrieObstacleCourseNoUI {
	public static void main(final String[] args) throws JSAPException {
		DRCRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS);
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