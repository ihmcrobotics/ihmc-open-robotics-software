package us.ihmc.darpaRoboticsChallenge.networkProcessor.kinematicsToolboxModule;

import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import java.util.*;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.humanoidRobotics.HumanoidFloatingRootJointRobot;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.sensorProcessing.simulatedSensors.SDFPerfectSimulatedSensorReader;
import us.ihmc.sensorProcessing.frames.ReferenceFrames;
import us.ihmc.sensorProcessing.sensorData.JointConfigurationGatherer;
import us.ihmc.sensorProcessing.communication.producers.DRCPoseCommunicator;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;

public class KinematicToolboxDiagnosticEnvironment
{
   private final String name = "Floating Robot";
   private final RobotDescription robotDescription = new RobotDescription(name);
   private final ReferenceFrames referenceFrames = null;
   
   public KinematicToolboxDiagnosticEnvironment(DRCRobotModel drcRobotModel)
   {
      FullHumanoidRobotModel humanoidFullRobotModel = drcRobotModel.createFullRobotModel();
      HumanoidJointNameMap sdfJointNameMap = drcRobotModel.getJointMap();
      HumanoidFloatingRootJointRobot humanoidFloatingRobotModel = new HumanoidFloatingRootJointRobot(robotDescription, sdfJointNameMap);
      SDFPerfectSimulatedSensorReader sdfPerfectReader = new SDFPerfectSimulatedSensorReader(humanoidFloatingRobotModel, humanoidFullRobotModel, referenceFrames);
      
      ForceSensorDefinition[] forceSensorDefinitionArray = humanoidFullRobotModel.getForceSensorDefinitions();
      List<ForceSensorDefinition> forceSensorDefinitionList = Arrays.asList(forceSensorDefinitionArray);
      ForceSensorDataHolder forceSensorDataHolder = new ForceSensorDataHolder(forceSensorDefinitionList);
      JointConfigurationGatherer jointConfigurationGatherer = new JointConfigurationGatherer(humanoidFullRobotModel, forceSensorDataHolder);

//      DRCPoseCommunicator poseCommunicator = new DRCPoseCommunicator(humanoidFullRobotModel, jointConfigurationGatherer, sensorReader, dataProducer,
//                                                                     sensorOutputMapReadOnly, sensorRawOutputMapReadOnly, robotMotionStatusFromController,
//                                                                     sensorInformation, scheduler, new IHMCCommunicationKryoNetClassList());
   }

}
