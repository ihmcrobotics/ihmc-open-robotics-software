package us.ihmc.darpaRoboticsChallenge.networkProcessor.kinematicsToolboxModule;

import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.humanoidRobotics.HumanoidFloatingRootJointRobot;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.sensorProcessing.simulatedSensors.SDFPerfectSimulatedSensorReader;
import us.ihmc.sensorProcessing.frames.ReferenceFrames;
import us.ihmc.sensorProcessing.sensorData.JointConfigurationGatherer;
import us.ihmc.sensorProcessing.communication.producers.DRCPoseCommunicator;

public class KinematicToolboxDiagnosticEnvironment
{
   private final String name = "Floating Robot";
   private final RobotDescription robotDescription = new RobotDescription(name);
   private final HumanoidJointNameMap sdfJointNameMap = null;

   private final HumanoidFloatingRootJointRobot floatingRobotModel = new HumanoidFloatingRootJointRobot(robotDescription, sdfJointNameMap);
   private final FullRobotModel fullRobotModel = null;
   private final ReferenceFrames referenceFrames = null;

   private final SDFPerfectSimulatedSensorReader sdfPerfectReader = new SDFPerfectSimulatedSensorReader(floatingRobotModel, fullRobotModel, referenceFrames);

   public KinematicToolboxDiagnosticEnvironment()
   {/*
      JointConfigurationGatherer jointConfigurationGatherer = new JointConfigurationGatherer(fullRobotModel, forceSensorDataHolderToSend);

      DRCPoseCommunicator poseCommunicator = new DRCPoseCommunicator(estimatorFullRobotModel, jointConfigurationGathererAndProducer, sensorReader, dataProducer,
                                                                     sensorOutputMapReadOnly, sensorRawOutputMapReadOnly, robotMotionStatusFromController,
                                                                     sensorInformation, scheduler, new IHMCCommunicationKryoNetClassList());
                                                                     estimatorController.setRawOutputWriter(poseCommunicator);
   */}

}
