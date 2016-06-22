package us.ihmc.llaQuadruped;

import java.io.IOException;

import us.ihmc.SdfLoader.OutputWriter;
import us.ihmc.SdfLoader.SDFFullQuadrupedRobotModel;
import us.ihmc.SdfLoader.SDFPerfectSimulatedOutputWriter;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.llaQuadruped.simulation.LLAQuadrupedGroundContactParameters;
import us.ihmc.quadrupedRobotics.QuadrupedTestAdministratorFactory;
import us.ihmc.quadrupedRobotics.communication.QuadrupedGlobalDataProducer;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.estimator.sensorProcessing.simulatedSensors.SDFQuadrupedPerfectSimulatedSensor;
import us.ihmc.quadrupedRobotics.estimator.stateEstimator.QuadrupedSensorInformation;
import us.ihmc.quadrupedRobotics.estimator.stateEstimator.QuadrupedStateEstimatorFactory;
import us.ihmc.quadrupedRobotics.factories.QuadrupedSimulationFactory;
import us.ihmc.quadrupedRobotics.model.QuadrupedModelFactory;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.model.QuadrupedStandPrepParameters;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactParameters;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.sensors.ContactSensorHolder;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.model.DesiredJointDataHolder;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.SimulatedSensorHolderAndReaderFromRobotFactory;
import us.ihmc.sensorProcessing.stateEstimation.SensorProcessingConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.DRCKinematicsBasedStateEstimator;

public class LLAQuadrupedTestFactory
{
   private static final double SIMULATION_DT = 0.00006;
   private static final double SIMULATION_GRAVITY = -9.81;
   private static final int RECORD_FREQUENCY = (int) (0.01 / SIMULATION_DT);
   private static final boolean USE_STATE_ESTIMATOR = false;
   private static final boolean SHOW_PLOTTER = true;
   private static final boolean USE_TRACK_AND_DOLLY = false;
   private static final boolean USE_NETWORKING = false;
   
   private QuadrupedSimulationFactory createSimulationFactory() throws IOException
   {
      QuadrupedModelFactory modelFactory = new LLAQuadrupedModelFactory();
      SDFRobot sdfRobot = modelFactory.createSdfRobot();
      SDFFullQuadrupedRobotModel fullRobotModel = modelFactory.createFullRobotModel();
      QuadrupedPhysicalProperties physicalProperties = new LLAQuadrupedPhysicalProperties();
      IHMCCommunicationKryoNetClassList netClassList = new LLAQuadrupedNetClassList();
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      SimulationConstructionSetParameters scsParameters = new SimulationConstructionSetParameters();
      QuadrupedStandPrepParameters standPrepParameters = new LLAQuadrupedStandPrepParameters();
      QuadrupedGroundContactParameters groundContactParameters = new LLAQuadrupedGroundContactParameters();
      SensorProcessingConfiguration sensorProcessingConfiguration = new LLAQuadrupedSensorProcessingConfiguration();
      QuadrupedSensorInformation sensorInformation = new LLAQuadrupedSensorInformation();
      StateEstimatorParameters stateEstimatorParameters = new LLAQuadrupedStateEstimatorParameters();
      
      QuadrupedReferenceFrames referenceFrames = new QuadrupedReferenceFrames(fullRobotModel, physicalProperties);
      OutputWriter outputWriter = new SDFPerfectSimulatedOutputWriter(sdfRobot, fullRobotModel);
      
      SensorReader sensorReader;
      if (USE_STATE_ESTIMATOR)
      {
         SixDoFJoint rootInverseDynamicsJoint = fullRobotModel.getRootJoint();
         IMUDefinition[] imuDefinitions = fullRobotModel.getIMUDefinitions();
         ForceSensorDefinition[] forceSensorDefinitions = fullRobotModel.getForceSensorDefinitions();
         ContactSensorHolder contactSensorHolder = null;
         RawJointSensorDataHolderMap rawJointSensorDataHolderMap = null;
         DesiredJointDataHolder estimatorDesiredJointDataHolder = null;

         SimulatedSensorHolderAndReaderFromRobotFactory sensorReaderFactory;
         sensorReaderFactory = new SimulatedSensorHolderAndReaderFromRobotFactory(sdfRobot, sensorProcessingConfiguration);
         sensorReaderFactory.build(rootInverseDynamicsJoint, imuDefinitions, forceSensorDefinitions, contactSensorHolder, rawJointSensorDataHolderMap,
                                   estimatorDesiredJointDataHolder, sdfRobot.getRobotsYoVariableRegistry());

         sensorReader = sensorReaderFactory.getSensorReader();
      }
      else
      {
         sensorReader = new SDFQuadrupedPerfectSimulatedSensor(sdfRobot, fullRobotModel, referenceFrames);
      }
      
      QuadrantDependentList<ContactablePlaneBody> contactableFeet;
      contactableFeet = QuadrupedStateEstimatorFactory.createFootContactableBodies(fullRobotModel, referenceFrames, physicalProperties);
      
      QuadrantDependentList<FootSwitchInterface> footSwitches;
      footSwitches = QuadrupedStateEstimatorFactory.createFootSwitches(contactableFeet, SIMULATION_GRAVITY, fullRobotModel, sdfRobot.getRobotsYoVariableRegistry());
      
      DRCKinematicsBasedStateEstimator stateEstimator;
      if (USE_STATE_ESTIMATOR)
      {
         stateEstimator = QuadrupedStateEstimatorFactory
               .createStateEstimator(sensorInformation, stateEstimatorParameters, fullRobotModel, sensorReader.getSensorOutputMapReadOnly(), contactableFeet,
                                     footSwitches, SIMULATION_GRAVITY, SIMULATION_DT, sdfRobot.getRobotsYoVariableRegistry(), yoGraphicsListRegistry);
      }
      else
      {
         stateEstimator = null;
      }
      
      PacketCommunicator packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.CONTROLLER_PORT, netClassList);
      packetCommunicator.connect();
      
      GlobalDataProducer globalDataProducer = new QuadrupedGlobalDataProducer(packetCommunicator);
      
      QuadrupedSimulationFactory simulationFactory = new QuadrupedSimulationFactory();
      simulationFactory.setControlDT(SIMULATION_DT);
      simulationFactory.setGravity(SIMULATION_GRAVITY);
      simulationFactory.setRecordFrequency(RECORD_FREQUENCY);
      simulationFactory.setGroundContactParameters(groundContactParameters);
      simulationFactory.setHeadController(null);
      simulationFactory.setGlobalDataProducer(globalDataProducer);
      simulationFactory.setModelFactory(modelFactory);
      simulationFactory.setSDFRobot(sdfRobot);
      simulationFactory.setSCSParameters(scsParameters);
      simulationFactory.setOutputWriter(outputWriter);
      simulationFactory.setSensorReader(sensorReader);
      simulationFactory.setShowPlotter(SHOW_PLOTTER);
      simulationFactory.setUseTrackAndDolly(USE_TRACK_AND_DOLLY);
      simulationFactory.setStandPrepParameters(standPrepParameters);
      simulationFactory.setFullRobotModel(fullRobotModel);
      simulationFactory.setPhysicalProperties(physicalProperties);
      simulationFactory.setYoGraphicsListRegistry(yoGraphicsListRegistry);
      simulationFactory.setFootSwitches(footSwitches);
      simulationFactory.setStateEstimator(stateEstimator);
      simulationFactory.setUseNetworking(USE_NETWORKING);
      return simulationFactory;
   }
   
   public QuadrupedTestAdministratorFactory createTestAdministratorFactory() throws IOException
   {
      QuadrupedTestAdministratorFactory testAdministratorFactory = new QuadrupedTestAdministratorFactory();
      testAdministratorFactory.setSimulationFactory(createSimulationFactory());
      return testAdministratorFactory;
   }
}
