package us.ihmc.llaQuadruped;

import java.io.IOException;

import us.ihmc.SdfLoader.OutputWriter;
import us.ihmc.SdfLoader.SDFFullQuadrupedRobotModel;
import us.ihmc.SdfLoader.SDFPerfectSimulatedOutputWriter;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.llaQuadruped.simulation.LLAQuadrupedGroundContactParameters;
import us.ihmc.quadrupedRobotics.QuadrupedTestConductor;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.estimator.stateEstimator.QuadrupedSensorInformation;
import us.ihmc.quadrupedRobotics.factories.QuadrupedSimulationFactory;
import us.ihmc.quadrupedRobotics.model.QuadrupedModelFactory;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.model.QuadrupedSimulationInitialPositionParameters;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactParameters;
import us.ihmc.sensorProcessing.sensorProcessors.SensorTimestampHolder;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.RequiredFactoryField;

public class LLAQuadrupedTestFactory
{
   private static final double SIMULATION_DT = 0.00006;
   private static final double SIMULATION_GRAVITY = -9.81;
   private static final int RECORD_FREQUENCY = (int) (0.01 / SIMULATION_DT);
   private static final boolean USE_STATE_ESTIMATOR = false;
   private static final boolean SHOW_PLOTTER = true;
   private static final boolean USE_TRACK_AND_DOLLY = false;
   private static final boolean USE_NETWORKING = false;
   
   private RequiredFactoryField<QuadrupedControlMode> controlMode = new RequiredFactoryField<>("controlMode");
   private RequiredFactoryField<QuadrupedGroundContactModelType> groundContactModelType = new RequiredFactoryField<>("controlMode");
   
   public QuadrupedTestConductor createTestConductor() throws IOException
   {
      FactoryTools.checkAllRequiredFactoryFieldsAreSet(this);
      
      QuadrupedModelFactory modelFactory = new LLAQuadrupedModelFactory();
      QuadrupedPhysicalProperties physicalProperties = new LLAQuadrupedPhysicalProperties();
      NetClassList netClassList = new LLAQuadrupedNetClassList();
      SimulationConstructionSetParameters scsParameters = new SimulationConstructionSetParameters();
      QuadrupedSimulationInitialPositionParameters initialPositionParameters = new LLAQuadrupedSimulationInitialPositionParameters();
      QuadrupedGroundContactParameters groundContactParameters = new LLAQuadrupedGroundContactParameters();
      QuadrupedSensorInformation sensorInformation = new LLAQuadrupedSensorInformation();
      StateEstimatorParameters stateEstimatorParameters = new LLAQuadrupedStateEstimatorParameters();
      
      SDFFullQuadrupedRobotModel fullRobotModel = modelFactory.createFullRobotModel();
      SDFRobot sdfRobot = modelFactory.createSdfRobot();
      
      SensorTimestampHolder timestampProvider = new LLAQuadrupedTimestampProvider(sdfRobot);
      
      QuadrupedReferenceFrames referenceFrames = new QuadrupedReferenceFrames(fullRobotModel, physicalProperties);
      OutputWriter outputWriter = new SDFPerfectSimulatedOutputWriter(sdfRobot, fullRobotModel);
      
      QuadrupedSimulationFactory simulationFactory = new QuadrupedSimulationFactory();
      simulationFactory.setControlDT(SIMULATION_DT);
      simulationFactory.setGravity(SIMULATION_GRAVITY);
      simulationFactory.setRecordFrequency(RECORD_FREQUENCY);
      simulationFactory.setGroundContactParameters(groundContactParameters);
      simulationFactory.setHeadControllerFactory(null);
      simulationFactory.setModelFactory(modelFactory);
      simulationFactory.setSDFRobot(sdfRobot);
      simulationFactory.setSCSParameters(scsParameters);
      simulationFactory.setOutputWriter(outputWriter);
      simulationFactory.setShowPlotter(SHOW_PLOTTER);
      simulationFactory.setUseTrackAndDolly(USE_TRACK_AND_DOLLY);
      simulationFactory.setInitialPositionParameters(initialPositionParameters);
      simulationFactory.setFullRobotModel(fullRobotModel);
      simulationFactory.setPhysicalProperties(physicalProperties);
      simulationFactory.setUseNetworking(USE_NETWORKING);
      simulationFactory.setTimestampHolder(timestampProvider);
      simulationFactory.setUseStateEstimator(USE_STATE_ESTIMATOR);
      simulationFactory.setStateEstimatorParameters(stateEstimatorParameters);
      simulationFactory.setSensorInformation(sensorInformation);
      simulationFactory.setReferenceFrames(referenceFrames);
      simulationFactory.setNetClassList(netClassList);
      simulationFactory.setGroundContactModelType(groundContactModelType.get());
      return new QuadrupedTestConductor(simulationFactory.createSimulation());
   }
   
   public void setControlMode(QuadrupedControlMode controlMode)
   {
      this.controlMode.set(controlMode);
   }
   
   public void setGroundContactModelType(QuadrupedGroundContactModelType groundContactModelType)
   {
      this.groundContactModelType.set(groundContactModelType);
   }
}
