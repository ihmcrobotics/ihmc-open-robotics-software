package us.ihmc.exampleSimulations.genericQuadruped;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.exampleSimulations.genericQuadruped.model.GenericQuadrupedModelFactory;
import us.ihmc.exampleSimulations.genericQuadruped.model.GenericQuadrupedPhysicalProperties;
import us.ihmc.exampleSimulations.genericQuadruped.model.GenericQuadrupedSensorInformation;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.*;
import us.ihmc.exampleSimulations.genericQuadruped.simulation.GenericQuadrupedGroundContactParameters;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedFallDetectionParameters;
import us.ihmc.quadrupedRobotics.estimator.stateEstimator.QuadrupedSensorInformation;
import us.ihmc.quadrupedRobotics.model.QuadrupedInitialOffsetAndYaw;
import us.ihmc.quadrupedRobotics.model.QuadrupedInitialPositionParameters;
import us.ihmc.quadrupedRobotics.model.QuadrupedModelFactory;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.output.SimulatedQuadrupedOutputWriter;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedPrivilegedConfigurationParameters;
import us.ihmc.quadrupedRobotics.planning.trajectory.DCMPlannerParameters;
import us.ihmc.quadrupedRobotics.simulation.GroundContactParameters;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedSimulationFactory;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotModels.OutputWriter;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.sensorProcessors.SensorTimestampHolder;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

import java.io.IOException;

public class GenericQuadrupedSimulationFactory
{
   private final QuadrupedGroundContactModelType groundContactModelType = QuadrupedGroundContactModelType.FLAT;
   private static final double CONTROL_DT = 0.001;
   private static final double SIMULATION_DT = 1e-4;
   private static final double SIMULATION_GRAVITY = -9.81;
   private static final boolean USE_STATE_ESTIMATOR = false;
   private static final int RECORD_FREQUENCY = (int) (0.01 / SIMULATION_DT);
   private static final boolean SHOW_PLOTTER = true;
   private static final boolean USE_TRACK_AND_DOLLY = false;
   private static final boolean USE_NETWORKING = true;

   private QuadrupedSimulationFactory simulationFactory = new QuadrupedSimulationFactory();

   public SimulationConstructionSet createSimulation() throws IOException
   {
      QuadrupedModelFactory modelFactory = new GenericQuadrupedModelFactory();
      QuadrupedPhysicalProperties physicalProperties = new GenericQuadrupedPhysicalProperties();
      SimulationConstructionSetParameters scsParameters = new SimulationConstructionSetParameters();
      QuadrupedInitialPositionParameters initialPositionParameters = new GenericQuadrupedDefaultInitialPosition();
      GroundContactParameters groundContactParameters = new GenericQuadrupedGroundContactParameters();
      QuadrupedSensorInformation sensorInformation = new GenericQuadrupedSensorInformation();
      StateEstimatorParameters stateEstimatorParameters = new GenericQuadrupedStateEstimatorParameters(false, CONTROL_DT);
      GenericQuadrupedHighLevelControllerParameters highLevelControllerParameters = new GenericQuadrupedHighLevelControllerParameters(
            modelFactory.getJointMap());
      DCMPlannerParameters dcmPlannerParameters = new GenericQuadrupedDCMPlannerParameters();
      GenericQuadrupedSitDownParameters sitDownParameters = new GenericQuadrupedSitDownParameters();
      QuadrupedPrivilegedConfigurationParameters privilegedConfigurationParameters = new GenericQuadrupedPrivilegedConfigurationParameters();
      QuadrupedFallDetectionParameters fallDetectionParameters = new GenericQuadrupedFallDetectionParameters();

      FullQuadrupedRobotModel fullRobotModel = modelFactory.createFullRobotModel();
      FloatingRootJointRobot sdfRobot = new FloatingRootJointRobot(modelFactory.getRobotDescription());
      ControllerCoreOptimizationSettings controllerCoreOptimizationSettings = new GenericQuadrupedControllerCoreOptimizationSettings(
            fullRobotModel.getTotalMass());

      SensorTimestampHolder timestampProvider = new GenericQuadrupedTimestampProvider(sdfRobot);
      QuadrantDependentList<Double> kneeTorqueTouchdownDetectionThreshold = new QuadrantDependentList<>(20.0, 20.0, 20.0, 20.0);
      QuadrantDependentList<Double> kneeTorqueTouchdownForSureDetectionThreshold = new QuadrantDependentList<>(75.0, 75.0, 75.0, 75.0);

      JointDesiredOutputList jointDesiredOutputList = new JointDesiredOutputList(fullRobotModel.getOneDoFJoints());
      QuadrupedReferenceFrames referenceFrames = new QuadrupedReferenceFrames(fullRobotModel);
      OutputWriter outputWriter = new SimulatedQuadrupedOutputWriter(sdfRobot, fullRobotModel, jointDesiredOutputList, CONTROL_DT);

      simulationFactory.setControlDT(CONTROL_DT);
      simulationFactory.setSimulationDT(SIMULATION_DT);
      simulationFactory.setGravity(SIMULATION_GRAVITY);
      simulationFactory.setRecordFrequency(RECORD_FREQUENCY);
      simulationFactory.setGroundContactModelType(groundContactModelType);
      simulationFactory.setGroundContactParameters(groundContactParameters);
      simulationFactory.setModelFactory(modelFactory);
      simulationFactory.setSDFRobot(sdfRobot);
      simulationFactory.setKneeTorqueTouchdownDetectionThreshold(kneeTorqueTouchdownDetectionThreshold);
      simulationFactory.setKneeTorqueTouchdownForSureDetectionThreshold(kneeTorqueTouchdownForSureDetectionThreshold);
      simulationFactory.setSCSParameters(scsParameters);
      simulationFactory.setOutputWriter(outputWriter);
      simulationFactory.setShowPlotter(SHOW_PLOTTER);
      simulationFactory.setUseTrackAndDolly(USE_TRACK_AND_DOLLY);
      simulationFactory.setInitialPositionParameters(initialPositionParameters);
      simulationFactory.setFullRobotModel(fullRobotModel);
      simulationFactory.setControllerCoreOptimizationSettings(controllerCoreOptimizationSettings);
      simulationFactory.setPhysicalProperties(physicalProperties);
      simulationFactory.setUseNetworking(USE_NETWORKING);
      simulationFactory.setTimestampHolder(timestampProvider);
      simulationFactory.setUseStateEstimator(USE_STATE_ESTIMATOR);
      simulationFactory.setStateEstimatorParameters(stateEstimatorParameters);
      simulationFactory.setSensorInformation(sensorInformation);
      simulationFactory.setJointDesiredOutputList(jointDesiredOutputList);
      simulationFactory.setReferenceFrames(referenceFrames);
      simulationFactory.setInitialOffset(new QuadrupedInitialOffsetAndYaw());
      simulationFactory.setHighLevelControllerParameters(highLevelControllerParameters);
      simulationFactory.setDCMPlannerParameters(dcmPlannerParameters);
      simulationFactory.setSitDownParameters(sitDownParameters);
      simulationFactory.setPrivilegedConfigurationParameters(privilegedConfigurationParameters);
      simulationFactory.setFallDetectionParameters(fallDetectionParameters);

      return simulationFactory.createSimulation();
   }

   public static void main(String[] commandLineArguments)
   {
      try
      {
         GenericQuadrupedSimulationFactory simulationFactory = new GenericQuadrupedSimulationFactory();
         SimulationConstructionSet scs = simulationFactory.createSimulation();
         scs.startOnAThread();
         scs.simulate();
      }
      catch (IOException ioException)
      {
         ioException.printStackTrace();
      }
   }
}
