package us.ihmc.exampleSimulations.genericQuadruped;

import us.ihmc.communication.net.NetClassList;
import us.ihmc.exampleSimulations.genericQuadruped.model.GenericQuadrupedModelFactory;
import us.ihmc.exampleSimulations.genericQuadruped.model.GenericQuadrupedPhysicalProperties;
import us.ihmc.exampleSimulations.genericQuadruped.model.GenericQuadrupedSensorInformation;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedPositionBasedCrawlControllerParameters;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedSimulationInitialPositionParameters;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedStateEstimatorParameters;
import us.ihmc.exampleSimulations.genericQuadruped.simulation.GenericQuadrupedGroundContactParameters;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.controller.QuadrupedSimulationFactory;
import us.ihmc.quadrupedRobotics.controller.position.states.QuadrupedPositionBasedCrawlControllerParameters;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.estimator.stateEstimator.QuadrupedSensorInformation;
import us.ihmc.quadrupedRobotics.model.QuadrupedModelFactory;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.model.QuadrupedSimulationInitialPositionParameters;
import us.ihmc.quadrupedRobotics.simulation.GroundContactParameters;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedParameterSet;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotModels.OutputWriter;
import us.ihmc.robotics.dataStructures.parameter.ParameterRegistry;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.sensorProcessors.SensorTimestampHolder;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationToolkit.outputWriters.PerfectSimulatedOutputWriter;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.factories.RequiredFactoryField;

import java.io.IOException;

public class GenericQuadrupedTestFactory implements QuadrupedTestFactory
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final double SIMULATION_DT = 0.00006;
   private static final double SIMULATION_GRAVITY = -9.81;
   private static final int RECORD_FREQUENCY = (int) (0.01 / SIMULATION_DT);
   private static final boolean USE_STATE_ESTIMATOR = false;
   private static final boolean SHOW_PLOTTER = true;
   private static final boolean USE_TRACK_AND_DOLLY = true;
   private static final boolean USE_NETWORKING = false;

   private final RequiredFactoryField<QuadrupedControlMode> controlMode = new RequiredFactoryField<>("controlMode");

   private final OptionalFactoryField<Boolean> useStateEstimator = new OptionalFactoryField<>("useStateEstimator");
   private final OptionalFactoryField<QuadrupedGroundContactModelType> groundContactModelType = new OptionalFactoryField<>("groundContactModelType");
   private final OptionalFactoryField<GroundProfile3D> providedGroundProfile3D = new OptionalFactoryField<>("providedGroundProfile3D");
   private final OptionalFactoryField<Boolean> usePushRobotController = new OptionalFactoryField<>("usePushRobotController");
   private final OptionalFactoryField<QuadrupedParameterSet> parameterSet = new OptionalFactoryField<>("parameterSet");

   @Override
   public GoalOrientedTestConductor createTestConductor() throws IOException
   {
      useStateEstimator.setDefaultValue(USE_STATE_ESTIMATOR);
      usePushRobotController.setDefaultValue(false);

      FactoryTools.checkAllFactoryFieldsAreSet(this);

      QuadrupedModelFactory modelFactory = new GenericQuadrupedModelFactory();
      QuadrupedPhysicalProperties physicalProperties = new GenericQuadrupedPhysicalProperties();
      NetClassList netClassList = new GenericQuadrupedNetClassList();
      QuadrupedSimulationInitialPositionParameters initialPositionParameters = new GenericQuadrupedSimulationInitialPositionParameters();
      GroundContactParameters groundContactParameters = new GenericQuadrupedGroundContactParameters();
      QuadrupedSensorInformation sensorInformation = new GenericQuadrupedSensorInformation();
      ParameterRegistry.getInstance().loadFromResources(QuadrupedParameterSet.SIMULATION_IDEAL.getPath());
      StateEstimatorParameters stateEstimatorParameters = new GenericQuadrupedStateEstimatorParameters();
      QuadrupedPositionBasedCrawlControllerParameters positionBasedCrawlControllerParameters = new GenericQuadrupedPositionBasedCrawlControllerParameters();

      FullQuadrupedRobotModel fullRobotModel = modelFactory.createFullRobotModel();
      FloatingRootJointRobot sdfRobot = new FloatingRootJointRobot(modelFactory.createSdfRobot());

      SensorTimestampHolder timestampProvider = new GenericQuadrupedTimestampProvider(sdfRobot);

      JointDesiredOutputList jointDesiredOutputList = new JointDesiredOutputList(fullRobotModel.getOneDoFJoints());
      QuadrupedReferenceFrames referenceFrames = new QuadrupedReferenceFrames(fullRobotModel, physicalProperties);
      OutputWriter outputWriter = new PerfectSimulatedOutputWriter(sdfRobot, fullRobotModel, jointDesiredOutputList);

      QuadrupedSimulationFactory simulationFactory = new QuadrupedSimulationFactory();
      simulationFactory.setControlDT(SIMULATION_DT);
      simulationFactory.setSimulationDT(SIMULATION_DT);
      simulationFactory.setGravity(SIMULATION_GRAVITY);
      simulationFactory.setRecordFrequency(RECORD_FREQUENCY);
      simulationFactory.setGroundContactParameters(groundContactParameters);
      simulationFactory.setModelFactory(modelFactory);
      simulationFactory.setSDFRobot(sdfRobot);
      simulationFactory.setSCSParameters(simulationTestingParameters);
      simulationFactory.setOutputWriter(outputWriter);
      simulationFactory.setShowPlotter(SHOW_PLOTTER);
      simulationFactory.setUseTrackAndDolly(USE_TRACK_AND_DOLLY);
      simulationFactory.setInitialPositionParameters(initialPositionParameters);
      simulationFactory.setFullRobotModel(fullRobotModel);
      simulationFactory.setPhysicalProperties(physicalProperties);
      simulationFactory.setUseNetworking(USE_NETWORKING);
      simulationFactory.setTimestampHolder(timestampProvider);
      simulationFactory.setUseStateEstimator(useStateEstimator.get());
      simulationFactory.setStateEstimatorParameters(stateEstimatorParameters);
      simulationFactory.setSensorInformation(sensorInformation);
      simulationFactory.setReferenceFrames(referenceFrames);
      simulationFactory.setJointDesiredOutputList(jointDesiredOutputList);
      simulationFactory.setNetClassList(netClassList);
      simulationFactory.setControlMode(controlMode.get());
      if (groundContactModelType.hasValue())
      {
         simulationFactory.setGroundContactModelType(groundContactModelType.get());
      }
      if (providedGroundProfile3D.hasValue())
      {
         simulationFactory.setGroundProfile3D(providedGroundProfile3D.get());
      }
      simulationFactory.setPositionBasedCrawlControllerParameters(positionBasedCrawlControllerParameters);
      simulationFactory.setUsePushRobotController(usePushRobotController.get());
      GoalOrientedTestConductor goalOrientedTestConductor = new GoalOrientedTestConductor(simulationFactory.createSimulation(), simulationTestingParameters);

      FactoryTools.disposeFactory(this);

      return goalOrientedTestConductor;
   }

   @Override
   public void setControlMode(QuadrupedControlMode controlMode)
   {
      this.controlMode.set(controlMode);
   }

   @Override
   public void setGroundContactModelType(QuadrupedGroundContactModelType groundContactModelType)
   {
      this.groundContactModelType.set(groundContactModelType);
   }

   @Override
   public void setUseStateEstimator(boolean useStateEstimator)
   {
      this.useStateEstimator.set(useStateEstimator);
   }

   @Override
   public void setGroundProfile3D(GroundProfile3D groundProfile3D)
   {
      providedGroundProfile3D.set(groundProfile3D);
   }

   @Override
   public void setUsePushRobotController(boolean usePushRobotController)
   {
      this.usePushRobotController.set(usePushRobotController);
   }

   @Override
   public void setParameterSet(QuadrupedParameterSet parameterSet)
   {
      this.parameterSet.set(parameterSet);
   }
}
