package us.ihmc.exampleSimulations.genericQuadruped;

import java.io.IOException;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.exampleSimulations.genericQuadruped.model.GenericQuadrupedModelFactory;
import us.ihmc.exampleSimulations.genericQuadruped.model.GenericQuadrupedPhysicalProperties;
import us.ihmc.exampleSimulations.genericQuadruped.model.GenericQuadrupedSensorInformation;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedControllerCoreOptimizationSettings;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedDefaultInitialPosition;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedPositionBasedCrawlControllerParameters;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedStateEstimatorParameters;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedXGaitSettings;
import us.ihmc.exampleSimulations.genericQuadruped.simulation.GenericQuadrupedGroundContactParameters;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.communication.QuadrupedNetClassList;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.controller.QuadrupedSimulationFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerEnum;
import us.ihmc.quadrupedRobotics.controller.position.states.QuadrupedPositionBasedCrawlControllerParameters;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.estimator.stateEstimator.QuadrupedSensorInformation;
import us.ihmc.quadrupedRobotics.input.QuadrupedTestTeleopScript;
import us.ihmc.quadrupedRobotics.input.managers.QuadrupedBodyPoseTeleopManager;
import us.ihmc.quadrupedRobotics.input.managers.QuadrupedStepTeleopManager;
import us.ihmc.quadrupedRobotics.model.QuadrupedModelFactory;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.model.QuadrupedSimulationInitialPositionParameters;
import us.ihmc.quadrupedRobotics.simulation.GroundContactParameters;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotModels.OutputWriter;
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
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class GenericQuadrupedTestFactory implements QuadrupedTestFactory
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final double CONTROL_DT = 0.001;
   private static final double SIMULATION_DT = 1e-4;
   private static final double SIMULATION_GRAVITY = -9.81;
   private static final int RECORD_FREQUENCY = (int) (0.01 / SIMULATION_DT);
   private static final boolean USE_STATE_ESTIMATOR = false;
   private static final boolean SHOW_PLOTTER = true;
   private static final boolean USE_TRACK_AND_DOLLY = true;
   private static final int TEST_INPUT_UPDATE_FREQUENCY = (int) (0.05 / SIMULATION_DT);

   private final RequiredFactoryField<QuadrupedControlMode> controlMode = new RequiredFactoryField<>("controlMode");

   private final OptionalFactoryField<Boolean> useStateEstimator = new OptionalFactoryField<>("useStateEstimator");
   private final OptionalFactoryField<QuadrupedGroundContactModelType> groundContactModelType = new OptionalFactoryField<>("groundContactModelType");
   private final OptionalFactoryField<GroundProfile3D> providedGroundProfile3D = new OptionalFactoryField<>("providedGroundProfile3D");
   private final OptionalFactoryField<Boolean> usePushRobotController = new OptionalFactoryField<>("usePushRobotController");
   private final OptionalFactoryField<QuadrupedSimulationInitialPositionParameters> initialPosition = new OptionalFactoryField<>("initialPosition");
   private final OptionalFactoryField<Boolean> useNetworking = new OptionalFactoryField<>("useNetworking");

   private QuadrupedStepTeleopManager stepTeleopManager;
   private QuadrupedBodyPoseTeleopManager bodyPoseTeleopManager;

   @Override
   public GoalOrientedTestConductor createTestConductor() throws IOException
   {
      useStateEstimator.setDefaultValue(USE_STATE_ESTIMATOR);
      initialPosition.setDefaultValue(new GenericQuadrupedDefaultInitialPosition());
      usePushRobotController.setDefaultValue(false);
      useNetworking.setDefaultValue(false);

      FactoryTools.checkAllFactoryFieldsAreSet(this);

      QuadrupedModelFactory modelFactory = new GenericQuadrupedModelFactory();
      QuadrupedPhysicalProperties physicalProperties = new GenericQuadrupedPhysicalProperties();
      NetClassList netClassList = new QuadrupedNetClassList();
      QuadrupedSimulationInitialPositionParameters initialPositionParameters = initialPosition.get();
      GroundContactParameters groundContactParameters = new GenericQuadrupedGroundContactParameters();
      QuadrupedSensorInformation sensorInformation = new GenericQuadrupedSensorInformation();
      StateEstimatorParameters stateEstimatorParameters = new GenericQuadrupedStateEstimatorParameters();
      QuadrupedPositionBasedCrawlControllerParameters positionBasedCrawlControllerParameters = new GenericQuadrupedPositionBasedCrawlControllerParameters();
      GenericQuadrupedXGaitSettings xGaitSettings = new GenericQuadrupedXGaitSettings();

      FullQuadrupedRobotModel fullRobotModel = modelFactory.createFullRobotModel();
      FloatingRootJointRobot sdfRobot = new FloatingRootJointRobot(modelFactory.createSdfRobot());
      ControllerCoreOptimizationSettings controllerCoreOptimizationSettings = new GenericQuadrupedControllerCoreOptimizationSettings(
            fullRobotModel.getTotalMass());

      SensorTimestampHolder timestampProvider = new GenericQuadrupedTimestampProvider(sdfRobot);

      JointDesiredOutputList jointDesiredOutputList = new JointDesiredOutputList(fullRobotModel.getOneDoFJoints());
      QuadrupedReferenceFrames referenceFrames = new QuadrupedReferenceFrames(fullRobotModel, physicalProperties);
      OutputWriter outputWriter = new PerfectSimulatedOutputWriter(sdfRobot, fullRobotModel, jointDesiredOutputList);

      QuadrupedSimulationFactory simulationFactory = new QuadrupedSimulationFactory();
      simulationFactory.setControlDT(CONTROL_DT);
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
      simulationFactory.setControllerCoreOptimizationSettings(controllerCoreOptimizationSettings);
      simulationFactory.setPhysicalProperties(physicalProperties);
      simulationFactory.setUseNetworking(useNetworking.get());
      simulationFactory.setTimestampHolder(timestampProvider);
      simulationFactory.setUseStateEstimator(useStateEstimator.get());
      simulationFactory.setStateEstimatorParameters(stateEstimatorParameters);
      simulationFactory.setSensorInformation(sensorInformation);
      simulationFactory.setReferenceFrames(referenceFrames);
      simulationFactory.setJointDesiredOutputList(jointDesiredOutputList);
      simulationFactory.setNetClassList(netClassList);
      simulationFactory.setControlMode(controlMode.get());
      simulationFactory.setXGaitSettings(xGaitSettings);
      simulationFactory.setInitialForceControlState(QuadrupedForceControllerEnum.FREEZE);
      simulationFactory.setUseLocalCommunicator(useNetworking.get());

      if (groundContactModelType.hasValue())
      {
         simulationFactory.setGroundContactModelType(groundContactModelType.get());
      }
      if (providedGroundProfile3D.hasValue())
      {
         simulationFactory.setGroundProfile3D(providedGroundProfile3D.get());
      }

      if(useNetworking.get())
      {
         YoVariableRegistry teleopRegistry = new YoVariableRegistry("TeleopRegistry");
         sdfRobot.getRobotsYoVariableRegistry().addChild(teleopRegistry);

         PacketCommunicator packetCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_PORT, netClassList);
         packetCommunicator.connect();
         stepTeleopManager = new QuadrupedStepTeleopManager(packetCommunicator, xGaitSettings, referenceFrames, teleopRegistry);
         bodyPoseTeleopManager = new QuadrupedBodyPoseTeleopManager(physicalProperties.getNominalCoMHeight(), packetCommunicator);

         new DefaultParameterReader().readParametersInRegistry(teleopRegistry);
      }
      else
      {
         stepTeleopManager = null;
      }

      simulationFactory.setPositionBasedCrawlControllerParameters(positionBasedCrawlControllerParameters);
      simulationFactory.setUsePushRobotController(usePushRobotController.get());
      GoalOrientedTestConductor goalOrientedTestConductor = new GoalOrientedTestConductor(simulationFactory.createSimulation(), simulationTestingParameters);

      if(useNetworking.get())
      {
         goalOrientedTestConductor.getScs().addScript(new QuadrupedTestTeleopScript(stepTeleopManager, bodyPoseTeleopManager, TEST_INPUT_UPDATE_FREQUENCY));
      }

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
   public void setInitialPosition(QuadrupedSimulationInitialPositionParameters initialPosition)
   {
      this.initialPosition.set(initialPosition);
   }

   @Override
   public void setUseNetworking(boolean useNetworking)
   {
      this.useNetworking.set(useNetworking);
   }

   @Override
   public QuadrupedStepTeleopManager getStepTeleopManager()
   {
      return stepTeleopManager;
   }

   @Override
   public QuadrupedBodyPoseTeleopManager getBodyPoseTeleopManager()
   {
      return bodyPoseTeleopManager;
   }
}
