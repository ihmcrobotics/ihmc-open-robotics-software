package us.ihmc.exampleSimulations.genericQuadruped;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.exampleSimulations.genericQuadruped.model.GenericQuadrupedModelFactory;
import us.ihmc.exampleSimulations.genericQuadruped.model.GenericQuadrupedPhysicalProperties;
import us.ihmc.exampleSimulations.genericQuadruped.model.GenericQuadrupedSensorInformation;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedControllerCoreOptimizationSettings;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedDefaultInitialPosition;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedStateEstimatorParameters;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedXGaitSettings;
import us.ihmc.exampleSimulations.genericQuadruped.simulation.GenericQuadrupedGroundContactParameters;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.communication.QuadrupedNetClassList;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerEnum;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.estimator.stateEstimator.QuadrupedSensorInformation;
import us.ihmc.quadrupedRobotics.input.QuadrupedTestTeleopScript;
import us.ihmc.quadrupedRobotics.input.managers.QuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.model.QuadrupedInitialOffsetAndYaw;
import us.ihmc.quadrupedRobotics.model.QuadrupedInitialPositionParameters;
import us.ihmc.quadrupedRobotics.model.QuadrupedModelFactory;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.output.SimulatedQuadrupedOutputWriter;
import us.ihmc.quadrupedRobotics.simulation.GroundContactParameters;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedSimulationFactory;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.OutputWriter;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.sensorProcessors.SensorTimestampHolder;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.factories.RequiredFactoryField;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.io.IOException;

public class GenericQuadrupedTestFactory implements QuadrupedTestFactory
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final double CONTROL_DT = 0.001;
   private static final double SIMULATION_DT = 1e-4;
   private static final double SIMULATION_GRAVITY = -9.81;
   private static final int RECORD_FREQUENCY = (int) (0.01 / SIMULATION_DT);
   private static final boolean USE_STATE_ESTIMATOR = true;
   private static final boolean SHOW_PLOTTER = true;
   private static final boolean USE_TRACK_AND_DOLLY = false;
   private static final int TEST_INPUT_UPDATE_FREQUENCY = (int) (0.05 / SIMULATION_DT);

   private final RequiredFactoryField<WholeBodyControllerCoreMode> controlMode = new RequiredFactoryField<>("controlMode");

   private final OptionalFactoryField<Boolean> useStateEstimator = new OptionalFactoryField<>("useStateEstimator");
   private final OptionalFactoryField<QuadrupedGroundContactModelType> groundContactModelType = new OptionalFactoryField<>("groundContactModelType");
   private final OptionalFactoryField<GroundProfile3D> providedGroundProfile3D = new OptionalFactoryField<>("providedGroundProfile3D");
   private final OptionalFactoryField<TerrainObject3D> providedTerrainObject3D = new OptionalFactoryField<>("providedTerrainObject3D");
   private final OptionalFactoryField<Boolean> usePushRobotController = new OptionalFactoryField<>("usePushRobotController");
   private final OptionalFactoryField<QuadrupedInitialPositionParameters> initialPosition = new OptionalFactoryField<>("initialPosition");
   private final OptionalFactoryField<QuadrupedInitialOffsetAndYaw> initialOffset = new OptionalFactoryField<>("initialOffset");
   private final OptionalFactoryField<Boolean> useNetworking = new OptionalFactoryField<>("useNetworking");
   private final OptionalFactoryField<SimulationConstructionSetParameters> scsParameters = new OptionalFactoryField<>("scsParameters");

   private FullQuadrupedRobotModel fullRobotModel;
   private QuadrupedTeleopManager stepTeleopManager;
   private YoGraphicsListRegistry graphicsListRegistry;
   private String robotName;
   private QuadrupedSimulationFactory simulationFactory;

   public GenericQuadrupedTestFactory()
   {
      simulationTestingParameters.setKeepSCSUp(!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer());
   }

   @Override
   public GoalOrientedTestConductor createTestConductor() throws IOException
   {
      useStateEstimator.setDefaultValue(USE_STATE_ESTIMATOR);
      initialPosition.setDefaultValue(new GenericQuadrupedDefaultInitialPosition());
      initialOffset.setDefaultValue(new QuadrupedInitialOffsetAndYaw());
      usePushRobotController.setDefaultValue(false);
      useNetworking.setDefaultValue(false);

      FactoryTools.checkAllFactoryFieldsAreSet(this);

      QuadrupedModelFactory modelFactory = new GenericQuadrupedModelFactory();
      QuadrupedPhysicalProperties physicalProperties = new GenericQuadrupedPhysicalProperties();
      NetClassList netClassList = new QuadrupedNetClassList();
      QuadrupedInitialPositionParameters initialPositionParameters = initialPosition.get();
      GroundContactParameters groundContactParameters = new GenericQuadrupedGroundContactParameters();
      QuadrupedSensorInformation sensorInformation = new GenericQuadrupedSensorInformation();
      StateEstimatorParameters stateEstimatorParameters = new GenericQuadrupedStateEstimatorParameters(false, CONTROL_DT);
      GenericQuadrupedXGaitSettings xGaitSettings = new GenericQuadrupedXGaitSettings();

      fullRobotModel = modelFactory.createFullRobotModel();
      FloatingRootJointRobot sdfRobot = new FloatingRootJointRobot(modelFactory.createSdfRobot());
      ControllerCoreOptimizationSettings controllerCoreOptimizationSettings = new GenericQuadrupedControllerCoreOptimizationSettings(
            fullRobotModel.getTotalMass());
      robotName = sdfRobot.getName();

      SensorTimestampHolder timestampProvider = new GenericQuadrupedTimestampProvider(sdfRobot);

      JointDesiredOutputList jointDesiredOutputList = new JointDesiredOutputList(fullRobotModel.getOneDoFJoints());
      QuadrupedReferenceFrames referenceFrames = new QuadrupedReferenceFrames(fullRobotModel, physicalProperties);
      OutputWriter outputWriter = new SimulatedQuadrupedOutputWriter(sdfRobot, fullRobotModel, jointDesiredOutputList, CONTROL_DT);

      QuadrantDependentList<Boolean> kneeOrientationsOutward = new QuadrantDependentList<>(false, false, false, false);

      simulationFactory = new QuadrupedSimulationFactory();
      simulationFactory.setControlDT(CONTROL_DT);
      simulationFactory.setSimulationDT(SIMULATION_DT);
      simulationFactory.setGravity(SIMULATION_GRAVITY);
      simulationFactory.setRecordFrequency(RECORD_FREQUENCY);
      simulationFactory.setGroundContactParameters(groundContactParameters);
      simulationFactory.setModelFactory(modelFactory);
      simulationFactory.setSDFRobot(sdfRobot);
      if (scsParameters.hasValue())
         simulationFactory.setSCSParameters(scsParameters.get());
      else
         simulationFactory.setSCSParameters(simulationTestingParameters);
      simulationFactory.setOutputWriter(outputWriter);
      simulationFactory.setShowPlotter(SHOW_PLOTTER);
      simulationFactory.setUseTrackAndDolly(USE_TRACK_AND_DOLLY);
      simulationFactory.setInitialPositionParameters(initialPositionParameters);
      simulationFactory.setInitialOffset(initialOffset.get());
      simulationFactory.setFullRobotModel(fullRobotModel);
      simulationFactory.setKneeOrientationsOutward(kneeOrientationsOutward);
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
      simulationFactory.setInitialForceControlState(QuadrupedControllerEnum.DO_NOTHING);
      simulationFactory.setUseLocalCommunicator(useNetworking.get());

      if (groundContactModelType.hasValue())
      {
         simulationFactory.setGroundContactModelType(groundContactModelType.get());
      }
      if (providedTerrainObject3D.hasValue())
      {
         if (providedGroundProfile3D.hasValue())
            throw new RuntimeException("You can only have one of these!");

         simulationFactory.setTerrainObject3D(providedTerrainObject3D.get());
      }
      if (providedGroundProfile3D.hasValue())
      {
         simulationFactory.setGroundProfile3D(providedGroundProfile3D.get());
      }

      if(useNetworking.get())
      {
         YoVariableRegistry teleopRegistry = new YoVariableRegistry("TeleopRegistry");
         sdfRobot.getRobotsYoVariableRegistry().addChild(teleopRegistry);

         Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.INTRAPROCESS, "quadruped_teleop_manager");

         graphicsListRegistry = new YoGraphicsListRegistry();
         stepTeleopManager = new QuadrupedTeleopManager(robotName, ros2Node, xGaitSettings, physicalProperties.getNominalCoMHeight(), referenceFrames, graphicsListRegistry, teleopRegistry);

         new DefaultParameterReader().readParametersInRegistry(teleopRegistry);
      }
      else
      {
         stepTeleopManager = null;
      }

      simulationFactory.setUsePushRobotController(usePushRobotController.get());
      GoalOrientedTestConductor goalOrientedTestConductor = new GoalOrientedTestConductor(simulationFactory.createSimulation(), simulationTestingParameters);

      if(useNetworking.get())
      {
         goalOrientedTestConductor.getScs().addScript(new QuadrupedTestTeleopScript(stepTeleopManager, TEST_INPUT_UPDATE_FREQUENCY, sdfRobot.getRobotsYoVariableRegistry()));
         goalOrientedTestConductor.getScs().addYoGraphicsListRegistry(graphicsListRegistry);
      }

      FactoryTools.disposeFactory(this);

      return goalOrientedTestConductor;
   }

   @Override
   public void setControlMode(WholeBodyControllerCoreMode controlMode)
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
   public void setTerrainObject3D(TerrainObject3D terrainObject3D)
   {
      providedTerrainObject3D.set(terrainObject3D);
   }

   @Override
   public void setUsePushRobotController(boolean usePushRobotController)
   {
      this.usePushRobotController.set(usePushRobotController);
   }

   @Override
   public void setInitialPosition(QuadrupedInitialPositionParameters initialPosition)
   {
      this.initialPosition.set(initialPosition);
   }

   @Override
   public void setUseNetworking(boolean useNetworking)
   {
      this.useNetworking.set(useNetworking);
   }

   @Override
   public void setScsParameters(SimulationConstructionSetParameters scsParameters)
   {
      this.scsParameters.set(scsParameters);
   }

   @Override
   public void setInitialOffset(QuadrupedInitialOffsetAndYaw initialOffset)
   {
      this.initialOffset.set(initialOffset);
   }

   @Override
   public QuadrupedTeleopManager getStepTeleopManager()
   {
      return stepTeleopManager;
   }

   @Override
   public String getRobotName()
   {
      return robotName;
   }

   @Override
   public FullRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   @Override
   public void close()
   {
      simulationFactory.close();
   }
}
