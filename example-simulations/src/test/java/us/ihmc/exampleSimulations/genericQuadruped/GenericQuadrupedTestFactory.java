package us.ihmc.exampleSimulations.genericQuadruped;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.exampleSimulations.genericQuadruped.model.GenericQuadrupedModelFactory;
import us.ihmc.exampleSimulations.genericQuadruped.model.GenericQuadrupedPhysicalProperties;
import us.ihmc.exampleSimulations.genericQuadruped.model.GenericQuadrupedSensorInformation;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedControllerCoreOptimizationSettings;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedDCMPlannerParameters;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedDefaultInitialPosition;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedFallDetectionParameters;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedHighLevelControllerParameters;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedPawPlannerParameters;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedPointFootSnapperParameters;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedPrivilegedConfigurationParameters;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedSitDownParameters;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedStateEstimatorParameters;
import us.ihmc.exampleSimulations.genericQuadruped.parameters.GenericQuadrupedXGaitSettings;
import us.ihmc.exampleSimulations.genericQuadruped.simulation.GenericQuadrupedGroundContactParameters;
import us.ihmc.graphicsDescription.conversion.YoGraphicConversionTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedNetworkModuleParameters;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedNetworkProcessor;
import us.ihmc.quadrupedCommunication.teleop.RemoteQuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.estimator.stateEstimator.QuadrupedSensorInformation;
import us.ihmc.quadrupedRobotics.model.QuadrupedInitialOffsetAndYaw;
import us.ihmc.quadrupedRobotics.model.QuadrupedInitialPositionParameters;
import us.ihmc.quadrupedRobotics.model.QuadrupedModelFactory;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.output.SimulatedQuadrupedOutputWriter;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedFallDetectionParameters;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedPrivilegedConfigurationParameters;
import us.ihmc.quadrupedRobotics.planning.trajectory.DCMPlannerParameters;
import us.ihmc.quadrupedRobotics.simulation.GroundContactParameters;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedSimulationFactory;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotModels.OutputWriter;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;

public class GenericQuadrupedTestFactory implements QuadrupedTestFactory
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final double CONTROL_DT = 0.001;
   private static final double SIMULATION_DT = 1e-4;
   private static final double SIMULATION_GRAVITY = -9.81;
   private static final int RECORD_FREQUENCY = (int) (0.01 / SIMULATION_DT);
   private static final boolean USE_STATE_ESTIMATOR = true;
   private static final boolean USE_TRACK_AND_DOLLY = false;

   private final OptionalFactoryField<Boolean> useStateEstimator = new OptionalFactoryField<>("useStateEstimator");
   private final OptionalFactoryField<QuadrupedGroundContactModelType> groundContactModelType = new OptionalFactoryField<>("groundContactModelType");
   private final OptionalFactoryField<TerrainObject3D> providedTerrainObject3D = new OptionalFactoryField<>("providedTerrainObject3D");
   private final OptionalFactoryField<Boolean> usePushRobotController = new OptionalFactoryField<>("usePushRobotController");
   private final OptionalFactoryField<QuadrupedInitialPositionParameters> initialPosition = new OptionalFactoryField<>("initialPosition");
   private final OptionalFactoryField<QuadrupedInitialOffsetAndYaw> initialOffset = new OptionalFactoryField<>("initialOffset");
   private final OptionalFactoryField<SimulationConstructionSetParameters> scsParameters = new OptionalFactoryField<>("scsParameters");

   private FullQuadrupedRobotModel fullRobotModel;
   private QuadrupedNetworkProcessor networkProcessor;
   private RemoteQuadrupedTeleopManager stepTeleopManager;
   private YoGraphicsListRegistry graphicsListRegistry;
   private String robotName;
   private QuadrupedSimulationFactory simulationFactory;

   public GenericQuadrupedTestFactory()
   {
      simulationTestingParameters.setKeepSCSUp(false);
   }

   @Override
   public GoalOrientedTestConductor createTestConductor()
   {
      useStateEstimator.setDefaultValue(USE_STATE_ESTIMATOR);
      initialPosition.setDefaultValue(new GenericQuadrupedDefaultInitialPosition());
      initialOffset.setDefaultValue(new QuadrupedInitialOffsetAndYaw());
      usePushRobotController.setDefaultValue(false);

      FactoryTools.checkAllFactoryFieldsAreSet(this);

      QuadrupedModelFactory modelFactory = new GenericQuadrupedModelFactory();
      QuadrupedPhysicalProperties physicalProperties = new GenericQuadrupedPhysicalProperties();
      QuadrupedInitialPositionParameters initialPositionParameters = initialPosition.get();
      GroundContactParameters groundContactParameters = new GenericQuadrupedGroundContactParameters();
      QuadrupedSensorInformation sensorInformation = new GenericQuadrupedSensorInformation();
      StateEstimatorParameters stateEstimatorParameters = new GenericQuadrupedStateEstimatorParameters(false, CONTROL_DT);
      GenericQuadrupedXGaitSettings xGaitSettings = new GenericQuadrupedXGaitSettings();
      GenericQuadrupedHighLevelControllerParameters highLevelControllerParameters = new GenericQuadrupedHighLevelControllerParameters(modelFactory.getJointMap());
      DCMPlannerParameters dcmPlannerParameters = new GenericQuadrupedDCMPlannerParameters();
      GenericQuadrupedSitDownParameters sitDownParameters = new GenericQuadrupedSitDownParameters();
      QuadrupedPrivilegedConfigurationParameters privilegedConfigurationParameters = new GenericQuadrupedPrivilegedConfigurationParameters();
      QuadrupedFallDetectionParameters fallDetectionParameters = new GenericQuadrupedFallDetectionParameters();

      fullRobotModel = modelFactory.createFullRobotModel();
      RobotDefinition robotDefinition = modelFactory.getRobotDefinition();
      initialPositionParameters.offsetInitialConfiguration(initialOffset.get());
      QuadrupedSimulationFactory.setRobotDefinitionInitialJointStates(initialPositionParameters, modelFactory.getQuadrupedJointNames(), modelFactory::getSDFNameForJointName, robotDefinition);
      Robot sdfRobot = new Robot(robotDefinition, SimulationConstructionSet2.inertialFrame);
      ControllerCoreOptimizationSettings controllerCoreOptimizationSettings = new GenericQuadrupedControllerCoreOptimizationSettings(fullRobotModel.getTotalMass());
      robotName = sdfRobot.getName();


      JointDesiredOutputList jointDesiredOutputList = new JointDesiredOutputList(fullRobotModel.getOneDoFJoints());
      QuadrupedReferenceFrames referenceFrames = new QuadrupedReferenceFrames(fullRobotModel);
      OutputWriter outputWriter = new SimulatedQuadrupedOutputWriter(sdfRobot, fullRobotModel, jointDesiredOutputList, CONTROL_DT);

      QuadrantDependentList<Double> kneeTorqueTouchdownDetectionThreshold = new QuadrantDependentList<>(20.0, 20.0, -20.0, -20.0);
      QuadrantDependentList<Double> kneeTorqueTouchdownForSureDetectionThreshold = new QuadrantDependentList<>(75.0, 75.0, -75.0, -75.0);

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
      simulationFactory.setUseTrackAndDolly(USE_TRACK_AND_DOLLY);
      simulationFactory.setFullRobotModel(fullRobotModel);
      simulationFactory.setKneeTorqueTouchdownDetectionThreshold(kneeTorqueTouchdownDetectionThreshold);
      simulationFactory.setKneeTorqueTouchdownForSureDetectionThreshold(kneeTorqueTouchdownForSureDetectionThreshold);
      simulationFactory.setControllerCoreOptimizationSettings(controllerCoreOptimizationSettings);
      simulationFactory.setPhysicalProperties(physicalProperties);
      simulationFactory.setUseStateEstimator(useStateEstimator.get());
      simulationFactory.setStateEstimatorParameters(stateEstimatorParameters);
      simulationFactory.setSensorInformation(sensorInformation);
      simulationFactory.setReferenceFrames(referenceFrames);
      simulationFactory.setJointDesiredOutputList(jointDesiredOutputList);
      simulationFactory.setInitialForceControlState(HighLevelControllerName.DO_NOTHING_BEHAVIOR);
      simulationFactory.setPubSubImplementation(PubSubImplementation.INTRAPROCESS);
      simulationFactory.setHighLevelControllerParameters(highLevelControllerParameters);
      simulationFactory.setDCMPlannerParameters(dcmPlannerParameters);
      simulationFactory.setSitDownParameters(sitDownParameters);
      simulationFactory.setPrivilegedConfigurationParameters(privilegedConfigurationParameters);
      simulationFactory.setFallDetectionParameters(fallDetectionParameters);

      if (groundContactModelType.hasValue())
      {
         simulationFactory.setGroundContactModelType(groundContactModelType.get());
      }
      if (providedTerrainObject3D.hasValue())
      {
         simulationFactory.setTerrainObject3D(providedTerrainObject3D.get());
      }

      YoRegistry teleopRegistry = new YoRegistry("TeleopRegistry");
      sdfRobot.getRegistry().addChild(teleopRegistry);

      ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.INTRAPROCESS, "quadruped_teleop_manager");

      QuadrupedNetworkModuleParameters networkModuleParameters = new QuadrupedNetworkModuleParameters();

      // enable teleop modules
      networkModuleParameters.enableFootstepPlanningModule(true);
      networkModuleParameters.enableStepTeleopModule(true);

      graphicsListRegistry = new YoGraphicsListRegistry();
      networkProcessor = new GenericQuadrupedNetworkProcessor(modelFactory,
                                                              physicalProperties.getFeetGroundContactPoints(),
                                                              new DefaultVisibilityGraphParameters(),
                                                              new GenericQuadrupedPawPlannerParameters(),
                                                              xGaitSettings,
                                                              new GenericQuadrupedPointFootSnapperParameters(),
                                                              PubSubImplementation.INTRAPROCESS,
                                                              networkModuleParameters);
      stepTeleopManager = new RemoteQuadrupedTeleopManager(robotName,
                                                           ros2Node,
                                                           networkProcessor,
                                                           modelFactory.createFullRobotModel(),
                                                           xGaitSettings,
                                                           teleopRegistry);
      networkProcessor.setRootRegistry(teleopRegistry, graphicsListRegistry);

      new DefaultParameterReader().readParametersInRegistry(teleopRegistry);

      simulationFactory.setUsePushRobotController(usePushRobotController.get());
      GoalOrientedTestConductor goalOrientedTestConductor = new GoalOrientedTestConductor(simulationFactory.createSimulation(), simulationTestingParameters);
      goalOrientedTestConductor.getScs().addYoGraphics(YoGraphicConversionTools.toYoGraphicDefinitions(graphicsListRegistry));

      FactoryTools.disposeFactory(this);

      ros2Node.destroy();

      return goalOrientedTestConductor;
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
   public RemoteQuadrupedTeleopManager getRemoteStepTeleopManager()
   {
      return stepTeleopManager;
   }

   @Override
   public String getRobotName()
   {
      return robotName;
   }

   @Override
   public FullQuadrupedRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   @Override
   public void close()
   {
      networkProcessor.close();
      simulationFactory.close();
   }
}
