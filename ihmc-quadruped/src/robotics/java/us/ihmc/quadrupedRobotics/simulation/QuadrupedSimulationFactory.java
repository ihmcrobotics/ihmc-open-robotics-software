package us.ihmc.quadrupedRobotics.simulation;

import java.io.InputStream;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerManager;
import us.ihmc.quadrupedRobotics.controller.QuadrupedSimulationController;
import us.ihmc.quadrupedRobotics.estimator.footSwitch.QuadrupedFootSwitchFactory;
import us.ihmc.quadrupedRobotics.estimator.stateEstimator.QuadrupedSensorInformation;
import us.ihmc.quadrupedRobotics.estimator.stateEstimator.QuadrupedSensorReaderWrapper;
import us.ihmc.quadrupedRobotics.estimator.stateEstimator.QuadrupedStateEstimatorFactory;
import us.ihmc.quadrupedRobotics.model.QuadrupedInitialPositionParameters;
import us.ihmc.quadrupedRobotics.model.QuadrupedModelFactory;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedFallDetectionParameters;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedPrivilegedConfigurationParameters;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedSitDownParameters;
import us.ihmc.quadrupedRobotics.planning.trajectory.DCMPlannerInterface;
import us.ihmc.quadrupedRobotics.planning.trajectory.DCMPlannerParameters;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotModels.OutputWriter;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.robotics.physics.CollidableHelper;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.robotics.stateMachine.core.StateChangedListener;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.session.tools.SCS1GraphicConversionTools;
import us.ihmc.scs2.simulation.parameters.ContactPointBasedContactParameters;
import us.ihmc.scs2.simulation.physicsEngine.contactPointBased.ContactPointBasedPhysicsEngine;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.controller.SimControllerInput;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataPublisher;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataPublisherFactory;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.simulatedSensors.SCS2SensorReaderFactory;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationToolkit.TerrainObjectDefinitionTools;
import us.ihmc.simulationToolkit.controllers.PushRobotControllerSCS2;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.stateEstimation.humanoid.StateEstimatorController;
import us.ihmc.systemIdentification.frictionId.simulators.CoulombViscousStribeckFrictionParameters;
import us.ihmc.systemIdentification.frictionId.simulators.SimulatedFrictionController;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.factories.RequiredFactoryField;
import us.ihmc.wholeBodyController.parameters.ParameterLoaderHelper;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedSimulationFactory
{
   private final RequiredFactoryField<FullQuadrupedRobotModel> fullRobotModel = new RequiredFactoryField<>("fullRobotModel");
   private final RequiredFactoryField<ControllerCoreOptimizationSettings> controllerCoreOptimizationSettings = new RequiredFactoryField<>("controllerCoreOptimizationSettings");
   private final RequiredFactoryField<QuadrupedPhysicalProperties> physicalProperties = new RequiredFactoryField<>("physicalProperties");
   private final RequiredFactoryField<Robot> sdfRobot = new RequiredFactoryField<>("sdfRobot");
   private final RequiredFactoryField<Double> simulationDT = new RequiredFactoryField<>("simulationDT");
   private final RequiredFactoryField<Double> controlDT = new RequiredFactoryField<>("controlDT");
   private final RequiredFactoryField<Double> gravity = new RequiredFactoryField<>("gravity");
   private final RequiredFactoryField<Integer> recordFrequency = new RequiredFactoryField<>("recordFrequency");
   private final RequiredFactoryField<Boolean> useTrackAndDolly = new RequiredFactoryField<>("useTrackAndDolly");
   private final RequiredFactoryField<QuadrupedModelFactory> modelFactory = new RequiredFactoryField<>("modelFactory");
   private final RequiredFactoryField<SimulationConstructionSetParameters> scsParameters = new RequiredFactoryField<>("scsParameters");
   private final RequiredFactoryField<GroundContactParameters> groundContactParameters = new RequiredFactoryField<>("groundContactParameters");
   private final RequiredFactoryField<OutputWriter> outputWriter = new RequiredFactoryField<>("outputWriter");
   private final RequiredFactoryField<Boolean> useStateEstimator = new RequiredFactoryField<>("useStateEstimator");
   private final RequiredFactoryField<QuadrupedSensorInformation> sensorInformation = new RequiredFactoryField<>("sensorInformation");
   private final RequiredFactoryField<StateEstimatorParameters> stateEstimatorParameters = new RequiredFactoryField<>("stateEstimatorParameters");
   private final RequiredFactoryField<QuadrupedReferenceFrames> referenceFrames = new RequiredFactoryField<>("referenceFrames");
   private final RequiredFactoryField<JointDesiredOutputList> jointDesiredOutputList = new RequiredFactoryField<>("jointDesiredOutputList");
   private final RequiredFactoryField<HighLevelControllerParameters> highLevelControllerParameters = new RequiredFactoryField<>("highLevelControllerParameters");
   private final RequiredFactoryField<DCMPlannerParameters> dcmPlannerParameters = new RequiredFactoryField<>("dcmPlannerParameters");
   private final RequiredFactoryField<QuadrupedSitDownParameters> sitDownParameters = new RequiredFactoryField<>("sitDownParameters");
   private final RequiredFactoryField<QuadrupedPrivilegedConfigurationParameters> privilegedConfigurationParameters = new RequiredFactoryField<>("privilegedConfigurationParameters");
   private final RequiredFactoryField<QuadrupedFallDetectionParameters> fallDetectionParameters = new RequiredFactoryField<>("fallDetectionParameters");
   private final RequiredFactoryField<PubSubImplementation> pubSubImplementation = new RequiredFactoryField<>("pubSubImplementation");

   private final OptionalFactoryField<CoulombViscousStribeckFrictionParameters> simulatedFrictionParameters = new OptionalFactoryField<>("jointFrictionParameters");
   private final OptionalFactoryField<QuadrupedGroundContactModelType> groundContactModelType = new OptionalFactoryField<>("groundContactModelType");
   protected final ArrayList<TerrainObjectDefinition> terrainObjectDefinitions = new ArrayList<>();
   private final OptionalFactoryField<Boolean> usePushRobotController = new OptionalFactoryField<>("usePushRobotController");
   private final OptionalFactoryField<FootSwitchType> footSwitchType = new OptionalFactoryField<>("footSwitchType");
   private final OptionalFactoryField<QuadrantDependentList<Double>> kneeTorqueTouchdownDetectionThreshold = new OptionalFactoryField<>("kneeTorqueTouchdownDetectionThreshold");
   private final OptionalFactoryField<QuadrantDependentList<Double>> kneeTorqueTouchdownForSureDetectionThreshold = new OptionalFactoryField<>("kneeTorqueTouchdownForSureDetectionThreshold");
   private final OptionalFactoryField<Integer> scsBufferSize = new OptionalFactoryField<>("scsBufferSize");
   private final OptionalFactoryField<HighLevelControllerName> initialForceControlState = new OptionalFactoryField<>("initialForceControlState");
   private final OptionalFactoryField<Boolean> createYoVariableServer = new OptionalFactoryField<>("createYoVariableServer");
   private final OptionalFactoryField<DCMPlannerInterface> customCoMTrajectoryPlanner = new OptionalFactoryField<>("customCoMTrajectoryPlanner");

   // TO CONSTRUCT
   private SimulationConstructionSet2 simulationConstructionSet;
   private YoRegistry factoryRegistry;
   private YoGraphicsListRegistry yoGraphicsListRegistry;
   private QuadrupedSensorReaderWrapper sensorReaderWrapper;
   private SensorReader sensorReader;
   private RobotMotionStatusHolder robotMotionStatusFromController;
   private QuadrantDependentList<ContactablePlaneBody> contactableFeet;
   private List<ContactablePlaneBody> contactablePlaneBodies;
   private QuadrantDependentList<FootSwitchInterface> controllerFootSwitches;
   private QuadrantDependentList<FootSwitchInterface> stateEstimatorFootSwitches;
   private StateEstimatorController stateEstimator;
   private CenterOfMassDataHolder centerOfMassDataHolder = null;
   private RealtimeROS2Node realtimeROS2Node;
   private QuadrupedControllerManager controllerManager;
   private RobotConfigurationDataPublisher robotConfigurationDataPublisher;
   private QuadrupedSimulationController simulationController;
   private YoVariableServer yoVariableServer;

   protected final CollidableHelper collidableHelper = new CollidableHelper();
   protected final String robotCollisionName = "robot";
   protected final String terrainCollisionName = "terrain";

   // CREATION

   private void setupYoRegistries()
   {
      factoryRegistry = new YoRegistry("factoryRegistry");
      yoGraphicsListRegistry = new YoGraphicsListRegistry();
      yoGraphicsListRegistry.setYoGraphicsUpdatedRemotely(true);
   }

   private void createPushRobotController()
   {
      if (usePushRobotController.get())
      {
         Robot pushableRobot = sdfRobot.get();
         String rootJointName = pushableRobot.getFloatingRootJoint().getName();

         YoDouble time = simulationConstructionSet.getTime();
         PushRobotControllerSCS2 bodyPushRobotController = new PushRobotControllerSCS2(time,
                                                                                       pushableRobot,
                                                                                       rootJointName,
                                                                                       new Vector3D(0.0, -0.00057633, 0.0383928),
                                                                                       0.01);
         simulationConstructionSet.addYoGraphic("PushRobotControllers", bodyPushRobotController.getForceVizDefinition());

         for (QuadrupedJointName quadrupedJointName : modelFactory.get().getQuadrupedJointNames())
         {
            String jointName = modelFactory.get().getSDFNameForJointName(quadrupedJointName);
            PushRobotControllerSCS2 jointPushRobotController = new PushRobotControllerSCS2(time, pushableRobot, jointName, new Vector3D(0.0, 0.0, 0.0), 0.01);
            simulationConstructionSet.addYoGraphic("PushRobotControllers", jointPushRobotController.getForceVizDefinition());
         }
      }
   }

   private void createSensorReader()
   {
      SensorReader sensorReader;
      FloatingJointBasics rootInverseDynamicsJoint = fullRobotModel.get().getRootJoint();
      IMUDefinition[] imuDefinitions = fullRobotModel.get().getIMUDefinitions();
      ForceSensorDefinition[] forceSensorDefinitions = fullRobotModel.get().getForceSensorDefinitions();
      JointDesiredOutputList estimatorDesiredJointDataHolder = null;

      SCS2SensorReaderFactory sensorReaderFactory;
      SimControllerInput controllerInput = sdfRobot.get().getControllerManager().getControllerInput();
      if (useStateEstimator.get())
         sensorReaderFactory = SCS2SensorReaderFactory.newSensorReaderFactory(controllerInput, stateEstimatorParameters.get());
      else
         sensorReaderFactory = SCS2SensorReaderFactory.newPerfectSensorReaderFactory(controllerInput);

      sensorReaderFactory.build(rootInverseDynamicsJoint, imuDefinitions, forceSensorDefinitions, estimatorDesiredJointDataHolder, factoryRegistry);

      sensorReader = sensorReaderFactory.getSensorReader();

      if (this.sensorReaderWrapper != null)
      {
         this.sensorReaderWrapper.setSensorReader(sensorReader);
         this.sensorReader = sensorReaderWrapper;
      }
      else
      {
         this.sensorReader = sensorReader;
      }
   }

   private void createContactableFeet()
   {
      ContactableBodiesFactory<RobotQuadrant> footContactableBodiesFactory = new ContactableBodiesFactory<>();
      footContactableBodiesFactory.setFullRobotModel(fullRobotModel.get());
      footContactableBodiesFactory.setFootContactPoints(physicalProperties.get().getFeetGroundContactPoints());
      footContactableBodiesFactory.setReferenceFrames(referenceFrames.get());
      contactableFeet = new QuadrantDependentList<>(footContactableBodiesFactory.createFootContactablePlaneBodies());
      footContactableBodiesFactory.disposeFactory();
   }

   private void createContactablePlaneBodies()
   {
      contactablePlaneBodies = new ArrayList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         contactablePlaneBodies.add(contactableFeet.get(robotQuadrant));
   }

   private void createFootSwitches()
   {
      QuadrupedFootSwitchFactory footSwitchFactory = new QuadrupedFootSwitchFactory();
      footSwitchFactory.setFootContactableBodies(contactableFeet);
      footSwitchFactory.setFullRobotModel(fullRobotModel.get());
      footSwitchFactory.setJointDesiredOutputList(jointDesiredOutputList.get());
      footSwitchFactory.setGravity(gravity.get());
      footSwitchFactory.setSimulatedRobot(sdfRobot.get());
      footSwitchFactory.setYoVariableRegistry(factoryRegistry);
      footSwitchFactory.setFootSwitchType(footSwitchType.get());
      footSwitchFactory.setVariableSuffix("Controller");

      controllerFootSwitches = footSwitchFactory.createFootSwitches();
   }

   private void createStateEstimatorFootSwitches()
   {
      QuadrupedFootSwitchFactory footSwitchFactory = new QuadrupedFootSwitchFactory();
      footSwitchFactory.setFootContactableBodies(contactableFeet);
      footSwitchFactory.setFullRobotModel(fullRobotModel.get());
      footSwitchFactory.setJointDesiredOutputList(jointDesiredOutputList.get());
      footSwitchFactory.setGravity(gravity.get());
      footSwitchFactory.setSimulatedRobot(sdfRobot.get());
      footSwitchFactory.setYoVariableRegistry(factoryRegistry);
      footSwitchFactory.setFootSwitchType(footSwitchType.get());
      footSwitchFactory.setVariableSuffix("StateEstimator");

      stateEstimatorFootSwitches = footSwitchFactory.createFootSwitches();
   }

   private void createStateEstimator()
   {
      if (useStateEstimator.get())
      {
         centerOfMassDataHolder = new CenterOfMassDataHolder();

         QuadrupedStateEstimatorFactory stateEstimatorFactory = new QuadrupedStateEstimatorFactory();
         stateEstimatorFactory.setEstimatorDT(controlDT.get());
         stateEstimatorFactory.setFootContactableBodies(contactableFeet);
         stateEstimatorFactory.setFootSwitches(stateEstimatorFootSwitches);
         stateEstimatorFactory.setFullRobotModel(fullRobotModel.get());
         stateEstimatorFactory.setGravity(gravity.get());
         stateEstimatorFactory.setSensorInformation(sensorInformation.get());
         stateEstimatorFactory.setSensorOutputMapReadOnly(sensorReader.getProcessedSensorOutputMap());
         stateEstimatorFactory.setStateEstimatorParameters(stateEstimatorParameters.get());
         stateEstimatorFactory.setCenterOfMassDataHolder(centerOfMassDataHolder);
         stateEstimatorFactory.setYoGraphicsListRegistry(yoGraphicsListRegistry);
         stateEstimatorFactory.setRobotMotionStatusFromControllerHolder(robotMotionStatusFromController);
         stateEstimator = stateEstimatorFactory.createStateEstimator();

         Robot simulationRobot = sdfRobot.get();
         StateChangedListener<HighLevelControllerName> reinilizator = QuadrupedSimulationFactory.createSimulationStateEstimatorReinilizator(HighLevelControllerName.STAND_READY,
                                                                                                                                            stateEstimator,
                                                                                                                                            () -> simulationRobot.getFloatingRootJoint()
                                                                                                                                                                 .getFrameAfterJoint()
                                                                                                                                                                 .getTransformToRoot());
         controllerManager.registerHighLevelStateChangedListener(reinilizator);
         factoryRegistry.addChild(stateEstimator.getYoRegistry());
      }
      else
      {
         stateEstimator = null;
      }
   }

   private void setupYoVariableServer()
   {
      if (createYoVariableServer.get())
      {
         yoVariableServer = new YoVariableServer(getClass(), null, new DataServerSettings(false), controlDT.get());
      }
   }

   private void createRealtimeROS2Node()
   {
      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(pubSubImplementation.get(), "ihmc_quadruped_simulation");
   }

   public void createControllerManager()
   {
      robotMotionStatusFromController = new RobotMotionStatusHolder();
      QuadrupedRuntimeEnvironment runtimeEnvironment = new QuadrupedRuntimeEnvironment(controlDT.get(),
                                                                                       simulationConstructionSet.getTime(),
                                                                                       fullRobotModel.get(),
                                                                                       controllerCoreOptimizationSettings.get(),
                                                                                       jointDesiredOutputList.get(),
                                                                                       sdfRobot.get().getRegistry(),
                                                                                       yoGraphicsListRegistry,
                                                                                       contactableFeet,
                                                                                       contactablePlaneBodies,
                                                                                       centerOfMassDataHolder,
                                                                                       controllerFootSwitches,
                                                                                       stateEstimatorFootSwitches,
                                                                                       gravity.get(),
                                                                                       highLevelControllerParameters.get(),
                                                                                       dcmPlannerParameters.get(),
                                                                                       sitDownParameters.get(),
                                                                                       privilegedConfigurationParameters.get(),
                                                                                       fallDetectionParameters.get(),
                                                                                       robotMotionStatusFromController);
      if (customCoMTrajectoryPlanner.hasValue())
         runtimeEnvironment.setComTrajectoryPlanner(customCoMTrajectoryPlanner.get());

      controllerManager = new QuadrupedControllerManager(runtimeEnvironment, physicalProperties.get(), initialForceControlState.get(), null);
   }

   private void createPoseCommunicator()
   {
      RobotConfigurationDataPublisherFactory factory = new RobotConfigurationDataPublisherFactory();
      factory.setDefinitionsToPublish(fullRobotModel.get());
      factory.setSensorSource(fullRobotModel.get(), sensorReader.getRawSensorOutputMap());
      factory.setRobotMotionStatusHolder(controllerManager.getMotionStatusHolder());
      factory.setROS2Info(realtimeROS2Node, ROS2Tools.getQuadrupedControllerOutputTopic(sdfRobot.get().getName()));

      robotConfigurationDataPublisher = factory.createRobotConfigurationDataPublisher();
   }

   private void createControllerNetworkSubscriber()
   {
      controllerManager.createControllerNetworkSubscriber(sdfRobot.get().getName(), realtimeROS2Node); // TODO Verify that it is the right name to use.
   }

   private void createGroundContactModel()
   {
      if (terrainObjectDefinitions.isEmpty())
      {
         switch (groundContactModelType.get())
         {
            case FLAT:
               setCommonAvatarEnvrionmentInterface(new FlatGroundEnvironment());
               break;
            default:
               break;
         }
      }
   }

   private void createSimulationController()
   {
      simulationController = new QuadrupedSimulationController(sdfRobot.get(),
                                                               sensorReader,
                                                               outputWriter.get(),
                                                               controllerManager,
                                                               stateEstimator,
                                                               robotConfigurationDataPublisher,
                                                               yoVariableServer);
      simulationController.getYoRegistry().addChild(factoryRegistry);
   }

   private void setupSDFRobot()
   {
      int simulationTicksPerControllerTick = (int) Math.round(controlDT.get() / simulationDT.get());
      sdfRobot.get().addThrottledController(simulationController, simulationTicksPerControllerTick);

      sdfRobot.get().initializeState();

      double totalMass = TotalMassCalculator.computeSubTreeMass(sdfRobot.get().getRootBody());

      if (useStateEstimator.get())
      {
         RigidBodyTransform rootJointTransform = sdfRobot.get().getFloatingRootJoint().getFrameAfterJoint().getTransformToRoot();
         stateEstimator.initializeEstimator(rootJointTransform);
      }

      simulationConstructionSet.getSimulationSession().getGravity().setZ(gravity.get());
      ContactPointBasedPhysicsEngine physicsEngine = (ContactPointBasedPhysicsEngine) simulationConstructionSet.getSimulationSession().getPhysicsEngine();
      ContactPointBasedContactParameters parameters = ContactPointBasedContactParameters.defaultParameters();
      parameters.setKz(groundContactParameters.get().getZStiffness());
      parameters.setBz(groundContactParameters.get().getZDamping());
      parameters.setKxy(groundContactParameters.get().getXYStiffness());
      parameters.setBxy(groundContactParameters.get().getXYDamping());
      physicsEngine.setGroundContactParameters(parameters);
      LogTools.info(sdfRobot.get().getName() + " total mass: " + totalMass);
   }

   public SimulationConstructionSet2 createSimulation()
   {
      simulationConstructionSet = new SimulationConstructionSet2();
      simulationConstructionSet.setVisualizerEnabled(scsParameters.get().getCreateGUI());
      groundContactModelType.setDefaultValue(QuadrupedGroundContactModelType.FLAT);
      usePushRobotController.setDefaultValue(false);
      footSwitchType.setDefaultValue(FootSwitchType.TouchdownBased);
      kneeTorqueTouchdownDetectionThreshold.setDefaultValue(new QuadrantDependentList<>(20.0, 20.0, 20.0, 20.0));
      kneeTorqueTouchdownForSureDetectionThreshold.setDefaultValue(new QuadrantDependentList<>(75.0, 75.0, 75.0, 75.0));
      createYoVariableServer.setDefaultValue(false);

      FactoryTools.checkAllFactoryFieldsAreSet(this);

      setupYoRegistries();
      createPushRobotController();
      createContactableFeet();
      createContactablePlaneBodies();
      createFootSwitches();
      createStateEstimatorFootSwitches();
      createSensorReader();
      createRealtimeROS2Node();
      createControllerManager();
      createStateEstimator();
      createControllerNetworkSubscriber();
      createPoseCommunicator();
      setupYoVariableServer();

      createGroundContactModel();
      createSimulationController();
      setupSDFRobot();
      setupJointFriction();

      realtimeROS2Node.spin();

      simulationConstructionSet.addRobot(sdfRobot.get());
      simulationConstructionSet.addTerrainObjects(terrainObjectDefinitions);
      simulationConstructionSet.addYoGraphics(SCS1GraphicConversionTools.toYoGraphicDefinitions(yoGraphicsListRegistry));
      simulationConstructionSet.setDT(simulationDT.get());
      simulationConstructionSet.setBufferRecordTickPeriod(recordFrequency.get());

      if (simulationConstructionSet.isVisualizerEnabled())
      {
         if (useTrackAndDolly.get())
            simulationConstructionSet.requestCameraRigidBodyTracking(sdfRobot.get().getName(), sdfRobot.get().getFloatingRootJoint().getSuccessor().getName());
      }

      InputStream parameterFile = modelFactory.get().getParameterInputStream();
      ParameterLoaderHelper.loadParameters(this, parameterFile, simulationController.getYoRegistry(), true);

      if (yoVariableServer != null)
      {
         yoVariableServer.start();
      }

      FactoryTools.disposeFactory(this);

      return simulationConstructionSet;
   }

   // OPTIONS

   private void setupJointFriction()
   {
      if (simulatedFrictionParameters.hasValue())
      {
         Robot robot = sdfRobot.get();
         SimulatedFrictionController springJointOutputWriter = new SimulatedFrictionController(robot.getControllerInput(),
                                                                                               robot.getControllerOutput(),
                                                                                               simulatedFrictionParameters.get());
         robot.addController(springJointOutputWriter);
      }
   }

   public void setSimulationDT(double simulationDT)
   {
      this.simulationDT.set(simulationDT);
   }

   public void setControlDT(double controlDT)
   {
      this.controlDT.set(controlDT);
   }

   public void setJointDesiredOutputList(JointDesiredOutputList jointDesiredOutputList)
   {
      this.jointDesiredOutputList.set(jointDesiredOutputList);
   }

   public void setHighLevelControllerParameters(HighLevelControllerParameters highLevelControllerParameters)
   {
      this.highLevelControllerParameters.set(highLevelControllerParameters);
   }

   public void setDCMPlannerParameters(DCMPlannerParameters dcmPlannerParameters)
   {
      this.dcmPlannerParameters.set(dcmPlannerParameters);
   }

   public void setSitDownParameters(QuadrupedSitDownParameters sitDownParameters)
   {
      this.sitDownParameters.set(sitDownParameters);
   }

   public void setPrivilegedConfigurationParameters(QuadrupedPrivilegedConfigurationParameters privilegedConfigurationParameters)
   {
      this.privilegedConfigurationParameters.set(privilegedConfigurationParameters);
   }

   public void setFallDetectionParameters(QuadrupedFallDetectionParameters fallDetectionParameters)
   {
      this.fallDetectionParameters.set(fallDetectionParameters);
   }

   public void setGravity(double gravity)
   {
      this.gravity.set(gravity);
   }

   public void setRecordFrequency(int recordFrequency)
   {
      this.recordFrequency.set(recordFrequency);
   }

   public void setUseTrackAndDolly(boolean useTrackAndDolly)
   {
      this.useTrackAndDolly.set(useTrackAndDolly);
   }

   public void setModelFactory(QuadrupedModelFactory modelFactory)
   {
      this.modelFactory.set(modelFactory);
   }

   public void setSCSParameters(SimulationConstructionSetParameters scsParameters)
   {
      this.scsParameters.set(scsParameters);
   }

   public void setGroundContactModelType(QuadrupedGroundContactModelType groundContactModelType)
   {
      this.groundContactModelType.set(groundContactModelType);
   }

   public void setSimulatedFrictionParameters(CoulombViscousStribeckFrictionParameters simulatedFrictionParameters)
   {
      this.simulatedFrictionParameters.set(simulatedFrictionParameters);
   }

   public void setGroundContactParameters(GroundContactParameters groundContactParameters)
   {
      this.groundContactParameters.set(groundContactParameters);
   }

   public void setOutputWriter(OutputWriter outputWriter)
   {
      this.outputWriter.set(outputWriter);
   }

   public void setPhysicalProperties(QuadrupedPhysicalProperties physicalProperties)
   {
      this.physicalProperties.set(physicalProperties);
   }

   public void setFullRobotModel(FullQuadrupedRobotModel fullRobotModel)
   {
      this.fullRobotModel.set(fullRobotModel);
   }

   public void setControllerCoreOptimizationSettings(ControllerCoreOptimizationSettings controllerCoreOptimizationSettings)
   {
      this.controllerCoreOptimizationSettings.set(controllerCoreOptimizationSettings);
   }

   public void setSDFRobot(Robot sdfRobot)
   {
      this.sdfRobot.set(sdfRobot);
   }

   public void setKneeTorqueTouchdownDetectionThreshold(QuadrantDependentList<Double> kneeTorqueTouchdownDetectionThreshold)
   {
      this.kneeTorqueTouchdownDetectionThreshold.set(kneeTorqueTouchdownDetectionThreshold);
   }

   public void setKneeTorqueTouchdownForSureDetectionThreshold(QuadrantDependentList<Double> kneeTorqueTouchdownDetectionThreshold)
   {
      this.kneeTorqueTouchdownForSureDetectionThreshold.set(kneeTorqueTouchdownDetectionThreshold);
   }

   public void setUseStateEstimator(boolean useStateEstimator)
   {
      this.useStateEstimator.set(useStateEstimator);
   }

   public void setSensorInformation(QuadrupedSensorInformation sensorInformation)
   {
      this.sensorInformation.set(sensorInformation);
   }

   public void setStateEstimatorParameters(StateEstimatorParameters stateEstimatorParameters)
   {
      this.stateEstimatorParameters.set(stateEstimatorParameters);
   }

   public void setReferenceFrames(QuadrupedReferenceFrames referenceFrames)
   {
      this.referenceFrames.set(referenceFrames);
   }

   public void addTerrainObjectDefinition(TerrainObjectDefinition terrainObjectDefinition)
   {
      terrainObjectDefinitions.add(terrainObjectDefinition);
   }

   public void setTerrainObject3D(TerrainObject3D terrainObject3D)
   {
      addTerrainObjectDefinition(TerrainObjectDefinitionTools.toTerrainObjectDefinition(terrainObject3D,
                                                                                        collidableHelper,
                                                                                        terrainCollisionName,
                                                                                        robotCollisionName));
   }

   public void setCommonAvatarEnvrionmentInterface(CommonAvatarEnvironmentInterface environment)
   {
      addTerrainObjectDefinition(TerrainObjectDefinitionTools.toTerrainObjectDefinition(environment,
                                                                                        collidableHelper,
                                                                                        terrainCollisionName,
                                                                                        robotCollisionName));
   }

   public void setUsePushRobotController(boolean usePushRobotController)
   {
      this.usePushRobotController.set(usePushRobotController);
   }

   public void setFootSwitchType(FootSwitchType footSwitchType)
   {
      this.footSwitchType.set(footSwitchType);
   }

   public void setScsBufferSize(int scsBufferSize)
   {
      this.scsBufferSize.set(scsBufferSize);
   }

   public void setInitialForceControlState(HighLevelControllerName initialForceControlState)
   {
      this.initialForceControlState.set(initialForceControlState);
   }

   public void setPubSubImplementation(PubSubImplementation pubSubImplementation)
   {
      this.pubSubImplementation.set(pubSubImplementation);
   }

   public void setSensorReaderWrapper(QuadrupedSensorReaderWrapper sensorReaderWrapper)
   {
      this.sensorReaderWrapper = sensorReaderWrapper;
   }

   public void setCreateYoVariableServer(boolean createYoVariableServer)
   {
      this.createYoVariableServer.set(createYoVariableServer);
   }

   public void setCustomCoMTrajectoryPlanner(DCMPlannerInterface dcmPlannerInterface)
   {
      this.customCoMTrajectoryPlanner.set(dcmPlannerInterface);
   }

   public void close()
   {
      if (realtimeROS2Node != null)
      {
         realtimeROS2Node.destroy();
      }
   }

   public static void setRobotDefinitionInitialJointStates(QuadrupedInitialPositionParameters parameters,
                                                           Collection<QuadrupedJointName> quadrupedJointNames,
                                                           Function<QuadrupedJointName, String> jointNameRetriever,
                                                           RobotDefinition robotDefinition)
   {
      robotDefinition.getRootJointDefinitions().get(0)
                     .setInitialJointState(new SixDoFJointState(parameters.getInitialBodyOrientation(), parameters.getInitialBodyPosition()));

      for (QuadrupedJointName quadrupedJointName : quadrupedJointNames)
      {
         double q_init = parameters.getInitialJointPosition(quadrupedJointName);
         String jointName = jointNameRetriever.apply(quadrupedJointName);
         OneDoFJointDefinition joint = robotDefinition.getOneDoFJointDefinition(jointName);
         joint.setInitialJointState(q_init);
      }
   }

   /**
    * <b>For simulation only!</b>
    * <p>
    * Creates a listener to be attached to the controller. The listener will trigger when the
    * controller state machine enters {@code controllerStateTrigger} for the first time. When
    * triggering, the state estimator position estimation is reinilization to the actual one using the
    * given supplier.
    * </p>
    * 
    * @param controllerStateTrigger           the enum value that should trigger the reinitialization.
    * @param stateEstimator                   the instance of the state estimator to be automatically
    *                                         reinitialized.
    * @param actualRootJointTransformSupplier the supplier of the root joint transofmr of the simulated
    *                                         robot.
    * @return the reinitializator.
    */
   public static StateChangedListener<HighLevelControllerName> createSimulationStateEstimatorReinilizator(HighLevelControllerName controllerStateTrigger,
                                                                                                          StateEstimatorController stateEstimator,
                                                                                                          Supplier<RigidBodyTransform> actualRootJointTransformSupplier)
   {
      return new StateChangedListener<HighLevelControllerName>()
      {
         private boolean reinitilizeEstimator = true;

         @Override
         public void stateChanged(HighLevelControllerName from, HighLevelControllerName to)
         {
            if (reinitilizeEstimator && to == controllerStateTrigger)
            {
               stateEstimator.initializeEstimator(actualRootJointTransformSupplier.get());
               reinitilizeEstimator = false;
            }
         }
      };
   }
}
