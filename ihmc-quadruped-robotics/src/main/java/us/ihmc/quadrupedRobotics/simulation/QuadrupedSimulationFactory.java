package us.ihmc.quadrupedRobotics.simulation;

import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedCommunication.QuadrupedControllerAPIDefinition;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerManager;
import us.ihmc.quadrupedRobotics.controller.QuadrupedSimulationController;
import us.ihmc.quadrupedRobotics.estimator.footSwitch.QuadrupedFootSwitchFactory;
import us.ihmc.quadrupedRobotics.estimator.sensorProcessing.simulatedSensors.SDFQuadrupedPerfectSimulatedSensor;
import us.ihmc.quadrupedRobotics.estimator.stateEstimator.QuadrupedSensorInformation;
import us.ihmc.quadrupedRobotics.estimator.stateEstimator.QuadrupedSensorReaderWrapper;
import us.ihmc.quadrupedRobotics.estimator.stateEstimator.QuadrupedStateEstimatorFactory;
import us.ihmc.quadrupedRobotics.model.QuadrupedInitialOffsetAndYaw;
import us.ihmc.quadrupedRobotics.model.QuadrupedInitialPositionParameters;
import us.ihmc.quadrupedRobotics.model.QuadrupedModelFactory;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedFallDetectionParameters;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedPrivilegedConfigurationParameters;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedSitDownParameters;
import us.ihmc.quadrupedRobotics.planning.trajectory.DCMPlannerParameters;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotModels.OutputWriter;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.robotics.stateMachine.core.StateChangedListener;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.sensorProcessing.communication.producers.DRCPoseCommunicator;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.sensorData.JointConfigurationGatherer;
import us.ihmc.sensorProcessing.sensorProcessors.SensorTimestampHolder;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.SimulatedSensorHolderAndReaderFromRobotFactory;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationConstructionSetTools.util.ground.RotatablePlaneTerrainProfile;
import us.ihmc.simulationToolkit.controllers.PushRobotController;
import us.ihmc.simulationToolkit.controllers.SpringJointOutputWriter;
import us.ihmc.simulationToolkit.parameters.SimulatedElasticityParameters;
import us.ihmc.simulationconstructionset.CameraMount;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.ViewportConfiguration;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.AlternatingSlopesGroundProfile;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.simulationconstructionset.util.ground.RollingGroundProfile;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.VaryingStairGroundProfile;
import us.ihmc.stateEstimation.humanoid.StateEstimatorController;
import us.ihmc.systemIdentification.frictionId.simulators.CoulombViscousStribeckFrictionParameters;
import us.ihmc.systemIdentification.frictionId.simulators.SimulatedFrictionController;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.factories.RequiredFactoryField;
import us.ihmc.wholeBodyController.parameters.ParameterLoaderHelper;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class QuadrupedSimulationFactory
{
   private final RequiredFactoryField<FullQuadrupedRobotModel> fullRobotModel = new RequiredFactoryField<>("fullRobotModel");
   private final RequiredFactoryField<ControllerCoreOptimizationSettings> controllerCoreOptimizationSettings = new RequiredFactoryField<>("controllerCoreOptimizationSettings");
   private final RequiredFactoryField<QuadrupedPhysicalProperties> physicalProperties = new RequiredFactoryField<>("physicalProperties");
   private final RequiredFactoryField<FloatingRootJointRobot> sdfRobot = new RequiredFactoryField<>("sdfRobot");
   private final RequiredFactoryField<Double> simulationDT = new RequiredFactoryField<>("simulationDT");
   private final RequiredFactoryField<Double> controlDT = new RequiredFactoryField<>("controlDT");
   private final RequiredFactoryField<Double> gravity = new RequiredFactoryField<>("gravity");
   private final RequiredFactoryField<Integer> recordFrequency = new RequiredFactoryField<>("recordFrequency");
   private final RequiredFactoryField<Boolean> useTrackAndDolly = new RequiredFactoryField<>("useTrackAndDolly");
   private final RequiredFactoryField<Boolean> showPlotter = new RequiredFactoryField<>("showPlotter");
   private final RequiredFactoryField<QuadrupedModelFactory> modelFactory = new RequiredFactoryField<>("modelFactory");
   private final RequiredFactoryField<SimulationConstructionSetParameters> scsParameters = new RequiredFactoryField<>("scsParameters");
   private final RequiredFactoryField<GroundContactParameters> groundContactParameters = new RequiredFactoryField<>("groundContactParameters");
   private final RequiredFactoryField<QuadrupedInitialPositionParameters> initialPositionParameters = new RequiredFactoryField<>("initialPositionParameters");
   private final RequiredFactoryField<QuadrupedInitialOffsetAndYaw> initialOffset = new RequiredFactoryField<>("initialOffset");
   private final RequiredFactoryField<OutputWriter> outputWriter = new RequiredFactoryField<>("outputWriter");
   private final RequiredFactoryField<SensorTimestampHolder> timestampProvider = new RequiredFactoryField<>("timestampProvider");
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

   private final OptionalFactoryField<SimulatedElasticityParameters> simulatedElasticityParameters = new OptionalFactoryField<>("simulatedElasticityParameters");
   private final OptionalFactoryField<CoulombViscousStribeckFrictionParameters> simulatedFrictionParameters = new OptionalFactoryField<>("jointFrictionParameters");
   private final OptionalFactoryField<QuadrupedGroundContactModelType> groundContactModelType = new OptionalFactoryField<>("groundContactModelType");
   private final OptionalFactoryField<GroundProfile3D> providedGroundProfile3D = new OptionalFactoryField<>("providedGroundProfile3D");
   private final OptionalFactoryField<TerrainObject3D> providedTerrainObject3D = new OptionalFactoryField<>("providedTerrainObject3D");
   private final OptionalFactoryField<Boolean> usePushRobotController = new OptionalFactoryField<>("usePushRobotController");
   private final OptionalFactoryField<FootSwitchType> footSwitchType = new OptionalFactoryField<>("footSwitchType");
   private final OptionalFactoryField<QuadrantDependentList<Double>> kneeTorqueTouchdownDetectionThreshold = new OptionalFactoryField<>("kneeTorqueTouchdownDetectionThreshold");
   private final OptionalFactoryField<QuadrantDependentList<Double>> kneeTorqueTouchdownForSureDetectionThreshold = new OptionalFactoryField<>("kneeTorqueTouchdownForSureDetectionThreshold");
   private final OptionalFactoryField<Integer> scsBufferSize = new OptionalFactoryField<>("scsBufferSize");
   private final OptionalFactoryField<HighLevelControllerName> initialForceControlState = new OptionalFactoryField<>("initialForceControlState");
   private final OptionalFactoryField<Boolean> useLocalCommunicator = new OptionalFactoryField<>("useLocalCommunicator");
   private final OptionalFactoryField<Boolean> createYoVariableServer = new OptionalFactoryField<>("createYoVariableServer");


   // TO CONSTRUCT
   private YoVariableRegistry factoryRegistry;
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
   private RealtimeRos2Node realtimeRos2Node;
   private QuadrupedControllerManager controllerManager;
   private DRCPoseCommunicator poseCommunicator;
   private GroundProfile3D groundProfile3D;
   private LinearGroundContactModel groundContactModel;
   private QuadrupedSimulationController simulationController;
   private List<CameraConfiguration> cameraConfigurations = new ArrayList<>();
   private YoVariableServer yoVariableServer;

   /**
    * The PacketCommunicator used as input of the controller is either equal to the output
    * PacketCommunicator of the network processor or the behavior module if any. It is bidirectional
    * meaning that it carries commands to be executed by the controller and that the controller is
    * able to send feedback the other way to whoever is listening to the PacketCommunicator.
    */
   private PacketCommunicator controllerPacketCommunicator;

   // CREATION

   private void setupYoRegistries()
   {
      factoryRegistry = new YoVariableRegistry("factoryRegistry");
      yoGraphicsListRegistry = new YoGraphicsListRegistry();
      yoGraphicsListRegistry.setYoGraphicsUpdatedRemotely(true);
   }

   private void createPushRobotController()
   {
      if (usePushRobotController.get())
      {
         FloatingRootJointRobot pushableRobot = sdfRobot.get();
         String rootJointName = pushableRobot.getRootJoint().getName();

         PushRobotController bodyPushRobotController = new PushRobotController(pushableRobot, rootJointName, new Vector3D(0.0, -0.00057633, 0.0383928), 0.01);
         yoGraphicsListRegistry.registerYoGraphic("PushRobotControllers", bodyPushRobotController.getForceVisualizer());

         for (QuadrupedJointName quadrupedJointName : modelFactory.get().getQuadrupedJointNames())
         {
            String jointName = modelFactory.get().getSDFNameForJointName(quadrupedJointName);
            PushRobotController jointPushRobotController = new PushRobotController(sdfRobot.get(), jointName, new Vector3D(0.0, 0.0, 0.0), 0.01);
            yoGraphicsListRegistry.registerYoGraphic("PushRobotControllers", jointPushRobotController.getForceVisualizer());
         }
      }
   }

   private void createSensorReader()
   {
      SensorReader sensorReader;
      if (useStateEstimator.get())
      {
         FloatingJointBasics rootInverseDynamicsJoint = fullRobotModel.get().getRootJoint();
         IMUDefinition[] imuDefinitions = fullRobotModel.get().getIMUDefinitions();
         ForceSensorDefinition[] forceSensorDefinitions = fullRobotModel.get().getForceSensorDefinitions();
         JointDesiredOutputList estimatorDesiredJointDataHolder = null;

         SimulatedSensorHolderAndReaderFromRobotFactory sensorReaderFactory;
         sensorReaderFactory = new SimulatedSensorHolderAndReaderFromRobotFactory(sdfRobot.get(), stateEstimatorParameters.get());
         sensorReaderFactory.build(rootInverseDynamicsJoint, imuDefinitions, forceSensorDefinitions, estimatorDesiredJointDataHolder, factoryRegistry);

         sensorReader = sensorReaderFactory.getSensorReader();
      }
      else
      {
         sensorReader = new SDFQuadrupedPerfectSimulatedSensor(sdfRobot.get(), fullRobotModel.get(), referenceFrames.get(), stateEstimatorFootSwitches);
      }

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
         stateEstimatorFactory.setSensorOutputMapReadOnly(sensorReader.getSensorOutputMapReadOnly());
         stateEstimatorFactory.setStateEstimatorParameters(stateEstimatorParameters.get());
         stateEstimatorFactory.setCenterOfMassDataHolder(centerOfMassDataHolder);
         stateEstimatorFactory.setYoGraphicsListRegistry(yoGraphicsListRegistry);
         stateEstimatorFactory.setRobotMotionStatusFromControllerHolder(robotMotionStatusFromController);
         stateEstimator = stateEstimatorFactory.createStateEstimator();

         FloatingRootJointRobot simulationRobot = sdfRobot.get();
         StateChangedListener<HighLevelControllerName> reinilizator = QuadrupedSimulationFactory.createSimulationStateEstimatorReinilizator(HighLevelControllerName.STAND_READY,
                                                                                                                                                stateEstimator,
                                                                                                                                                () -> simulationRobot.getRootJoint()
                                                                                                                                                                     .getJointTransform3D());
         controllerManager.registerHighLevelStateChangedListener(reinilizator);
         factoryRegistry.addChild(stateEstimator.getYoVariableRegistry());
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
         yoVariableServer = new YoVariableServer(getClass(), null, LogSettings.SIMULATION, controlDT.get());
      }
   }

   private void createRealtimeRos2Node()
   {
      PubSubImplementation pubSubImplementation = useLocalCommunicator.get() ? PubSubImplementation.INTRAPROCESS : PubSubImplementation.FAST_RTPS;
      realtimeRos2Node = ROS2Tools.createRealtimeRos2Node(pubSubImplementation, "ihmc_quadruped_simulation");
   }

   public void createControllerManager()
   {
      robotMotionStatusFromController = new RobotMotionStatusHolder();
      QuadrupedRuntimeEnvironment runtimeEnvironment = new QuadrupedRuntimeEnvironment(controlDT.get(), sdfRobot.get().getYoTime(), fullRobotModel.get(),
                                                                                       controllerCoreOptimizationSettings.get(), jointDesiredOutputList.get(),
                                                                                       sdfRobot.get().getRobotsYoVariableRegistry(), yoGraphicsListRegistry,
                                                                                       contactableFeet, contactablePlaneBodies, centerOfMassDataHolder,
                                                                                       controllerFootSwitches, stateEstimatorFootSwitches, gravity.get(),
                                                                                       highLevelControllerParameters.get(), dcmPlannerParameters.get(),
                                                                                       sitDownParameters.get(), privilegedConfigurationParameters.get(),
                                                                                       fallDetectionParameters.get(), robotMotionStatusFromController);

      controllerManager = new QuadrupedControllerManager(runtimeEnvironment, physicalProperties.get(), initialForceControlState.get(), null);
   }

   private void createPoseCommunicator()
   {
      JointConfigurationGatherer jointConfigurationGathererAndProducer = new JointConfigurationGatherer(fullRobotModel.get());
      MessageTopicNameGenerator publisherTopicNameGenerator = QuadrupedControllerAPIDefinition.getPublisherTopicNameGenerator(sdfRobot.get().getName());
      poseCommunicator = new DRCPoseCommunicator(fullRobotModel.get(),
                                                 jointConfigurationGathererAndProducer,
                                                 publisherTopicNameGenerator,
                                                 realtimeRos2Node,
                                                 timestampProvider.get(),
                                                 sensorReader.getSensorRawOutputMapReadOnly(),
                                                 controllerManager.getMotionStatusHolder(),
                                                 null);
   }

   private void createControllerNetworkSubscriber()
   {
      controllerManager.createControllerNetworkSubscriber(sdfRobot.get().getName(), realtimeRos2Node); // TODO Verify that it is the right name to use.
   }

   private void createGroundContactModel()
   {
      if (!providedGroundProfile3D.hasValue() && !providedTerrainObject3D.hasValue())
      {
         switch (groundContactModelType.get())
         {
         case FLAT:
            groundProfile3D = new FlatGroundProfile(0.0);
            break;
         case ROLLING_HILLS:
            groundProfile3D = new RollingGroundProfile(0.025, 1.0, 0.0, -20.0, 20.0, -20.0, 20.0);
            break;
         case ROTATABLE:
            groundProfile3D = new RotatablePlaneTerrainProfile(new Point3D(), sdfRobot.get(), yoGraphicsListRegistry, controlDT.get());
            break;
         case SLOPES:
            double xMin = -5.0, xMax = 40.0;
            double yMin = -5.0, yMax = 5.0;
            double[][] xSlopePairs = new double[][] {{1.0, 0.0}, {3.0, 0.1}};
            groundProfile3D = new AlternatingSlopesGroundProfile(xSlopePairs, xMin, xMax, yMin, yMax);
            break;
         case STEPPED:
            groundProfile3D = new VaryingStairGroundProfile(0.0, 0.0, new double[] {1.5, 1.0, 1.0, 0.5}, new double[] {0.0, 0.05, -0.1, 0.05, 0.05});
            break;
         default:
            groundProfile3D = null;
            break;
         }
      }
      else if (providedTerrainObject3D.hasValue())
      {
         if (providedGroundProfile3D.hasValue())
            throw new RuntimeException("You can only set either a terrain object or a ground profile.");

         groundProfile3D = providedTerrainObject3D.get();
      }
      else
      {
         groundProfile3D = providedGroundProfile3D.get();
      }

      groundContactModel = new LinearGroundContactModel(sdfRobot.get(), sdfRobot.get().getRobotsYoVariableRegistry());
      groundContactModel.setZStiffness(groundContactParameters.get().getZStiffness());
      groundContactModel.setZDamping(groundContactParameters.get().getZDamping());
      groundContactModel.setXYStiffness(groundContactParameters.get().getXYStiffness());
      groundContactModel.setXYDamping(groundContactParameters.get().getXYDamping());
      groundContactModel.setGroundProfile3D(groundProfile3D);
   }

   private void createSimulationController()
   {
      simulationController = new QuadrupedSimulationController(sdfRobot.get(), sensorReader, outputWriter.get(), controllerManager, stateEstimator,
                                                               poseCommunicator, yoVariableServer);
      simulationController.getYoVariableRegistry().addChild(factoryRegistry);
   }

   private void setupSDFRobot()
   {
      initialPositionParameters.get().offsetInitialConfiguration(initialOffset.get());

      int simulationTicksPerControllerTick = (int) Math.round(controlDT.get() / simulationDT.get());
      sdfRobot.get().setController(simulationController, simulationTicksPerControllerTick);
      sdfRobot.get().setPositionInWorld(initialPositionParameters.get().getInitialBodyPosition());
      sdfRobot.get().setOrientation(initialPositionParameters.get().getInitialBodyOrientation());

      for (QuadrupedJointName quadrupedJointName : modelFactory.get().getQuadrupedJointNames())
      {
         OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint = sdfRobot.get()
                                                                   .getOneDegreeOfFreedomJoint(modelFactory.get().getSDFNameForJointName(quadrupedJointName));
         if (oneDegreeOfFreedomJoint != null)
         {
            oneDegreeOfFreedomJoint.setQ(initialPositionParameters.get().getInitialJointPosition(quadrupedJointName));
         }
      }
      try
      {
         sdfRobot.get().update();
         sdfRobot.get().doDynamicsButDoNotIntegrate();
         sdfRobot.get().update();
      }
      catch (UnreasonableAccelerationException unreasonableAccelerationException)
      {
         throw new RuntimeException("UnreasonableAccelerationException");
      }

      Point3D initialCoMPosition = new Point3D();
      double totalMass = sdfRobot.get().computeCenterOfMass(initialCoMPosition);

      if (useStateEstimator.get())
      {
         RigidBodyTransform rootJointTransform = sdfRobot.get().getRootJoint().getJointTransform3D();
         stateEstimator.initializeEstimator(rootJointTransform);
      }

      sdfRobot.get().setGravity(gravity.get());
      sdfRobot.get().setGroundContactModel(groundContactModel);
      PrintTools.info(this, sdfRobot.get().getName() + " total mass: " + totalMass);
   }

   private void setupCameras()
   {
      FloatingRootJointRobot robot = sdfRobot.get();
      RigidBodyTransform cameraTransform = new RigidBodyTransform();
      List<CameraMount> cameraMounts = new ArrayList<>();

      // straight behind
      cameraTransform.setTranslation(-2.4, 0.0, 0.5);
      cameraTransform.setRotationEuler(0.0, Math.toRadians(15.0), 0.0);
      cameraMounts.add(new CameraMount("ThirdPersonViewBehind", cameraTransform, 1.4, 0.5, 20.0, robot));

      // behind and to the side
      cameraTransform.setTranslation(-2.0, 0.95, 1.0);
      cameraTransform.setRotationEuler(0.0, Math.toRadians(25.0), Math.toRadians(-18.0));
      cameraMounts.add(new CameraMount("ThirdPersonViewSide", cameraTransform, 1.4, 0.5, 20.0, robot));

      // top-down
      cameraTransform.setTranslation(0.5, 0.0, 3.5);
      cameraTransform.setRotationEuler(0.0, Math.toRadians(90.0), 0.0);
      cameraMounts.add(new CameraMount("TopDownView", cameraTransform, 1.4, 0.5, 20.0, robot));

      for (int i = 0; i < cameraMounts.size(); i++)
      {
         robot.getRootJoint().addCameraMount(cameraMounts.get(i));
         CameraConfiguration cameraConfiguration = new CameraConfiguration(cameraMounts.get(i).getName());
         cameraConfiguration.setCameraMount(cameraMounts.get(i).getName());
         cameraConfiguration.setCameraTracking(false, false, false, false);
         cameraConfiguration.setCameraDolly(false, false, false, false);
         cameraConfigurations.add(cameraConfiguration);
      }
   }

   public SimulationConstructionSet createSimulation()
   {
      groundContactModelType.setDefaultValue(QuadrupedGroundContactModelType.FLAT);
      usePushRobotController.setDefaultValue(false);
      footSwitchType.setDefaultValue(FootSwitchType.TouchdownBased);
      useLocalCommunicator.setDefaultValue(false);
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
      createRealtimeRos2Node();
      createControllerManager();
      createStateEstimator();
      createControllerNetworkSubscriber();
      createPoseCommunicator();
      setupYoVariableServer();

      createGroundContactModel();
      createSimulationController();
      setupSDFRobot();
      setupJointElasticity();
      setupJointFriction();
      setupCameras();

      realtimeRos2Node.spin();

      SimulationConstructionSet scs = new SimulationConstructionSet(sdfRobot.get(), scsParameters.get());

      if (groundContactModelType.get() == QuadrupedGroundContactModelType.ROTATABLE || providedTerrainObject3D.hasValue())
      {
         scs.setGroundVisible(false);
      }
      if (scsBufferSize.hasValue())
      {
         scs.setMaxBufferSize(scsBufferSize.get());
      }

      if (providedTerrainObject3D.hasValue())
      {
         scs.addStaticLinkGraphics(providedTerrainObject3D.get().getLinkGraphics());
      }

      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.setDT(simulationDT.get(), recordFrequency.get());
      if (scs.getSimulationConstructionSetParameters().getCreateGUI())
      {
         scs.setCameraTrackingVars("q_x", "q_y", "q_z");
         scs.setCameraDollyVars("q_x", "q_y", "q_z");
         scs.setCameraTracking(useTrackAndDolly.get(), true, true, true);
         scs.setCameraDolly(useTrackAndDolly.get(), true, true, false);
         scs.setCameraDollyOffsets(4.0 * 0.66, -3.0 * 0.66 , 1 * 0.66);
         scs.setCameraPosition(-2.5, -4.5, 1.5);
         SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
         simulationOverheadPlotterFactory.setVariableNameToTrack("centerOfMass");
         simulationOverheadPlotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
         simulationOverheadPlotterFactory.setShowOnStart(showPlotter.get());
         simulationOverheadPlotterFactory.createOverheadPlotter();

         for (int i = 0; i < cameraConfigurations.size(); i++)
         {
            scs.setupCamera(cameraConfigurations.get(i));
         }

         ViewportConfiguration viewportConfiguration = new ViewportConfiguration("Multi-View");
         viewportConfiguration.addCameraView("ThirdPersonViewBehind", 0, 0, 1, 1);
         viewportConfiguration.addCameraView("Right Side", 1, 0, 1, 1);
         scs.setupViewport(viewportConfiguration);
      }

      InputStream parameterFile = modelFactory.get().getParameterInputStream();
      ParameterLoaderHelper.loadParameters(this, parameterFile, simulationController.getYoVariableRegistry(), true);
      scs.setParameterRootPath(simulationController.getYoVariableRegistry().getParent());

      if (yoVariableServer != null)
      {
         yoVariableServer.start();
      }

      FactoryTools.disposeFactory(this);

      return scs;
   }

   // OPTIONS

   private void setupJointElasticity()
   {
      if (simulatedElasticityParameters.hasValue())
      {
         FloatingRootJointRobot floatingRootJointRobot = sdfRobot.get();
         SpringJointOutputWriter springJointOutputWriter = new SpringJointOutputWriter(floatingRootJointRobot, simulatedElasticityParameters.get(),
                                                                                       simulationDT.get());
         floatingRootJointRobot.setController(springJointOutputWriter, 1);
      }
   }

   private void setupJointFriction()
   {
      if (simulatedFrictionParameters.hasValue())
      {
         FloatingRootJointRobot floatingRootJointRobot = sdfRobot.get();
         SimulatedFrictionController springJointOutputWriter = new SimulatedFrictionController(floatingRootJointRobot, simulatedFrictionParameters.get());
         floatingRootJointRobot.setController(springJointOutputWriter, 1);
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

   public void setShowPlotter(boolean showPlotter)
   {
      this.showPlotter.set(showPlotter);
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

   public void setSimulatedElasticityParameters(SimulatedElasticityParameters simulatedElasticityParameters)
   {
      this.simulatedElasticityParameters.set(simulatedElasticityParameters);
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

   public void setInitialPositionParameters(QuadrupedInitialPositionParameters initialPositionParameters)
   {
      this.initialPositionParameters.set(initialPositionParameters);
   }

   public void setInitialOffset(QuadrupedInitialOffsetAndYaw initialOffset)
   {
      this.initialOffset.set(initialOffset);
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

   public void setTimestampHolder(SensorTimestampHolder timestampProvider)
   {
      this.timestampProvider.set(timestampProvider);
   }

   public void setSDFRobot(FloatingRootJointRobot sdfRobot)
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

   public void setGroundProfile3D(GroundProfile3D groundProfile3D)
   {
      providedGroundProfile3D.set(groundProfile3D);
   }

   public void setTerrainObject3D(TerrainObject3D terrainObject3D)
   {
      providedTerrainObject3D.set(terrainObject3D);
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

   public void setUseLocalCommunicator(boolean useLocalCommunicator)
   {
      this.useLocalCommunicator.set(useLocalCommunicator);
   }

   public void setSensorReaderWrapper(QuadrupedSensorReaderWrapper sensorReaderWrapper)
   {
      this.sensorReaderWrapper = sensorReaderWrapper;
   }

   public void setCreateYoVariableServer(boolean createYoVariableServer)
   {
      this.createYoVariableServer.set(createYoVariableServer);
   }

   public void close()
   {
      if(realtimeRos2Node != null)
      {
         realtimeRos2Node.destroy();
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
    * @param controllerStateTrigger the enum value that should trigger the reinitialization.
    * @param stateEstimator the instance of the state estimator to be automatically reinitialized.
    * @param actualRootJointTransformSupplier the supplier of the root joint transofmr of the simulated robot.
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
