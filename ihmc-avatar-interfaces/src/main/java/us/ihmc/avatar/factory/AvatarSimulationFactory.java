package us.ihmc.avatar.factory;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.tuple.ImmutablePair;

import controller_msgs.msg.dds.StampedPosePacket;
import gnu.trove.map.TObjectDoubleMap;
import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.avatar.AvatarControllerThread;
import us.ihmc.avatar.AvatarEstimatorThread;
import us.ihmc.avatar.ControllerTask;
import us.ihmc.avatar.EstimatorTask;
import us.ihmc.avatar.RobotVisualizerList;
import us.ihmc.avatar.SimulationRobotVisualizer;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.SimulatedDRCRobotTimeProvider;
import us.ihmc.avatar.drcRobot.shapeContactSettings.DRCRobotModelShapeCollisionSettings;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextDataFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.BarrierScheduler.TaskOverrunBehavior;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicator;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPDGains;
import us.ihmc.robotics.partNames.JointRole;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;
import us.ihmc.sensorProcessing.parameters.DRCRobotLidarParameters;
import us.ihmc.sensorProcessing.simulatedSensors.DRCPerfectSensorReaderFactory;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.sensorProcessing.simulatedSensors.SimulatedSensorHolderAndReaderFromRobotFactory;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationToolkit.controllers.ActualCMPComputer;
import us.ihmc.simulationToolkit.controllers.JointLowLevelJointControlSimulator;
import us.ihmc.simulationToolkit.controllers.PIDLidarTorqueController;
import us.ihmc.simulationToolkit.controllers.PassiveJointController;
import us.ihmc.simulationToolkit.controllers.SimulatedRobotCenterOfMassVisualizer;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.simulationconstructionset.physics.collision.DefaultCollisionHandler;
import us.ihmc.simulationconstructionset.physics.collision.HybridImpulseSpringDamperCollisionHandler;
import us.ihmc.simulationconstructionset.physics.collision.simple.CollisionManager;
import us.ihmc.simulationconstructionset.util.AdditionalPanelTools;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.factories.RequiredFactoryField;
import us.ihmc.wholeBodyController.DRCOutputProcessor;
import us.ihmc.wholeBodyController.DRCOutputProcessorWithStateChangeSmoother;
import us.ihmc.wholeBodyController.DRCOutputProcessorWithTorqueOffsets;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.frameObjects.FrameIndexMap;

public class AvatarSimulationFactory
{
   private final RequiredFactoryField<DRCRobotModel> robotModel = new RequiredFactoryField<>("robotModel");
   private final RequiredFactoryField<HighLevelHumanoidControllerFactory> highLevelHumanoidControllerFactory = new RequiredFactoryField<>("highLevelHumanoidControllerFactory");
   private final RequiredFactoryField<CommonAvatarEnvironmentInterface> commonAvatarEnvironment = new RequiredFactoryField<>("commonAvatarEnvironmentInterface");
   private final RequiredFactoryField<DRCRobotInitialSetup<HumanoidFloatingRootJointRobot>> robotInitialSetup = new RequiredFactoryField<>("robotInitialSetup");
   private final RequiredFactoryField<DRCSCSInitialSetup> scsInitialSetup = new RequiredFactoryField<>("scsInitialSetup");
   private final RequiredFactoryField<DRCGuiInitialSetup> guiInitialSetup = new RequiredFactoryField<>("guiInitialSetup");
   private final RequiredFactoryField<RealtimeRos2Node> realtimeRos2Node = new RequiredFactoryField<>("realtimeRos2Node");

   private final OptionalFactoryField<Double> gravity = new OptionalFactoryField<>("gravity");
   private final OptionalFactoryField<Boolean> doSlowIntegrationForTorqueOffset = new OptionalFactoryField<>("doSlowIntegrationForTorqueOffset");
   private final OptionalFactoryField<Boolean> doSmoothJointTorquesAtControllerStateChanges = new OptionalFactoryField<>("doSmoothJointTorquesAtControllerStateChanges");
   private final OptionalFactoryField<Boolean> addActualCMPVisualization = new OptionalFactoryField<>("addActualCMPVisualization");
   private final OptionalFactoryField<Boolean> createCollisionMeshes = new OptionalFactoryField<>("createCollisionMeshes");
   private final OptionalFactoryField<Boolean> createYoVariableServer = new OptionalFactoryField<>("createYoVariableServer");
   private final OptionalFactoryField<PelvisPoseCorrectionCommunicatorInterface> externalPelvisCorrectorSubscriber = new OptionalFactoryField<>("externalPelvisCorrectorSubscriber");

   // TO CONSTRUCT
   private HumanoidFloatingRootJointRobot humanoidFloatingRootJointRobot;
   private YoVariableServer yoVariableServer;
   private SimulationConstructionSet simulationConstructionSet;
   private SensorReaderFactory sensorReaderFactory;
   private JointDesiredOutputWriter simulationOutputWriter;
   private DRCOutputProcessor simulationOutputProcessor;
   private AvatarEstimatorThread stateEstimationThread;
   private AvatarControllerThread controllerThread;
   private DisposableRobotController robotController;
   private SimulatedDRCRobotTimeProvider simulatedRobotTimeProvider;
   private ActualCMPComputer actualCMPComputer;
   private FullHumanoidRobotModel masterFullRobotModel;
   private HumanoidRobotContextData masterContext;

   private DRCRobotModelShapeCollisionSettings shapeCollisionSettings;

   private void createHumanoidFloatingRootJointRobot()
   {
      humanoidFloatingRootJointRobot = robotModel.get().createHumanoidFloatingRootJointRobot(createCollisionMeshes.get());
   }

   private void initializeCollisionManager()
   {
      if (shapeCollisionSettings != null && shapeCollisionSettings.useShapeCollision())
      {
         CollisionManager collisionManager;
         if (shapeCollisionSettings.useHybridImpulseHandler())
         {
            HybridImpulseSpringDamperCollisionHandler collisionHandler = new HybridImpulseSpringDamperCollisionHandler(shapeCollisionSettings.getRestitutionCoefficient(),
                                                                                                                       shapeCollisionSettings.getFrictionCoefficient(),
                                                                                                                       simulationConstructionSet.getRootRegistry(),
                                                                                                                       new YoGraphicsListRegistry());

            collisionHandler.setKp(shapeCollisionSettings.getHybridSpringCoefficient());
            collisionHandler.setKd(shapeCollisionSettings.getHybridDamperCoefficient());

            collisionManager = new CollisionManager(commonAvatarEnvironment.get().getTerrainObject3D(), collisionHandler);
         }
         else
         {
            DefaultCollisionHandler collisionHandler = new DefaultCollisionHandler(shapeCollisionSettings.getRestitutionCoefficient(),
                                                                                   shapeCollisionSettings.getFrictionCoefficient());
            collisionManager = new CollisionManager(commonAvatarEnvironment.get().getTerrainObject3D(), collisionHandler);
         }
         simulationConstructionSet.initializeShapeCollision(collisionManager);
      }
   }

   private void setupYoVariableServer()
   {
      if (createYoVariableServer.get())
      {
         yoVariableServer = new YoVariableServer(getClass(), robotModel.get().getLogModelProvider(), robotModel.get().getLogSettings(),
                                                 robotModel.get().getEstimatorDT());
      }
   }

   private void setupSimulationConstructionSet()
   {
      SimulationConstructionSetParameters simulationConstructionSetParameters = guiInitialSetup.get().getSimulationConstructionSetParameters();
      simulationConstructionSetParameters.setDataBufferSize(scsInitialSetup.get().getSimulationDataBufferSize());

      List<Robot> allSimulatedRobotList = new ArrayList<Robot>();
      allSimulatedRobotList.add(humanoidFloatingRootJointRobot);
      if (commonAvatarEnvironment.get() != null && commonAvatarEnvironment.get().getEnvironmentRobots() != null)
      {
         allSimulatedRobotList.addAll(commonAvatarEnvironment.get().getEnvironmentRobots());

         commonAvatarEnvironment.get().addContactPoints(humanoidFloatingRootJointRobot.getAllGroundContactPoints());
         commonAvatarEnvironment.get().createAndSetContactControllerToARobot();
      }

      simulationConstructionSet = new SimulationConstructionSet(allSimulatedRobotList.toArray(new Robot[0]), guiInitialSetup.get().getGraphics3DAdapter(),
                                                                simulationConstructionSetParameters);

      if (simulationConstructionSetParameters.getCreateGUI())
      {
         FrameIndexMap.FrameIndexFinder frameIndexMap = new FrameIndexMap.FrameIndexFinder(ReferenceFrame.getWorldFrame());
         AdditionalPanelTools.setupFrameView(simulationConstructionSet, frameIndexMap::getReferenceFrame, SCSVisualizer.createFrameFilter());
      }

      simulationConstructionSet.setDT(robotModel.get().getSimulateDT(), 1);
   }

   private void setupSensorReaderFactory()
   {
      StateEstimatorParameters stateEstimatorParameters = robotModel.get().getStateEstimatorParameters();
      if (scsInitialSetup.get().usePerfectSensors())
      {
         double estimatorDT = stateEstimatorParameters.getEstimatorDT();
         sensorReaderFactory = new DRCPerfectSensorReaderFactory(humanoidFloatingRootJointRobot, estimatorDT);
      }
      else
      {
         sensorReaderFactory = new SimulatedSensorHolderAndReaderFromRobotFactory(humanoidFloatingRootJointRobot, stateEstimatorParameters);
      }
   }

   private void setupSimulationOutputWriter()
   {
      simulationOutputWriter = robotModel.get().getCustomSimulationOutputWriter(humanoidFloatingRootJointRobot, masterContext);
   }

   private void setupSimulationOutputProcessor()
   {
      simulationOutputProcessor = robotModel.get().getCustomSimulationOutputProcessor(humanoidFloatingRootJointRobot);

      if (doSmoothJointTorquesAtControllerStateChanges.get())
      {
         DRCOutputProcessorWithStateChangeSmoother drcOutputWriterWithStateChangeSmoother = new DRCOutputProcessorWithStateChangeSmoother(simulationOutputProcessor);
         highLevelHumanoidControllerFactory.get()
                                           .attachControllerStateChangedListener(drcOutputWriterWithStateChangeSmoother.createControllerStateChangedListener());

         simulationOutputProcessor = drcOutputWriterWithStateChangeSmoother;
      }

      if (doSlowIntegrationForTorqueOffset.get())
      {
         DRCOutputProcessorWithTorqueOffsets outputWriterWithTorqueOffsets = new DRCOutputProcessorWithTorqueOffsets(simulationOutputProcessor,
                                                                                                                     robotModel.get().getControllerDT());
         simulationOutputProcessor = outputWriterWithTorqueOffsets;
      }
   }

   private void setupStateEstimationThread()
   {
      String robotName = robotModel.get().getSimpleRobotName();

      MessageTopicNameGenerator publisherTopicNameGenerator = ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      MessageTopicNameGenerator subscriberTopicNameGenerator = ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);

      PelvisPoseCorrectionCommunicatorInterface pelvisPoseCorrectionCommunicator;
      if (externalPelvisCorrectorSubscriber.hasValue())
      {
         pelvisPoseCorrectionCommunicator = externalPelvisCorrectorSubscriber.get();
      }
      else
      {
         pelvisPoseCorrectionCommunicator = new PelvisPoseCorrectionCommunicator(realtimeRos2Node.get(), publisherTopicNameGenerator);
         ROS2Tools.createCallbackSubscription(realtimeRos2Node.get(), StampedPosePacket.class, subscriberTopicNameGenerator,
                                              s -> pelvisPoseCorrectionCommunicator.receivedPacket(s.takeNextData()));
      }

      HumanoidRobotContextDataFactory contextDataFactory = new HumanoidRobotContextDataFactory();
      stateEstimationThread = new AvatarEstimatorThread(robotName, robotModel.get().getSensorInformation(), robotModel.get().getContactPointParameters(),
                                                        robotModel.get(), robotModel.get().getStateEstimatorParameters(), sensorReaderFactory,
                                                        contextDataFactory, realtimeRos2Node.get(), pelvisPoseCorrectionCommunicator, simulationOutputWriter,
                                                        gravity.get());
   }

   private void setupControllerThread()
   {
      String robotName = robotModel.get().getSimpleRobotName();
      HumanoidRobotContextDataFactory contextDataFactory = new HumanoidRobotContextDataFactory();
      controllerThread = new AvatarControllerThread(robotName, robotModel.get(), robotModel.get().getSensorInformation(),
                                                    highLevelHumanoidControllerFactory.get(), contextDataFactory, simulationOutputProcessor,
                                                    realtimeRos2Node.get(), gravity.get(), robotModel.get().getEstimatorDT());
   }

   private void createMasterContext()
   {
      // Create intermediate data buffer for threading.
      masterFullRobotModel = robotModel.get().createFullRobotModel();
      masterContext = new HumanoidRobotContextData(masterFullRobotModel);
   }

   private void setupMultiThreadedRobotController()
   {
      DRCRobotModel robotModel = this.robotModel.get();

      // Create the tasks that will be run on their own threads.
      int estimatorDivisor = (int) Math.round(robotModel.getEstimatorDT() / robotModel.getSimulateDT());
      int controllerDivisor = (int) Math.round(robotModel.getControllerDT() / robotModel.getSimulateDT());
      SimulationRobotVisualizer estimatorRobotVisualizer = new SimulationRobotVisualizer();
      SimulationRobotVisualizer controllerRobotVisualizer = new SimulationRobotVisualizer();
      RobotVisualizerList estimatorRobotVisualizerList = new RobotVisualizerList(estimatorRobotVisualizer, yoVariableServer);
      RobotVisualizerList controllerRobotVisualizerList = new RobotVisualizerList(controllerRobotVisualizer, yoVariableServer);
      HumanoidRobotControlTask estimatorTask = new EstimatorTask(stateEstimationThread, estimatorDivisor, masterFullRobotModel, estimatorRobotVisualizerList);
      HumanoidRobotControlTask controllerTask = new ControllerTask(controllerThread, controllerDivisor, masterFullRobotModel, controllerRobotVisualizerList);
      SimulatedHandControlTask handControlTask = robotModel.createSimulatedHandController(humanoidFloatingRootJointRobot, realtimeRos2Node.get());

      List<HumanoidRobotControlTask> tasks = new ArrayList<HumanoidRobotControlTask>();
      tasks.add(estimatorTask);
      tasks.add(controllerTask);
      if (handControlTask != null)
         tasks.add(handControlTask);

      // Create the controller that will run the tasks.
      String controllerName = "DRCSimulation";
      if (!scsInitialSetup.get().getRunMultiThreaded())
      {
         LogTools.warn("Running simulation in single threaded mode");
         robotController = new SingleThreadedRobotController<>(controllerName, tasks, masterContext);
      }
      else
      {
         TaskOverrunBehavior overrunBehavior = TaskOverrunBehavior.BUSY_WAIT;
         robotController = new BarrierScheduledRobotController<>(controllerName, tasks, masterContext, overrunBehavior);
      }

      // Add registry and graphics to SCS.
      robotController.getYoVariableRegistry().addChild(estimatorRobotVisualizer.getRegistry());
      if (estimatorRobotVisualizer.getGraphicsListRegistry() != null)
         simulationConstructionSet.addYoGraphicsListRegistry(estimatorRobotVisualizer.getGraphicsListRegistry());
      robotController.getYoVariableRegistry().addChild(controllerRobotVisualizer.getRegistry());
      if (controllerRobotVisualizer.getGraphicsListRegistry() != null)
         simulationConstructionSet.addYoGraphicsListRegistry(controllerRobotVisualizer.getGraphicsListRegistry());
      if (handControlTask != null)
         robotController.getYoVariableRegistry().addChild(handControlTask.getRegistry());
   }

   private void initializeStateEstimatorToActual()
   {
      if (scsInitialSetup.get().getInitializeEstimatorToActual())
      {
         LogTools.info("Initializing estimator to actual");

         /**
          * The following is to get the initial CoM position from the robot. It is cheating for now,
          * and we need to move to where the robot itself determines coordinates, and the sensors
          * are all in the robot-determined world coordinates..
          */
         robotInitialSetup.get().initializeRobot(humanoidFloatingRootJointRobot, robotModel.get().getJointMap());
         try
         {
            humanoidFloatingRootJointRobot.update();
            humanoidFloatingRootJointRobot.doDynamicsButDoNotIntegrate();
            humanoidFloatingRootJointRobot.update();
         }
         catch (UnreasonableAccelerationException e)
         {
            throw new RuntimeException("UnreasonableAccelerationException");
         }

         Point3D initialCoMPosition = new Point3D();
         humanoidFloatingRootJointRobot.computeCenterOfMass(initialCoMPosition);

         RigidBodyTransform rootJointTransform = humanoidFloatingRootJointRobot.getRootJoint().getJointTransform3D();

         TObjectDoubleMap<String> jointPositions = new TObjectDoubleHashMap<>();
         for (OneDegreeOfFreedomJoint joint : humanoidFloatingRootJointRobot.getOneDegreeOfFreedomJoints())
         {
            jointPositions.put(joint.getName(), joint.getQ());
         }

         stateEstimationThread.initializeEstimator(rootJointTransform, jointPositions);
      }
   }

   private void setupThreadedRobotController()
   {
      humanoidFloatingRootJointRobot.setController(robotController);
   }

   private void setupLidarController()
   {
      DRCRobotLidarParameters lidarParameters = robotModel.get().getSensorInformation().getLidarParameters(0);
      if (lidarParameters != null && lidarParameters.getLidarSpindleJointName() != null)
      {
         PIDLidarTorqueController pidLidarTorqueController = new PIDLidarTorqueController(humanoidFloatingRootJointRobot,
                                                                                          lidarParameters.getLidarSpindleJointName(),
                                                                                          lidarParameters.getLidarSpindleVelocity(),
                                                                                          robotModel.get().getSimulateDT());
         humanoidFloatingRootJointRobot.setController(pidLidarTorqueController);
      }
   }

   private void setupPositionControlledJointsForSimulation()
   {
      String[] positionControlledJointNames = robotModel.get().getJointMap().getPositionControlledJointsForSimulation();
      if (positionControlledJointNames != null)
      {
         for (String positionControlledJointName : positionControlledJointNames)
         {
            OneDegreeOfFreedomJoint simulatedJoint = humanoidFloatingRootJointRobot.getOneDegreeOfFreedomJoint(positionControlledJointName);
            FullRobotModel controllerFullRobotModel = controllerThread.getFullRobotModel();
            OneDoFJointBasics controllerJoint = controllerFullRobotModel.getOneDoFJointByName(positionControlledJointName);

            if (simulatedJoint == null || controllerJoint == null)
               continue;

            JointDesiredOutputListBasics controllerLowLevelDataList = controllerThread.getDesiredJointDataHolder();
            JointDesiredOutputBasics controllerDesiredOutput = controllerLowLevelDataList.getJointDesiredOutput(controllerJoint);

            JointRole jointRole = robotModel.get().getJointMap().getJointRole(positionControlledJointName);
            boolean isUpperBodyJoint = ((jointRole != JointRole.LEG) && (jointRole != JointRole.SPINE));
            boolean isBackJoint = jointRole == JointRole.SPINE;

            JointLowLevelJointControlSimulator positionControlSimulator = new JointLowLevelJointControlSimulator(simulatedJoint, controllerJoint,
                                                                                                                 controllerDesiredOutput, isUpperBodyJoint,
                                                                                                                 isBackJoint, false,
                                                                                                                 controllerFullRobotModel.getTotalMass(),
                                                                                                                 robotModel.get().getSimulateDT());
            humanoidFloatingRootJointRobot.setController(positionControlSimulator);
         }
      }
   }

   private void setupPassiveJoints()
   {
      YoVariableRegistry robotsYoVariableRegistry = humanoidFloatingRootJointRobot.getRobotsYoVariableRegistry();
      List<ImmutablePair<String, YoPDGains>> passiveJointNameWithGains = robotModel.get().getJointMap().getPassiveJointNameWithGains(robotsYoVariableRegistry);
      if (passiveJointNameWithGains != null)
      {
         for (int i = 0; i < passiveJointNameWithGains.size(); i++)
         {
            String jointName = passiveJointNameWithGains.get(i).getLeft();
            OneDegreeOfFreedomJoint simulatedJoint = humanoidFloatingRootJointRobot.getOneDegreeOfFreedomJoint(jointName);
            YoPDGains gains = passiveJointNameWithGains.get(i).getRight();
            PassiveJointController passiveJointController = new PassiveJointController(simulatedJoint, gains);
            humanoidFloatingRootJointRobot.setController(passiveJointController);
         }
      }
   }

   private void setupSimulatedRobotTimeProvider()
   {
      simulatedRobotTimeProvider = new SimulatedDRCRobotTimeProvider(robotModel.get().getSimulateDT());
      humanoidFloatingRootJointRobot.setController(simulatedRobotTimeProvider);
   }

   private void setupCMPVisualization()
   {
      actualCMPComputer = new ActualCMPComputer(addActualCMPVisualization.get(), simulationConstructionSet, humanoidFloatingRootJointRobot);
      if (addActualCMPVisualization.get())
      {
         humanoidFloatingRootJointRobot.setController(actualCMPComputer);
      }
   }

   private void setupCOMVisualization()
   {
      SimulatedRobotCenterOfMassVisualizer simulatedRobotCenterOfMassVisualizer = new SimulatedRobotCenterOfMassVisualizer(humanoidFloatingRootJointRobot,
                                                                                                                           robotModel.get().getSimulateDT());
      humanoidFloatingRootJointRobot.setController(simulatedRobotCenterOfMassVisualizer);
   }

   private void initializeSimulationConstructionSet()
   {
      simulationConstructionSet.setParameterRootPath(robotController.getYoVariableRegistry());

      humanoidFloatingRootJointRobot.setDynamicIntegrationMethod(scsInitialSetup.get().getDynamicIntegrationMethod());
      scsInitialSetup.get().initializeSimulation(simulationConstructionSet);

      if (guiInitialSetup.get().isGuiShown())
      {
         SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = simulationConstructionSet.createSimulationOverheadPlotterFactory();
         simulationOverheadPlotterFactory.setShowOnStart(guiInitialSetup.get().isShowOverheadView());
         simulationOverheadPlotterFactory.setVariableNameToTrack("centerOfMass");
         simulationOverheadPlotterFactory.addYoGraphicsListRegistries(controllerThread.getYoGraphicsListRegistry());
         simulationOverheadPlotterFactory.addYoGraphicsListRegistries(stateEstimationThread.getYoGraphicsListRegistry());
         simulationOverheadPlotterFactory.addYoGraphicsListRegistries(actualCMPComputer.getYoGraphicsListRegistry());
         simulationOverheadPlotterFactory.createOverheadPlotter();
         guiInitialSetup.get().initializeGUI(simulationConstructionSet, humanoidFloatingRootJointRobot, robotModel.get());
      }

      if (commonAvatarEnvironment.get() != null && commonAvatarEnvironment.get().getTerrainObject3D() != null)
      {
         simulationConstructionSet.addStaticLinkGraphics(commonAvatarEnvironment.get().getTerrainObject3D().getLinkGraphics());
      }

      scsInitialSetup.get().initializeRobot(humanoidFloatingRootJointRobot, robotModel.get(), null);
      robotInitialSetup.get().initializeRobot(humanoidFloatingRootJointRobot, robotModel.get().getJointMap());
      humanoidFloatingRootJointRobot.update();
   }

   public AvatarSimulation createAvatarSimulation()
   {
      gravity.setDefaultValue(-9.81);
      doSlowIntegrationForTorqueOffset.setDefaultValue(false);
      doSmoothJointTorquesAtControllerStateChanges.setDefaultValue(false);
      addActualCMPVisualization.setDefaultValue(true);
      createCollisionMeshes.setDefaultValue(false);
      createYoVariableServer.setDefaultValue(false);

      FactoryTools.checkAllFactoryFieldsAreSet(this);

      createHumanoidFloatingRootJointRobot();
      createMasterContext();
      setupYoVariableServer();
      setupSimulationConstructionSet();
      setupSensorReaderFactory();
      setupSimulationOutputWriter();
      setupSimulationOutputProcessor();
      setupStateEstimationThread();
      setupControllerThread();
      setupMultiThreadedRobotController();
      initializeStateEstimatorToActual();
      setupThreadedRobotController();
      setupLidarController();
      setupPositionControlledJointsForSimulation();
      setupPassiveJoints();
      setupSimulatedRobotTimeProvider();
      setupCMPVisualization();
      setupCOMVisualization();

      initializeCollisionManager();
      initializeSimulationConstructionSet();

      AvatarSimulation avatarSimulation = new AvatarSimulation();
      avatarSimulation.setSimulationConstructionSet(simulationConstructionSet);
      avatarSimulation.setHighLevelHumanoidControllerFactory(highLevelHumanoidControllerFactory.get());
      avatarSimulation.setYoVariableServer(yoVariableServer);
      avatarSimulation.setControllerThread(controllerThread);
      avatarSimulation.setStateEstimationThread(stateEstimationThread);
      avatarSimulation.setRobotController(robotController);
      avatarSimulation.setHumanoidFloatingRootJointRobot(humanoidFloatingRootJointRobot);
      avatarSimulation.setSimulatedRobotTimeProvider(simulatedRobotTimeProvider);
      avatarSimulation.setFullHumanoidRobotModel(controllerThread.getFullRobotModel());

      FactoryTools.disposeFactory(this);

      return avatarSimulation;
   }

   public void setRobotModel(DRCRobotModel robotModel)
   {
      this.robotModel.set(robotModel);
   }

   public void setHighLevelHumanoidControllerFactory(HighLevelHumanoidControllerFactory highLevelHumanoidControllerFactory)
   {
      this.highLevelHumanoidControllerFactory.set(highLevelHumanoidControllerFactory);
   }

   public void setCommonAvatarEnvironment(CommonAvatarEnvironmentInterface commonAvatarEnvironment)
   {
      this.commonAvatarEnvironment.set(commonAvatarEnvironment);
   }

   public void setRobotInitialSetup(DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup)
   {
      this.robotInitialSetup.set(robotInitialSetup);
   }

   public void setSCSInitialSetup(DRCSCSInitialSetup scsInitialSetup)
   {
      this.scsInitialSetup.set(scsInitialSetup);
   }

   public void setGuiInitialSetup(DRCGuiInitialSetup guiInitialSetup)
   {
      this.guiInitialSetup.set(guiInitialSetup);
   }

   public void setRealtimeRos2Node(RealtimeRos2Node realtimeRos2Node)
   {
      this.realtimeRos2Node.set(realtimeRos2Node);
   }

   public void setDoSlowIntegrationForTorqueOffset(boolean doSlowIntegrationForTorqueOffset)
   {
      this.doSlowIntegrationForTorqueOffset.set(doSlowIntegrationForTorqueOffset);
   }

   public void setDoSmoothJointTorquesAtControllerStateChanges(boolean doSmoothJointTorquesAtControllerStateChanges)
   {
      this.doSmoothJointTorquesAtControllerStateChanges.set(doSmoothJointTorquesAtControllerStateChanges);
   }

   public void setAddActualCMPVisualization(boolean addActualCMPVisualization)
   {
      this.addActualCMPVisualization.set(addActualCMPVisualization);
   }

   public void setCreateCollisionMeshes(boolean createCollisionMeshes)
   {
      this.createCollisionMeshes.set(createCollisionMeshes);
   }

   public void setCreateYoVariableServer(boolean createYoVariableServer)
   {
      this.createYoVariableServer.set(createYoVariableServer);
   }

   public void setGravity(double gravity)
   {
      this.gravity.set(gravity);
   }

   public void setShapeCollisionSettings(DRCRobotModelShapeCollisionSettings shapeCollisionSettings)
   {
      this.shapeCollisionSettings = shapeCollisionSettings;
   }

   public void setExternalPelvisCorrectorSubscriber(PelvisPoseCorrectionCommunicatorInterface externalPelvisCorrectorSubscriber)
   {
      this.externalPelvisCorrectorSubscriber.set(externalPelvisCorrectorSubscriber);
   }

}
