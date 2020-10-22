package us.ihmc.avatar.factory;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.SystemUtils;
import org.apache.commons.lang3.tuple.ImmutablePair;

import controller_msgs.msg.dds.StampedPosePacket;
import gnu.trove.map.TObjectDoubleMap;
import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.avatar.AvatarControllerThread;
import us.ihmc.avatar.AvatarEstimatorThread;
import us.ihmc.avatar.AvatarEstimatorThreadFactory;
import us.ihmc.avatar.BarrierSchedulerTools;
import us.ihmc.avatar.ControllerTask;
import us.ihmc.avatar.EstimatorTask;
import us.ihmc.avatar.SimulationRobotVisualizer;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.SimulatedDRCRobotTimeProvider;
import us.ihmc.avatar.drcRobot.shapeContactSettings.DRCRobotModelShapeCollisionSettings;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.avatar.kinematicsSimulation.IntraprocessYoVariableLogger;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextDataFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.BarrierScheduler.TaskOverrunBehavior;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicator;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.dataBuffers.RegistrySendBufferBuilder;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPDGains;
import us.ihmc.robotics.physics.CollidableHelper;
import us.ihmc.robotics.physics.MultiBodySystemStateWriter;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;
import us.ihmc.sensorProcessing.parameters.AvatarRobotLidarParameters;
import us.ihmc.sensorProcessing.simulatedSensors.DRCPerfectSensorReaderFactory;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.sensorProcessing.simulatedSensors.SimulatedSensorHolderAndReaderFromRobotFactory;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationToolkit.controllers.ActualCMPComputer;
import us.ihmc.simulationToolkit.controllers.PIDLidarTorqueController;
import us.ihmc.simulationToolkit.controllers.PassiveJointController;
import us.ihmc.simulationToolkit.controllers.SimulatedRobotCenterOfMassVisualizer;
import us.ihmc.simulationToolkit.physicsEngine.ExperimentalSimulation;
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
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.factories.RequiredFactoryField;
import us.ihmc.tools.gui.AWTTools;
import us.ihmc.wholeBodyController.DRCOutputProcessor;
import us.ihmc.yoVariables.euclid.referenceFrame.interfaces.FrameIndexMap;
import us.ihmc.yoVariables.registry.YoRegistry;

public class AvatarSimulationFactory
{
   private final RequiredFactoryField<DRCRobotModel> robotModel = new RequiredFactoryField<>("robotModel");
   private final RequiredFactoryField<HighLevelHumanoidControllerFactory> highLevelHumanoidControllerFactory = new RequiredFactoryField<>("highLevelHumanoidControllerFactory");
   private final RequiredFactoryField<CommonAvatarEnvironmentInterface> commonAvatarEnvironment = new RequiredFactoryField<>("commonAvatarEnvironmentInterface");
   private final RequiredFactoryField<DRCRobotInitialSetup<HumanoidFloatingRootJointRobot>> robotInitialSetup = new RequiredFactoryField<>("robotInitialSetup");
   private final RequiredFactoryField<DRCSCSInitialSetup> scsInitialSetup = new RequiredFactoryField<>("scsInitialSetup");
   private final RequiredFactoryField<DRCGuiInitialSetup> guiInitialSetup = new RequiredFactoryField<>("guiInitialSetup");
   private final RequiredFactoryField<RealtimeROS2Node> realtimeROS2Node = new RequiredFactoryField<>("realtimeROS2Node");

   private final OptionalFactoryField<Double> gravity = new OptionalFactoryField<>("gravity");
   private final OptionalFactoryField<Boolean> addActualCMPVisualization = new OptionalFactoryField<>("addActualCMPVisualization");
   private final OptionalFactoryField<Boolean> createCollisionMeshes = new OptionalFactoryField<>("createCollisionMeshes");
   private final OptionalFactoryField<Boolean> createYoVariableServer = new OptionalFactoryField<>("createYoVariableServer");
   private final OptionalFactoryField<Boolean> logToFile = new OptionalFactoryField<>("logToFile");
   private final OptionalFactoryField<PelvisPoseCorrectionCommunicatorInterface> externalPelvisCorrectorSubscriber = new OptionalFactoryField<>("externalPelvisCorrectorSubscriber");

   // TO CONSTRUCT
   private HumanoidFloatingRootJointRobot humanoidFloatingRootJointRobot;
   private YoVariableServer yoVariableServer;
   private IntraprocessYoVariableLogger intraprocessYoVariableLogger;
   private SimulationConstructionSet simulationConstructionSet;
   private SensorReaderFactory sensorReaderFactory;
   private JointDesiredOutputWriter simulationOutputWriter;
   private DRCOutputProcessor simulationOutputProcessor;
   private AvatarEstimatorThread estimatorThread;
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
      robotInitialSetup.get().initializeRobot(humanoidFloatingRootJointRobot, robotModel.get().getJointMap());
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
         yoVariableServer = new YoVariableServer(getClass(),
                                                 robotModel.get().getLogModelProvider(),
                                                 robotModel.get().getLogSettings(),
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

      if (scsInitialSetup.get().getUseExperimentalPhysicsEngine())
      {
         ExperimentalSimulation experimentalSimulation = new ExperimentalSimulation(allSimulatedRobotList.toArray(new Robot[0]),
                                                                                    simulationConstructionSetParameters.getDataBufferSize());
         experimentalSimulation.setGravity(new Vector3D(0.0, 0.0, -Math.abs(gravity.get())));

         CollidableHelper helper = new CollidableHelper();
         String environmentCollisionMask = "ground";
         String robotCollisionMask = robotModel.get().getSimpleRobotName();
         MultiBodySystemStateWriter robotInitialStateWriter = ExperimentalSimulation.toRobotInitialStateWriter(robotInitialSetup.get()::initializeRobot,
                                                                                                               robotModel.get(),
                                                                                                               robotModel.get().getJointMap());
         experimentalSimulation.addEnvironmentCollidables(helper, robotCollisionMask, environmentCollisionMask, commonAvatarEnvironment.get());
         RobotCollisionModel simulationRobotCollisionModel = robotModel.get()
                                                                       .getSimulationRobotCollisionModel(helper, robotCollisionMask, environmentCollisionMask);
         experimentalSimulation.configureRobot(robotModel.get(), simulationRobotCollisionModel, robotInitialStateWriter);

         simulationConstructionSet = new SimulationConstructionSet(experimentalSimulation,
                                                                   guiInitialSetup.get().getGraphics3DAdapter(),
                                                                   simulationConstructionSetParameters);
         simulationConstructionSet.getRootRegistry().addChild(experimentalSimulation.getPhysicsEngineRegistry());
         simulationConstructionSet.addYoGraphicsListRegistry(experimentalSimulation.getPhysicsEngineGraphicsRegistry());
      }
      else
      {
         simulationConstructionSet = new SimulationConstructionSet(allSimulatedRobotList.toArray(new Robot[0]),
                                                                   guiInitialSetup.get().getGraphics3DAdapter(),
                                                                   simulationConstructionSetParameters);
      }

      if (simulationConstructionSetParameters.getCreateGUI())
      {
         FrameIndexMap.FrameIndexFinder frameIndexMap = new FrameIndexMap.FrameIndexFinder(ReferenceFrame.getWorldFrame());
         AdditionalPanelTools.setupFrameView(simulationConstructionSet, frameIndexMap::getReferenceFrame, SCSVisualizer.createFrameFilter());
      }

      simulationConstructionSet.setDT(robotModel.get().getSimulateDT(), 1);
      try
      {
         if (!SystemUtils.IS_OS_WINDOWS)
            simulationConstructionSet.getGUI().getFrame().setSize(AWTTools.getDimensionOfSmallestScreenScaled(2.0 / 3.0));
      }
      catch (NullPointerException npe)
      {
         // do nothing
      }
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
   }

   private void setupStateEstimationThread()
   {
      String robotName = robotModel.get().getSimpleRobotName();

      ROS2Topic outputTopic = ROS2Tools.getControllerOutputTopic(robotName);
      ROS2Topic inputTopic = ROS2Tools.getControllerInputTopic(robotName);

      PelvisPoseCorrectionCommunicatorInterface pelvisPoseCorrectionCommunicator;
      if (externalPelvisCorrectorSubscriber.hasValue())
      {
         pelvisPoseCorrectionCommunicator = externalPelvisCorrectorSubscriber.get();
      }
      else
      {
         pelvisPoseCorrectionCommunicator = new PelvisPoseCorrectionCommunicator(realtimeROS2Node.get(), outputTopic);
         ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeROS2Node.get(),
                                                       StampedPosePacket.class,
                                                       inputTopic,
                                                       s -> pelvisPoseCorrectionCommunicator.receivedPacket(s.takeNextData()));
      }

      HumanoidRobotContextDataFactory contextDataFactory = new HumanoidRobotContextDataFactory();
      AvatarEstimatorThreadFactory avatarEstimatorThreadFactory = new AvatarEstimatorThreadFactory();
      avatarEstimatorThreadFactory.setROS2Info(realtimeROS2Node.get(), robotName);
      avatarEstimatorThreadFactory.configureWithDRCRobotModel(robotModel.get(), robotInitialSetup.get());
      avatarEstimatorThreadFactory.setSensorReaderFactory(sensorReaderFactory);
      avatarEstimatorThreadFactory.setHumanoidRobotContextDataFactory(contextDataFactory);
      avatarEstimatorThreadFactory.setExternalPelvisCorrectorSubscriber(pelvisPoseCorrectionCommunicator);
      avatarEstimatorThreadFactory.setJointDesiredOutputWriter(simulationOutputWriter);
      avatarEstimatorThreadFactory.setGravity(gravity.get());
      estimatorThread = avatarEstimatorThreadFactory.createAvatarEstimatorThread();
   }

   private void setupControllerThread()
   {
      String robotName = robotModel.get().getSimpleRobotName();
      HumanoidRobotContextDataFactory contextDataFactory = new HumanoidRobotContextDataFactory();
      controllerThread = new AvatarControllerThread(robotName,
                                                    robotModel.get(),
                                                    robotInitialSetup.get(),
                                                    robotModel.get().getSensorInformation(),
                                                    highLevelHumanoidControllerFactory.get(),
                                                    contextDataFactory,
                                                    simulationOutputProcessor,
                                                    realtimeROS2Node.get(),
                                                    gravity.get(),
                                                    robotModel.get().getEstimatorDT());
   }

   private void createMasterContext()
   {
      // Create intermediate data buffer for threading.
      masterFullRobotModel = robotModel.get().createFullRobotModel();
      robotInitialSetup.get().initializeFullRobotModel(masterFullRobotModel);
      masterContext = new HumanoidRobotContextData(masterFullRobotModel);
   }

   private void setupMultiThreadedRobotController()
   {
      DRCRobotModel robotModel = this.robotModel.get();

      // Create the tasks that will be run on their own threads.
      int estimatorDivisor = (int) Math.round(robotModel.getEstimatorDT() / robotModel.getSimulateDT());
      int controllerDivisor = (int) Math.round(robotModel.getControllerDT() / robotModel.getSimulateDT());
      HumanoidRobotControlTask estimatorTask = new EstimatorTask(estimatorThread, estimatorDivisor, robotModel.getSimulateDT(), masterFullRobotModel);
      HumanoidRobotControlTask controllerTask = new ControllerTask(controllerThread, controllerDivisor, robotModel.getSimulateDT(), masterFullRobotModel);
      SimulatedHandControlTask handControlTask = robotModel.createSimulatedHandController(humanoidFloatingRootJointRobot, realtimeROS2Node.get());

      // Previously done in estimator thread write
      if (simulationOutputWriter != null)
      {
         estimatorTask.addRunnableOnSchedulerThread(() ->
         {
            if (estimatorThread.getHumanoidRobotContextData().getControllerRan())
               simulationOutputWriter.writeAfter();
         });
      }
      // Previously done in estimator thread read
      SensorReader sensorReader = estimatorThread.getSensorReader();
      estimatorTask.addRunnableOnSchedulerThread(() ->
      {
         long newTimestamp = sensorReader.read(masterContext.getSensorDataContext());
         masterContext.setTimestamp(newTimestamp);
      });
      if (simulationOutputWriter != null)
      {
         estimatorTask.addRunnableOnSchedulerThread(() ->
         {
            if (estimatorThread.getHumanoidRobotContextData().getControllerRan())
               simulationOutputWriter.writeBefore(estimatorThread.getHumanoidRobotContextData().getTimestamp());
         });
      }
      // Previously done in controller thread write
      if (simulationOutputProcessor != null)
      {
         controllerTask.addRunnableOnSchedulerThread(BarrierSchedulerTools.createProcessorUpdater(simulationOutputProcessor, controllerThread));
      }

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
         robotController = new BarrierScheduledRobotController(controllerName, tasks, masterContext, overrunBehavior, robotModel.getSimulateDT());
         tasks.forEach(task -> new Thread(task, task.getClass().getSimpleName() + "Thread").start());
      }

      if (logToFile.hasValue() && logToFile.get())
      {
         ArrayList<RegistrySendBufferBuilder> builders = new ArrayList<>();
         builders.add(new RegistrySendBufferBuilder(estimatorThread.getYoRegistry(), estimatorThread.getFullRobotModel().getElevator(), null));
         builders.add(new RegistrySendBufferBuilder(controllerThread.getYoVariableRegistry(), null, controllerThread.getYoGraphicsListRegistry()));
         intraprocessYoVariableLogger = new IntraprocessYoVariableLogger(getClass().getSimpleName(),
                                                                         robotModel.getLogModelProvider(),
                                                                         builders,
                                                                         100000,
                                                                         robotModel.getEstimatorDT(),
                                                                         IntraprocessYoVariableLogger.DEFAULT_INCOMING_LOGS_DIRECTORY);
         estimatorTask.addRunnableOnTaskThread(() -> intraprocessYoVariableLogger.update(estimatorThread.getHumanoidRobotContextData().getTimestamp()));
      }

      // If running with server setup the server registries and their updates.
      if (yoVariableServer != null)
      {
         yoVariableServer.setMainRegistry(estimatorThread.getYoRegistry(),
                                          estimatorThread.getFullRobotModel().getElevator(),
                                          estimatorThread.getYoGraphicsListRegistry());
         estimatorTask.addRunnableOnTaskThread(() -> yoVariableServer.update(estimatorThread.getHumanoidRobotContextData().getTimestamp(),
                                                                             estimatorThread.getYoRegistry()));

         yoVariableServer.addRegistry(controllerThread.getYoVariableRegistry(), controllerThread.getYoGraphicsListRegistry());
         controllerTask.addRunnableOnTaskThread(() -> yoVariableServer.update(controllerThread.getHumanoidRobotContextData().getTimestamp(),
                                                                              controllerThread.getYoVariableRegistry()));
      }

      // Add registry and graphics to SCS.
      SimulationRobotVisualizer estimatorRobotVisualizer = new SimulationRobotVisualizer(estimatorThread.getYoRegistry(),
                                                                                         estimatorThread.getYoGraphicsListRegistry());
      SimulationRobotVisualizer controllerRobotVisualizer = new SimulationRobotVisualizer(controllerThread.getYoVariableRegistry(),
                                                                                          controllerThread.getYoGraphicsListRegistry());
      estimatorTask.addRunnableOnSchedulerThread(() -> estimatorRobotVisualizer.update());
      controllerTask.addRunnableOnSchedulerThread(() -> controllerRobotVisualizer.update());
      addRegistryAndGraphics(estimatorRobotVisualizer, robotController.getYoRegistry(), simulationConstructionSet);
      addRegistryAndGraphics(controllerRobotVisualizer, robotController.getYoRegistry(), simulationConstructionSet);
      if (handControlTask != null)
         robotController.getYoRegistry().addChild(handControlTask.getRegistry());
   }

   private static void addRegistryAndGraphics(SimulationRobotVisualizer visualizer, YoRegistry registry, SimulationConstructionSet scs)
   {
      registry.addChild(visualizer.getRegistry());
      if (visualizer.getGraphicsListRegistry() != null)
      {
         scs.attachSimulationRewoundListener(() -> visualizer.updateGraphics());
         scs.attachPlayCycleListener(tick -> visualizer.updateGraphics());
         scs.addYoGraphicsListRegistry(visualizer.getGraphicsListRegistry(), false);
      }
   }

   private void initializeStateEstimatorToActual()
   {
      if (scsInitialSetup.get().getInitializeEstimatorToActual())
      {
         LogTools.info("Initializing estimator to actual");

         /**
          * The following is to get the initial CoM position from the robot. It is cheating for now, and we
          * need to move to where the robot itself determines coordinates, and the sensors are all in the
          * robot-determined world coordinates..
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

         initializeEstimator(humanoidFloatingRootJointRobot, estimatorThread);
      }
   }

   public static void initializeEstimator(HumanoidFloatingRootJointRobot humanoidFloatingRootJointRobot, AvatarEstimatorThread estimatorThread)
   {
      RigidBodyTransform rootJointTransform = humanoidFloatingRootJointRobot.getRootJoint().getJointTransform3D();

      TObjectDoubleMap<String> jointPositions = new TObjectDoubleHashMap<>();
      for (OneDegreeOfFreedomJoint joint : humanoidFloatingRootJointRobot.getOneDegreeOfFreedomJoints())
      {
         jointPositions.put(joint.getName(), joint.getQ());
      }

      estimatorThread.initializeStateEstimators(rootJointTransform, jointPositions);
   }

   private void setupThreadedRobotController()
   {
      humanoidFloatingRootJointRobot.setController(robotController);
   }

   private void setupLidarController()
   {
      AvatarRobotLidarParameters lidarParameters = robotModel.get().getSensorInformation().getLidarParameters(0);
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
      RobotController lowLevelController = robotModel.get().getSimulationLowLevelControllerFactory()
                                                     .createLowLevelController(controllerThread.getFullRobotModel(),
                                                                               humanoidFloatingRootJointRobot,
                                                                               controllerThread.getDesiredJointDataHolder());
      if (lowLevelController != null)
         humanoidFloatingRootJointRobot.setController(lowLevelController);
   }

   private void setupPassiveJoints()
   {
      YoRegistry robotsYoVariableRegistry = humanoidFloatingRootJointRobot.getRobotsYoRegistry();
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
      simulationConstructionSet.setParameterRootPath(robotController.getYoRegistry());

      humanoidFloatingRootJointRobot.setDynamicIntegrationMethod(scsInitialSetup.get().getDynamicIntegrationMethod());
      scsInitialSetup.get().initializeSimulation(simulationConstructionSet);

      if (guiInitialSetup.get().isGuiShown())
      {
         SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = simulationConstructionSet.createSimulationOverheadPlotterFactory();
         simulationOverheadPlotterFactory.setShowOnStart(guiInitialSetup.get().isShowOverheadView());
         simulationOverheadPlotterFactory.setVariableNameToTrack("centerOfMass");
         simulationOverheadPlotterFactory.addYoGraphicsListRegistries(controllerThread.getYoGraphicsListRegistry());
         simulationOverheadPlotterFactory.addYoGraphicsListRegistries(estimatorThread.getYoGraphicsListRegistry());
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
      avatarSimulation.setRobotInitialSetup(robotInitialSetup.get());
      avatarSimulation.setRobotModel(robotModel.get());
      avatarSimulation.setSimulationConstructionSet(simulationConstructionSet);
      avatarSimulation.setHighLevelHumanoidControllerFactory(highLevelHumanoidControllerFactory.get());
      avatarSimulation.setYoVariableServer(yoVariableServer);
      avatarSimulation.setIntraprocessYoVariableLogger(intraprocessYoVariableLogger);
      avatarSimulation.setControllerThread(controllerThread);
      avatarSimulation.setStateEstimationThread(estimatorThread);
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

   public void setRealtimeROS2Node(RealtimeROS2Node realtimeROS2Node)
   {
      this.realtimeROS2Node.set(realtimeROS2Node);
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

   public void setLogToFile(boolean logToFile)
   {
      this.logToFile.set(logToFile);
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
