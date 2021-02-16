package us.ihmc.avatar.scs2;

import java.util.ArrayList;
import java.util.List;

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
import us.ihmc.avatar.factory.BarrierScheduledRobotController;
import us.ihmc.avatar.factory.DisposableRobotController;
import us.ihmc.avatar.factory.HumanoidRobotControlTask;
import us.ihmc.avatar.factory.RobotDefinitionTools;
import us.ihmc.avatar.factory.SimulatedHandControlTask;
import us.ihmc.avatar.factory.SingleThreadedRobotController;
import us.ihmc.avatar.factory.TerrainObjectDefinitionTools;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.avatar.kinematicsSimulation.IntraprocessYoVariableLogger;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextDataFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.BarrierScheduler.TaskOverrunBehavior;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicator;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.dataBuffers.RegistrySendBufferBuilder;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.physics.CollidableHelper;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;
import us.ihmc.sensorProcessing.parameters.AvatarRobotLidarParameters;
import us.ihmc.sensorProcessing.simulatedSensors.DRCPerfectSensorReaderFactory;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.sensorProcessing.simulatedSensors.SimulatedSensorHolderAndReaderFromRobotFactory;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationToolkit.controllers.PIDLidarTorqueController;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.factories.RequiredFactoryField;
import us.ihmc.wholeBodyController.DRCOutputProcessor;

public class SCS2AvatarSimulationFactory
{
   private static final ReferenceFrame inertialFrame = ReferenceFrame.getWorldFrame();

   private final RequiredFactoryField<DRCRobotModel> robotModel = new RequiredFactoryField<>("robotModel");
   private final RequiredFactoryField<HighLevelHumanoidControllerFactory> highLevelHumanoidControllerFactory = new RequiredFactoryField<>("highLevelHumanoidControllerFactory");
   private final RequiredFactoryField<TerrainObjectDefinition> terrainObjectDefinition = new RequiredFactoryField<>("terrainObjectDefinition");
   private final RequiredFactoryField<DRCRobotInitialSetup<HumanoidFloatingRootJointRobot>> robotInitialSetup = new RequiredFactoryField<>("robotInitialSetup");
   private final RequiredFactoryField<DRCSCSInitialSetup> scsInitialSetup = new RequiredFactoryField<>("scsInitialSetup");
   private final RequiredFactoryField<DRCGuiInitialSetup> guiInitialSetup = new RequiredFactoryField<>("guiInitialSetup");
   private final RequiredFactoryField<RealtimeROS2Node> realtimeROS2Node = new RequiredFactoryField<>("realtimeROS2Node");

   private final OptionalFactoryField<Double> gravity = new OptionalFactoryField<>("gravity");
   private final OptionalFactoryField<Boolean> createYoVariableServer = new OptionalFactoryField<>("createYoVariableServer");
   private final OptionalFactoryField<Boolean> logToFile = new OptionalFactoryField<>("logToFile");
   private final OptionalFactoryField<PelvisPoseCorrectionCommunicatorInterface> externalPelvisCorrectorSubscriber = new OptionalFactoryField<>("externalPelvisCorrectorSubscriber");

   // TO CONSTRUCT
   private RobotDefinition robotDefinition;
   private Robot robot;
   private YoVariableServer yoVariableServer;
   private IntraprocessYoVariableLogger intraprocessYoVariableLogger;
   private SimulationSession simulationSession;
   private SensorReaderFactory sensorReaderFactory;
   private JointDesiredOutputWriter simulationOutputWriter;
   private DRCOutputProcessor simulationOutputProcessor;
   private AvatarEstimatorThread estimatorThread;
   private AvatarControllerThread controllerThread;
   private DisposableRobotController robotController;
   private SimulatedDRCRobotTimeProvider simulatedRobotTimeProvider;
   private FullHumanoidRobotModel masterFullRobotModel;
   private HumanoidRobotContextData masterContext;

   private void createRobot()
   {
      robotDefinition = RobotDefinitionTools.toRobotDefinition(robotModel.get().getRobotDescription());
      RobotCollisionModel collisionModel = robotModel.get().getSimulationRobotCollisionModel(new CollidableHelper(), "robot", "terrain");
      if (collisionModel != null)
         RobotDefinitionTools.addCollisionsToRobotDefinition(collisionModel.getRobotCollidables(robotModel.get().createFullRobotModel().getElevator()),
                                                             robotDefinition);
      HumanoidFloatingRootJointRobot tempRobotForInitialState = robotModel.get().createHumanoidFloatingRootJointRobot(false);
      robotInitialSetup.get().initializeRobot(tempRobotForInitialState, robotModel.get().getJointMap());
      RobotDefinitionTools.addInitialStateToRobotDefinition(tempRobotForInitialState, robotDefinition);
      robot = new Robot(robotDefinition, inertialFrame);
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

      simulationSession = new SimulationSession();
      simulationSession.submitBufferSizeRequest(scsInitialSetup.get().getSimulationDataBufferSize());
      simulationSession.addTerrainObject(terrainObjectDefinition.get());
      RobotDefinition robotDefinition = RobotDefinitionTools.toRobotDefinition(robotModel.get().getRobotDescription());
      RobotCollisionModel collisionModel = robotModel.get().getSimulationRobotCollisionModel(new CollidableHelper(), "robot", "terrain");
      if (collisionModel != null)
         RobotDefinitionTools.addCollisionsToRobotDefinition(collisionModel.getRobotCollidables(robotModel.get().createFullRobotModel().getElevator()),
                                                             robotDefinition);
      HumanoidFloatingRootJointRobot tempRobotForInitialState = robotModel.get().createHumanoidFloatingRootJointRobot(false);
      robotInitialSetup.get().initializeRobot(tempRobotForInitialState, robotModel.get().getJointMap());
      RobotDefinitionTools.addInitialStateToRobotDefinition(tempRobotForInitialState, robotDefinition);
      simulationSession.addRobot(robotDefinition);
      simulationSession.setSessionTickToTimeIncrement(Conversions.secondsToNanoseconds(robotModel.get().getSimulateDT()));
   }

   private void setupSensorReaderFactory()
   {
      StateEstimatorParameters stateEstimatorParameters = robotModel.get().getStateEstimatorParameters();
      if (scsInitialSetup.get().usePerfectSensors())
      {
         double estimatorDT = stateEstimatorParameters.getEstimatorDT();
         sensorReaderFactory = new SCS2SensorReadFactory(robot.getControllerManager().getControllerInput(), scsInitialSetup.get().usePerfectSensors());
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
         builders.add(new RegistrySendBufferBuilder(controllerThread.getYoVariableRegistry(), controllerThread.getYoGraphicsListRegistry()));
         intraprocessYoVariableLogger = new IntraprocessYoVariableLogger(getClass().getSimpleName(),
                                                                         robotModel.getLogModelProvider(),
                                                                         builders,
                                                                         100000,
                                                                         robotModel.getEstimatorDT());
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

   private void setupSimulatedRobotTimeProvider()
   {
      simulatedRobotTimeProvider = new SimulatedDRCRobotTimeProvider(robotModel.get().getSimulateDT());
      humanoidFloatingRootJointRobot.setController(simulatedRobotTimeProvider);
   }

   private void initializeSimulationConstructionSet()
   {
      if (simulationConstructionSet != null)
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
   }

   public SCS2AvatarSimulation createAvatarSimulation()
   {
      gravity.setDefaultValue(-9.81);
      createYoVariableServer.setDefaultValue(false);

      FactoryTools.checkAllFactoryFieldsAreSet(this);

      createRobot();
      createMasterContext();
      setupYoVariableServer();
      setupSimulationConstructionSet();
      setupSensorReaderFactory();
      setupStateEstimationThread();
      setupControllerThread();
      setupMultiThreadedRobotController();
      initializeStateEstimatorToActual();
      setupThreadedRobotController();
      setupLidarController();
      setupPositionControlledJointsForSimulation();
      setupSimulatedRobotTimeProvider();

      initializeSimulationConstructionSet();

      SCS2AvatarSimulation avatarSimulation = new SCS2AvatarSimulation();
      avatarSimulation.setRobotModel(robotModel.get());
      avatarSimulation.setSimulationSession(simulationSession);
      avatarSimulation.setHighLevelHumanoidControllerFactory(highLevelHumanoidControllerFactory.get());
      avatarSimulation.setYoVariableServer(yoVariableServer);
      avatarSimulation.setIntraprocessYoVariableLogger(intraprocessYoVariableLogger);
      avatarSimulation.setControllerThread(controllerThread);
      avatarSimulation.setStateEstimationThread(estimatorThread);
      avatarSimulation.setRobotController(robotController);
      avatarSimulation.setRobotDefinition(robotDefinition);
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

   public void setTerrainObjectDefinition(TerrainObjectDefinition terrainObjectDefinition)
   {
      this.terrainObjectDefinition.set(terrainObjectDefinition);
   }

   public void setCommonAvatarEnvrionmentInterface(CommonAvatarEnvironmentInterface environment)
   {
      setTerrainObjectDefinition(TerrainObjectDefinitionTools.toTerrainObjectDefinition(environment));
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

   public void setExternalPelvisCorrectorSubscriber(PelvisPoseCorrectionCommunicatorInterface externalPelvisCorrectorSubscriber)
   {
      this.externalPelvisCorrectorSubscriber.set(externalPelvisCorrectorSubscriber);
   }
}
