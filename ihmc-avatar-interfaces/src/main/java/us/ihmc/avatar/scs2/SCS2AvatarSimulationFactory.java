package us.ihmc.avatar.scs2;

import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.DO_NOTHING_BEHAVIOR;
import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.WALKING;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import controller_msgs.msg.dds.StampedPosePacket;
import gnu.trove.map.TObjectDoubleMap;
import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.avatar.AvatarControllerThread;
import us.ihmc.avatar.AvatarEstimatorThread;
import us.ihmc.avatar.AvatarEstimatorThreadFactory;
import us.ihmc.avatar.AvatarSimulatedHandControlThread;
import us.ihmc.avatar.ControllerTask;
import us.ihmc.avatar.EstimatorTask;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.SimulatedDRCRobotTimeProvider;
import us.ihmc.avatar.factory.BarrierScheduledRobotController;
import us.ihmc.avatar.factory.DisposableRobotController;
import us.ihmc.avatar.factory.HumanoidRobotControlTask;
import us.ihmc.avatar.factory.RobotDefinitionTools;
import us.ihmc.avatar.factory.SimulatedHandControlTask;
import us.ihmc.avatar.factory.SimulatedHandOutputWriter;
import us.ihmc.avatar.factory.SimulatedHandSensorReader;
import us.ihmc.avatar.factory.SingleThreadedRobotController;
import us.ihmc.avatar.factory.TerrainObjectDefinitionTools;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.initialSetup.RobotInitialSetup;
import us.ihmc.avatar.logging.IntraprocessYoVariableLogger;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextDataFactory;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControllerParameters;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.BarrierScheduler.TaskOverrunBehavior;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicator;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.dataBuffers.RegistrySendBufferBuilder;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.physics.CollidableHelper;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.tools.SCS1GraphicConversionTools;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.parameters.ContactPointBasedContactParameters;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;
import us.ihmc.scs2.simulation.physicsEngine.contactPointBased.ContactPointBasedPhysicsEngine;
import us.ihmc.scs2.simulation.physicsEngine.impulseBased.ImpulseBasedPhysicsEngine;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.controller.SimControllerInput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;
import us.ihmc.sensorProcessing.parameters.AvatarRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationconstructionset.dataBuffer.MirroredYoVariableRegistry;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.factories.RequiredFactoryField;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.RobotContactPointParameters.GroundContactModelParameters;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SCS2AvatarSimulationFactory
{
   private final RequiredFactoryField<DRCRobotModel> robotModel = new RequiredFactoryField<>("robotModel");
   private final RequiredFactoryField<HighLevelHumanoidControllerFactory> highLevelHumanoidControllerFactory = new RequiredFactoryField<>("highLevelHumanoidControllerFactory");
   private final RequiredFactoryField<TerrainObjectDefinition> terrainObjectDefinition = new RequiredFactoryField<>("terrainObjectDefinition");
   private final RequiredFactoryField<RealtimeROS2Node> realtimeROS2Node = new RequiredFactoryField<>("realtimeROS2Node");

   private final OptionalFactoryField<Double> simulationDT = new OptionalFactoryField<>("simulationDT");
   private final OptionalFactoryField<RobotInitialSetup<HumanoidFloatingRootJointRobot>> robotInitialSetup = new OptionalFactoryField<>("robotInitialSetup");
   private final OptionalFactoryField<Double> gravity = new OptionalFactoryField<>("gravity", -9.81);
   private final OptionalFactoryField<Boolean> createYoVariableServer = new OptionalFactoryField<>("createYoVariableServer", false);
   private final OptionalFactoryField<Boolean> logToFile = new OptionalFactoryField<>("logToFile", false);
   private final OptionalFactoryField<PelvisPoseCorrectionCommunicatorInterface> externalPelvisCorrectorSubscriber = new OptionalFactoryField<>("externalPelvisCorrectorSubscriber");
   private final OptionalFactoryField<Integer> simulationDataBufferSize = new OptionalFactoryField<>("simulationDataBufferSize", 8192);
   private final OptionalFactoryField<Integer> simulationDataRecordTickPeriod = new OptionalFactoryField<>("simulationDataRecordTickPeriod");
   private final OptionalFactoryField<Boolean> usePerfectSensors = new OptionalFactoryField<>("usePerfectSensors", false);
   private final OptionalFactoryField<SCS2JointDesiredOutputWriterFactory> outputWriterFactory = new OptionalFactoryField<>("outputWriterFactory",
                                                                                                                            (in,
                                                                                                                             out) -> new SCS2OutputWriter(in,
                                                                                                                                                          out,
                                                                                                                                                          true));
   private final OptionalFactoryField<Boolean> runMultiThreaded = new OptionalFactoryField<>("runMultiThreaded", true);
   private final OptionalFactoryField<Boolean> initializeEstimatorToActual = new OptionalFactoryField<>("initializeEstimatorToActual", true);
   private final OptionalFactoryField<Boolean> showGUI = new OptionalFactoryField<>("showGUI", true);
   private final OptionalFactoryField<Boolean> automaticallyStartSimulation = new OptionalFactoryField<>("automaticallyStartSimulation", false);

   private final OptionalFactoryField<Boolean> useImpulseBasePhysicsEngine = new OptionalFactoryField<>("useImpulseBasePhysicsEngine", false);
   private final OptionalFactoryField<Boolean> enableSimulatedRobotDamping = new OptionalFactoryField<>("enableSimulatedRobotDamping", true);
   private final OptionalFactoryField<List<Robot>> secondaryRobots = new OptionalFactoryField<>("secondaryRobots", new ArrayList<>());
   private final OptionalFactoryField<String> simulationName = new OptionalFactoryField<>("simulationName");

   // TO CONSTRUCT
   private RobotDefinition robotDefinition;
   private Robot robot;
   private YoVariableServer yoVariableServer;
   private IntraprocessYoVariableLogger intraprocessYoVariableLogger;
   private SimulationSession simulationSession;
   private JointDesiredOutputWriter simulationOutputWriter;
   private AvatarEstimatorThread estimatorThread;
   private AvatarControllerThread controllerThread;
   private DisposableRobotController robotController;
   private SimulatedDRCRobotTimeProvider simulatedRobotTimeProvider;

   private final CollidableHelper collidableHelper = new CollidableHelper();
   private final String robotCollisionName = "robot";
   private final String terrainCollisionName = "terrain";

   public SCS2AvatarSimulation createAvatarSimulation()
   {
      simulationDataRecordTickPeriod.setDefaultValue((int) Math.max(1.0, robotModel.get().getControllerDT() / simulationDT.get()));

      FactoryTools.checkAllFactoryFieldsAreSet(this);

      setupSimulationConstructionSet();
      setupYoVariableServer();
      setupSimulationOutputWriter();
      setupStateEstimationThread();
      setupControllerThread();
      setupMultiThreadedRobotController();
      setupLidarController();
      initializeStateEstimatorToActual();
      setupSimulatedRobotTimeProvider();

      SCS2AvatarSimulation avatarSimulation = new SCS2AvatarSimulation();
      avatarSimulation.setRobotModel(robotModel.get());
      avatarSimulation.setSimulationSession(simulationSession);
      avatarSimulation.setHighLevelHumanoidControllerFactory(highLevelHumanoidControllerFactory.get());
      avatarSimulation.setYoVariableServer(yoVariableServer);
      avatarSimulation.setIntraprocessYoVariableLogger(intraprocessYoVariableLogger);
      avatarSimulation.setControllerThread(controllerThread);
      avatarSimulation.setEstimatorThread(estimatorThread);
      avatarSimulation.setRobotController(robotController);
      avatarSimulation.setRobot(robot);
      avatarSimulation.setSimulatedRobotTimeProvider(simulatedRobotTimeProvider);
      avatarSimulation.setFullHumanoidRobotModel(controllerThread.getFullRobotModel());
      avatarSimulation.setShowGUI(showGUI.get());
      avatarSimulation.setAutomaticallyStartSimulation(automaticallyStartSimulation.get());
      avatarSimulation.setRealTimeROS2Node(realtimeROS2Node.get());

      FactoryTools.disposeFactory(this);

      return avatarSimulation;
   }

   private void setupSimulationConstructionSet()
   {
      DRCRobotModel robotModel = this.robotModel.get();

      robotDefinition = robotModel.getRobotDefinition();

      if (!enableSimulatedRobotDamping.get())
      {
         for (JointDefinition joint : robotDefinition.getAllJoints())
         {
            if (joint instanceof OneDoFJointDefinition)
            {
               ((OneDoFJointDefinition) joint).setDamping(0.0);
            }
         }
      }

      RobotCollisionModel collisionModel = robotModel.getSimulationRobotCollisionModel(collidableHelper, robotCollisionName, terrainCollisionName);
      if (collisionModel != null)
         RobotDefinitionTools.addCollisionsToRobotDefinition(collisionModel.getRobotCollidables(robotModel.createFullRobotModel().getElevator()),
                                                             robotDefinition);
      robotInitialSetup.get().initializeRobotDefinition(robotDefinition);
      Set<String> lastSimulatedJoints = robotModel.getJointMap().getLastSimulatedJoints();
      lastSimulatedJoints.forEach(lastSimulatedJoint -> robotDefinition.addSubtreeJointsToIgnore(lastSimulatedJoint));

      PhysicsEngineFactory physicsEngineFactory;

      if (useImpulseBasePhysicsEngine.hasValue() && useImpulseBasePhysicsEngine.get())
      {
         physicsEngineFactory = (inertialFrame, rootRegistry) -> new ImpulseBasedPhysicsEngine(inertialFrame, rootRegistry);
      }
      else
      {
         physicsEngineFactory = (inertialFrame, rootRegistry) ->
         {
            ContactPointBasedPhysicsEngine physicsEngine = new ContactPointBasedPhysicsEngine(inertialFrame, rootRegistry);
            GroundContactModelParameters contactModelParameters = robotModel.getContactPointParameters().getContactModelParameters(robotModel.getSimulateDT());
            ContactPointBasedContactParameters parameters = ContactPointBasedContactParameters.defaultParameters();
            parameters.setKz(contactModelParameters.getZStiffness());
            parameters.setBz(contactModelParameters.getZDamping());
            parameters.setKxy(contactModelParameters.getXYStiffness());
            parameters.setBxy(contactModelParameters.getXYDamping());
            physicsEngine.setGroundContactParameters(parameters);
            return physicsEngine;
         };
      }

      String name = simulationName.hasValue() ? simulationName.get() : Session.retrieveCallerName();
      simulationSession = new SimulationSession(name, physicsEngineFactory);
      simulationSession.initializeBufferSize(simulationDataBufferSize.get());
      simulationSession.initializeBufferRecordTickPeriod(simulationDataRecordTickPeriod.get());
      simulationSession.addTerrainObject(terrainObjectDefinition.get());
      robot = simulationSession.addRobot(robotDefinition);
      robot.getControllerManager()
           .addController(new SCS2StateEstimatorDebugVariables(simulationSession.getInertialFrame(),
                                                               gravity.get(),
                                                               robot.getControllerManager().getControllerInput()));

      for (Robot secondaryRobot : secondaryRobots.get())
         simulationSession.addRobot(secondaryRobot);

      simulationSession.setSessionDTSeconds(simulationDT.get());
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

   private void setupSimulationOutputWriter()
   {
      simulationOutputWriter = outputWriterFactory.get().build(robot.getControllerManager().getControllerInput(),
                                                               robot.getControllerManager().getControllerOutput());
   }

   private void setupStateEstimationThread()
   {
      String robotName = robotModel.get().getSimpleRobotName();

      StateEstimatorParameters stateEstimatorParameters = robotModel.get().getStateEstimatorParameters();
      SimControllerInput controllerInput = robot.getControllerManager().getControllerInput();
      SCS2SensorReaderFactory sensorReaderFactory;
      if (usePerfectSensors.get())
         sensorReaderFactory = SCS2SensorReaderFactory.newPerfectSensorReaderFactory(controllerInput);
      else
         sensorReaderFactory = SCS2SensorReaderFactory.newSensorReaderFactory(controllerInput, stateEstimatorParameters);

      ROS2Topic<?> outputTopic = ROS2Tools.getControllerOutputTopic(robotName);
      ROS2Topic<?> inputTopic = ROS2Tools.getControllerInputTopic(robotName);

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
                                                    null,
                                                    realtimeROS2Node.get(),
                                                    gravity.get(),
                                                    robotModel.get().getEstimatorDT());
      simulationSession.addYoGraphicDefinitions(SCS1GraphicConversionTools.toYoGraphicDefinitions(controllerThread.getYoGraphicsListRegistry()));
   }

   private void setupMultiThreadedRobotController()
   {
      DRCRobotModel robotModel = this.robotModel.get();

      // Create intermediate data buffer for threading.
      FullHumanoidRobotModel masterFullRobotModel = robotModel.createFullRobotModel();
      robotInitialSetup.get().initializeFullRobotModel(masterFullRobotModel);
      HumanoidRobotContextData masterContext = new HumanoidRobotContextData(masterFullRobotModel);

      // Create the tasks that will be run on their own threads.
      int estimatorDivisor = (int) Math.round(robotModel.getEstimatorDT() / simulationDT.get());
      int controllerDivisor = (int) Math.round(robotModel.getControllerDT() / simulationDT.get());
      int handControlDivisor = (int) Math.round(robotModel.getSimulatedHandControlDT() / simulationDT.get());
      HumanoidRobotControlTask estimatorTask = new EstimatorTask(estimatorThread, estimatorDivisor, simulationDT.get(), masterFullRobotModel);
      HumanoidRobotControlTask controllerTask = new ControllerTask(controllerThread, controllerDivisor, simulationDT.get(), masterFullRobotModel);

      AvatarSimulatedHandControlThread handControlThread = robotModel.createSimulatedHandController(realtimeROS2Node.get());
      SimulatedHandControlTask handControlTask = null;

      if (handControlThread != null)
      {
         List<String> fingerJointNames = handControlThread.getControlledOneDoFJoints().stream().map(JointReadOnly::getName).collect(Collectors.toList());
         SimulatedHandSensorReader handSensorReader = new SCS2SimulatedHandSensorReader(robot.getControllerManager().getControllerInput(), fingerJointNames);
         SimulatedHandOutputWriter handOutputWriter = new SCS2SimulatedHandOutputWriter(robot.getControllerManager().getControllerInput(),
                                                                                        robot.getControllerManager().getControllerOutput());
         handControlTask = new SimulatedHandControlTask(handSensorReader, handControlThread, handOutputWriter, handControlDivisor, simulationDT.get());
      }

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

      List<HumanoidRobotControlTask> tasks = new ArrayList<>();
      tasks.add(estimatorTask);
      tasks.add(controllerTask);
      if (handControlTask != null)
         tasks.add(handControlTask);

      // Create the controller that will run the tasks.
      String controllerName = "DRCSimulation";
      if (!runMultiThreaded.get())
      {
         LogTools.warn("Running simulation in single threaded mode");
         robotController = new SingleThreadedRobotController<>(controllerName, tasks, masterContext);
      }
      else
      {
         TaskOverrunBehavior overrunBehavior = TaskOverrunBehavior.BUSY_WAIT;
         robotController = new BarrierScheduledRobotController(controllerName, tasks, masterContext, overrunBehavior, simulationDT.get());
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

      setupWithMirroredRegistry(estimatorThread.getYoRegistry(), estimatorTask, robotController.getYoRegistry());
      setupWithMirroredRegistry(controllerThread.getYoVariableRegistry(), controllerTask, robotController.getYoRegistry());
      if (handControlThread != null)
         setupWithMirroredRegistry(handControlThread.getYoVariableRegistry(), handControlTask, robotController.getYoRegistry());
      robot.getRegistry().addChild(robotController.getYoRegistry());
      robot.getControllerManager().addController(new Controller()
      {
         @Override
         public void doControl()
         {
            robotController.doControl();
         }

         @Override
         public void pause()
         {
            if (robotController instanceof BarrierScheduledRobotController)
               ((BarrierScheduledRobotController) robotController).waitUntilTasksDone();
         }
      });
   }

   private static void setupWithMirroredRegistry(YoRegistry registry, HumanoidRobotControlTask owner, YoRegistry schedulerRegistry)
   {
      MirroredYoVariableRegistry mirroredRegistry = new MirroredYoVariableRegistry(registry);
      owner.addRunnableOnSchedulerThread(() -> mirroredRegistry.updateMirror());
      schedulerRegistry.addChild(mirroredRegistry);
   }

   private void setupLidarController()
   {
      AvatarRobotLidarParameters lidarParameters = robotModel.get().getSensorInformation().getLidarParameters(0);
      if (lidarParameters != null && lidarParameters.getLidarSpindleJointName() != null)
      {
         ControllerInput controllerInput = robot.getControllerManager().getControllerInput();
         ControllerOutput controllerOutput = robot.getControllerManager().getControllerOutput();
         robot.getControllerManager()
              .addController(new SCS2PIDLidarTorqueController(controllerInput,
                                                              controllerOutput,
                                                              lidarParameters.getLidarSpindleJointName(),
                                                              lidarParameters.getLidarSpindleVelocity(),
                                                              simulationDT.get()));
      }
   }

   private void initializeStateEstimatorToActual()
   {
      if (initializeEstimatorToActual.get())
      {
         LogTools.info("Initializing estimator to actual");

         robotInitialSetup.get().initializeRobot(robot.getRootBody());
         robot.updateFrames();
         FloatingJointBasics rootJoint = (FloatingJointBasics) robot.getRootBody().getChildrenJoints().get(0);
         RigidBodyTransform rootJointTransform = new RigidBodyTransform(rootJoint.getJointPose().getOrientation(), rootJoint.getJointPose().getPosition());

         TObjectDoubleMap<String> jointPositions = new TObjectDoubleHashMap<>();
         SubtreeStreams.fromChildren(OneDoFJointBasics.class, robot.getRootBody()).forEach(joint -> jointPositions.put(joint.getName(), joint.getQ()));
         estimatorThread.initializeStateEstimators(rootJointTransform, jointPositions);
      }
   }

   private void setupSimulatedRobotTimeProvider()
   {
      simulatedRobotTimeProvider = new SimulatedDRCRobotTimeProvider(simulationDT.get());
      robot.getControllerManager().addController(() -> simulatedRobotTimeProvider.doControl());
   }

   public void setSimulationName(String simulationName)
   {
      this.simulationName.set(simulationName);
   }

   public void setRobotModel(DRCRobotModel robotModel)
   {
      this.robotModel.set(robotModel);
      simulationDT.setDefaultValue(robotModel.getSimulateDT());
      robotInitialSetup.setDefaultValue(robotModel.getDefaultRobotInitialSetup(0, 0));
   }

   public HighLevelHumanoidControllerFactory setDefaultHighLevelHumanoidControllerFactory()
   {
      DRCRobotModel robotModel = this.robotModel.get();
      HighLevelControllerParameters highLevelControllerParameters = robotModel.getHighLevelControllerParameters();
      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      PushRecoveryControllerParameters pushRecoveryControllerParameters = robotModel.getPushRecoveryControllerParameters();
      CoPTrajectoryParameters copTrajectoryParameters = robotModel.getCoPTrajectoryParameters();
      HumanoidRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
      SideDependentList<String> feetForceSensorNames = sensorInformation.getFeetForceSensorNames();
      SideDependentList<String> feetContactSensorNames = sensorInformation.getFeetContactSensorNames();
      SideDependentList<String> wristForceSensorNames = sensorInformation.getWristForceSensorNames();

      RobotContactPointParameters<RobotSide> contactPointParameters = robotModel.getContactPointParameters();
      ArrayList<String> additionalContactRigidBodyNames = contactPointParameters.getAdditionalContactRigidBodyNames();
      ArrayList<String> additionalContactNames = contactPointParameters.getAdditionalContactNames();
      ArrayList<RigidBodyTransform> additionalContactTransforms = contactPointParameters.getAdditionalContactTransforms();

      ContactableBodiesFactory<RobotSide> contactableBodiesFactory = new ContactableBodiesFactory<>();
      contactableBodiesFactory.setFootContactPoints(contactPointParameters.getFootContactPoints());
      contactableBodiesFactory.setToeContactParameters(contactPointParameters.getControllerToeContactPoints(),
                                                       contactPointParameters.getControllerToeContactLines());
      for (int i = 0; i < contactPointParameters.getAdditionalContactNames().size(); i++)
         contactableBodiesFactory.addAdditionalContactPoint(additionalContactRigidBodyNames.get(i),
                                                            additionalContactNames.get(i),
                                                            additionalContactTransforms.get(i));

      HighLevelHumanoidControllerFactory controllerFactory = new HighLevelHumanoidControllerFactory(contactableBodiesFactory,
                                                                                                    feetForceSensorNames,
                                                                                                    feetContactSensorNames,
                                                                                                    wristForceSensorNames,
                                                                                                    highLevelControllerParameters,
                                                                                                    walkingControllerParameters,
                                                                                                    pushRecoveryControllerParameters,
                                                                                                    copTrajectoryParameters);
      HighLevelControllerName fallbackControllerState = highLevelControllerParameters.getFallbackControllerState();
      controllerFactory.useDefaultDoNothingControlState();
      controllerFactory.useDefaultWalkingControlState();

      controllerFactory.addRequestableTransition(DO_NOTHING_BEHAVIOR, WALKING);
      controllerFactory.addRequestableTransition(WALKING, DO_NOTHING_BEHAVIOR);

      controllerFactory.addControllerFailureTransition(DO_NOTHING_BEHAVIOR, fallbackControllerState);
      controllerFactory.addControllerFailureTransition(WALKING, fallbackControllerState);

      controllerFactory.setInitialState(HighLevelControllerName.WALKING);

      controllerFactory.createControllerNetworkSubscriber(robotModel.getSimpleRobotName(), realtimeROS2Node.get());

      setHighLevelHumanoidControllerFactory(controllerFactory);
      return controllerFactory;
   }

   public HighLevelHumanoidControllerFactory setDefaultHighLevelHumanoidControllerFactory(boolean useVelocityAndHeadingScript,
                                                                                          HeadingAndVelocityEvaluationScriptParameters walkingScriptParameters)
   {
      HighLevelHumanoidControllerFactory controllerFactory = setDefaultHighLevelHumanoidControllerFactory();
      controllerFactory.createComponentBasedFootstepDataMessageGenerator(useVelocityAndHeadingScript, walkingScriptParameters);
      return controllerFactory;
   }

   public void setHighLevelHumanoidControllerFactory(HighLevelHumanoidControllerFactory highLevelHumanoidControllerFactory)
   {
      this.highLevelHumanoidControllerFactory.set(highLevelHumanoidControllerFactory);
   }

   public HighLevelHumanoidControllerFactory getHighLevelHumanoidControllerFactory()
   {
      return highLevelHumanoidControllerFactory.get();
   }

   public void setTerrainObjectDefinition(TerrainObjectDefinition terrainObjectDefinition)
   {
      this.terrainObjectDefinition.set(terrainObjectDefinition);
   }

   public void setCommonAvatarEnvrionmentInterface(CommonAvatarEnvironmentInterface environment)
   {
      setTerrainObjectDefinition(TerrainObjectDefinitionTools.toTerrainObjectDefinition(environment,
                                                                                        collidableHelper,
                                                                                        terrainCollisionName,
                                                                                        robotCollisionName));
   }

   public void setRobotInitialSetup(RobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup)
   {
      this.robotInitialSetup.set(robotInitialSetup);
   }

   public void setStartingLocationOffset(OffsetAndYawRobotInitialSetup startingLocationOffset)
   {
      robotInitialSetup.get().setInitialYaw(startingLocationOffset.getYaw());
      robotInitialSetup.get().setInitialGroundHeight(startingLocationOffset.getGroundHeight());
      robotInitialSetup.get().setOffset(startingLocationOffset.getAdditionalOffset());
   }

   public void setSimulationDT(double simulationDT)
   {
      this.simulationDT.set(simulationDT);
   }

   public void setSimulationDataBufferSize(int simulationDataBufferSize)
   {
      this.simulationDataBufferSize.set(simulationDataBufferSize);
   }

   public void setSimulationDataRecordTimePeriod(double simulationDataRecordTimePeriod)
   {
      simulationDataRecordTickPeriod.set((int) Math.max(1.0, simulationDataRecordTimePeriod / simulationDT.get()));
   }

   public void setSimulationDataRecordTickPeriod(int simulationDataRecordTickPeriod)
   {
      this.simulationDataRecordTickPeriod.set(simulationDataRecordTickPeriod);
   }

   public void setUsePerfectSensors(boolean usePerfectSensors)
   {
      this.usePerfectSensors.set(usePerfectSensors);
   }

   public void setOutputWriterFactory(SCS2JointDesiredOutputWriterFactory outputWriterFactory)
   {
      this.outputWriterFactory.set(outputWriterFactory);
   }

   public void setRunMultiThreaded(boolean runMultiThreaded)
   {
      this.runMultiThreaded.set(runMultiThreaded);
   }

   public void setInitializeEstimatorToActual(boolean initializeEstimatorToActual)
   {
      this.initializeEstimatorToActual.set(initializeEstimatorToActual);
   }

   public void setShowGUI(boolean showGUI)
   {
      this.showGUI.set(showGUI);
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

   public void setAutomaticallyStartSimulation(boolean automaticallyStartSimulation)
   {
      this.automaticallyStartSimulation.set(automaticallyStartSimulation);
   }

   public void setExternalPelvisCorrectorSubscriber(PelvisPoseCorrectionCommunicatorInterface externalPelvisCorrectorSubscriber)
   {
      this.externalPelvisCorrectorSubscriber.set(externalPelvisCorrectorSubscriber);
   }

   public void setUseImpulseBasedPhysicsEngine(boolean useImpulseBasePhysicsEngine)
   {
      this.useImpulseBasePhysicsEngine.set(useImpulseBasePhysicsEngine);
   }

   public void setEnableSimulatedRobotDamping(boolean enableSimulatedRobotDamping)
   {
      this.enableSimulatedRobotDamping.set(enableSimulatedRobotDamping);
   }

   public void addSecondaryRobot(Robot secondaryRobot)
   {
      this.secondaryRobots.get().add(secondaryRobot);
   }
}
