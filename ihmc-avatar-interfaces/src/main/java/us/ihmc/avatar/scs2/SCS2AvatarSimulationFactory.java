package us.ihmc.avatar.scs2;

import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.DO_NOTHING_BEHAVIOR;
import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.PUSH_RECOVERY;
import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.WALKING;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Consumer;
import java.util.stream.Collectors;

import gnu.trove.map.TObjectDoubleMap;
import gnu.trove.map.hash.TObjectDoubleHashMap;
import ihmc_common_msgs.msg.dds.StampedPosePacket;
import us.ihmc.avatar.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.SimulatedDRCRobotTimeProvider;
import us.ihmc.avatar.factory.BarrierScheduledRobotController;
import us.ihmc.avatar.factory.DisposableRobotController;
import us.ihmc.avatar.factory.HumanoidRobotControlTask;
import us.ihmc.avatar.factory.SimulatedHandControlTask;
import us.ihmc.avatar.factory.SimulatedHandOutputWriter;
import us.ihmc.avatar.factory.SimulatedHandSensorReader;
import us.ihmc.avatar.factory.SingleThreadedRobotController;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.initialSetup.RobotInitialSetup;
import us.ihmc.avatar.logging.IntraprocessYoVariableLogger;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextDataFactory;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeightMapBasedFootstepAdjustment;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.ComponentBasedFootstepDataMessageGeneratorFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.HumanoidSteppingPluginFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.JoystickBasedSteppingPluginFactory;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.BarrierScheduler.TaskOverrunBehavior;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.HeightMap;
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
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.tools.SCS1GraphicConversionTools;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletPhysicsEngine;
import us.ihmc.scs2.simulation.parameters.ContactParametersReadOnly;
import us.ihmc.scs2.simulation.parameters.ContactPointBasedContactParameters;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;
import us.ihmc.scs2.simulation.physicsEngine.contactPointBased.ContactPointBasedPhysicsEngine;
import us.ihmc.scs2.simulation.physicsEngine.impulseBased.ImpulseBasedPhysicsEngine;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.controller.SimControllerInput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;
import us.ihmc.sensorProcessing.parameters.AvatarRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.sensorProcessing.simulatedSensors.SCS2SensorReaderFactory;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationToolkit.RobotDefinitionTools;
import us.ihmc.simulationToolkit.TerrainObjectDefinitionTools;
import us.ihmc.simulationconstructionset.dataBuffer.MirroredYoVariableRegistry;
import us.ihmc.tools.factories.FactoryFieldNotSetException;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.factories.RequiredFactoryField;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.RobotContactPointParameters.GroundContactModelParameters;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SCS2AvatarSimulationFactory
{
   protected final RequiredFactoryField<DRCRobotModel> robotModel = new RequiredFactoryField<>("robotModel");
   protected final RequiredFactoryField<HighLevelHumanoidControllerFactory> highLevelHumanoidControllerFactory = new RequiredFactoryField<>("highLevelHumanoidControllerFactory");
   protected final ArrayList<TerrainObjectDefinition> terrainObjectDefinitions = new ArrayList<>();

   protected final OptionalFactoryField<RealtimeROS2Node> realtimeROS2Node = new OptionalFactoryField<>("realtimeROS2Node");
   protected final OptionalFactoryField<Double> simulationDT = new OptionalFactoryField<>("simulationDT");
   protected final OptionalFactoryField<RobotInitialSetup<HumanoidFloatingRootJointRobot>> robotInitialSetup = new OptionalFactoryField<>("robotInitialSetup");
   protected final OptionalFactoryField<Double> gravity = new OptionalFactoryField<>("gravity", -9.81);
   protected final OptionalFactoryField<Boolean> createYoVariableServer = new OptionalFactoryField<>("createYoVariableServer", false);
   protected final OptionalFactoryField<Boolean> logToFile = new OptionalFactoryField<>("logToFile", false);
   protected final OptionalFactoryField<PelvisPoseCorrectionCommunicatorInterface> externalPelvisCorrectorSubscriber = new OptionalFactoryField<>("externalPelvisCorrectorSubscriber");
   protected final OptionalFactoryField<Integer> simulationDataBufferSize = new OptionalFactoryField<>("simulationDataBufferSize", 8192);
   protected final OptionalFactoryField<Integer> simulationDataRecordTickPeriod = new OptionalFactoryField<>("simulationDataRecordTickPeriod");
   protected final OptionalFactoryField<Boolean> usePerfectSensors = new OptionalFactoryField<>("usePerfectSensors", false);
   protected final OptionalFactoryField<SCS2JointDesiredOutputWriterFactory> outputWriterFactory = new OptionalFactoryField<>("outputWriterFactory",
                                                                                                                              (in,
                                                                                                                               out) -> new SCS2OutputWriter(in,
                                                                                                                                                            out,
                                                                                                                                                            true));
   protected final OptionalFactoryField<Boolean> runMultiThreaded = new OptionalFactoryField<>("runMultiThreaded", true);
   protected final OptionalFactoryField<Boolean> initializeEstimatorToActual = new OptionalFactoryField<>("initializeEstimatorToActual", true);
   protected final OptionalFactoryField<Boolean> showGUI = new OptionalFactoryField<>("showGUI", true);
   protected final OptionalFactoryField<Boolean> automaticallyStartSimulation = new OptionalFactoryField<>("automaticallyStartSimulation", false);

   protected final OptionalFactoryField<Boolean> useImpulseBasedPhysicsEngine = new OptionalFactoryField<>("useImpulseBasePhysicsEngine", false);
   protected final OptionalFactoryField<Boolean> useBulletPhysicsEngine = new OptionalFactoryField<>("useBulletPhysicsEngine", false);
   protected final OptionalFactoryField<Consumer<RobotDefinition>> bulletCollisionMutator = new OptionalFactoryField<>("bulletCollisionMutator");
   protected final OptionalFactoryField<ContactParametersReadOnly> impulseBasedPhysicsEngineContactParameters = new OptionalFactoryField<>("impulseBasedPhysicsEngineParameters");
   protected final OptionalFactoryField<Boolean> enableSimulatedRobotDamping = new OptionalFactoryField<>("enableSimulatedRobotDamping", true);
   protected final OptionalFactoryField<Boolean> useRobotDefinitionCollisions = new OptionalFactoryField<>("useRobotDefinitionCollisions", false);
   protected final OptionalFactoryField<List<Robot>> secondaryRobots = new OptionalFactoryField<>("secondaryRobots", new ArrayList<>());
   protected final OptionalFactoryField<String> simulationName = new OptionalFactoryField<>("simulationName");

   private final OptionalFactoryField<Boolean> useHeadingAndVelocityScript = new OptionalFactoryField<>("useHeadingAndVelocityScript");
   private final OptionalFactoryField<HeightMap> heightMapForFootstepZ = new OptionalFactoryField<>("heightMapForFootstepZ");
   private final OptionalFactoryField<HeadingAndVelocityEvaluationScriptParameters> headingAndVelocityEvaluationScriptParameters = new OptionalFactoryField<>("headingAndVelocityEvaluationScriptParameters");

   // TO CONSTRUCT
   protected RobotDefinition robotDefinition;
   protected Robot robot;
   protected YoVariableServer yoVariableServer;
   protected IntraprocessYoVariableLogger intraprocessYoVariableLogger;
   protected SimulationConstructionSet2 simulationConstructionSet;
   protected JointDesiredOutputWriter simulationOutputWriter;
   protected HumanoidRobotContextData masterContext;
   protected AvatarEstimatorThread estimatorThread;
   protected AvatarControllerThread controllerThread;
   protected AvatarStepGeneratorThread stepGeneratorThread;
   protected DisposableRobotController robotController;
   protected SimulatedDRCRobotTimeProvider simulatedRobotTimeProvider;
   protected PelvisPoseCorrectionCommunicatorInterface pelvisPoseCorrectionCommunicator;

   protected final CollidableHelper collidableHelper = new CollidableHelper();
   protected final String robotCollisionName = "robot";
   protected final String terrainCollisionName = "terrain";

   public SCS2AvatarSimulation createAvatarSimulation()
   {
      simulationDataRecordTickPeriod.setDefaultValue((int) Math.max(1.0, robotModel.get().getControllerDT() / simulationDT.get()));

      FactoryTools.checkAllFactoryFieldsAreSet(this);

      setupSimulationConstructionSet();
      setupYoVariableServer();
      setupSimulationOutputWriter();
      setupStateEstimationThread();
      setupControllerThread();
      setupStepGeneratorThread();
      setupMultiThreadedRobotController();
      setupLidarController();
      initializeStateEstimatorToActual();
      setupSimulatedRobotTimeProvider();

      SCS2AvatarSimulation avatarSimulation = new SCS2AvatarSimulation();
      avatarSimulation.setRobotModel(robotModel.get());
      avatarSimulation.setRobotInitialSetup(robotInitialSetup.get());
      avatarSimulation.setSimulationConstructionSet(simulationConstructionSet);
      avatarSimulation.setHighLevelHumanoidControllerFactory(highLevelHumanoidControllerFactory.get());
      avatarSimulation.setYoVariableServer(yoVariableServer);
      avatarSimulation.setIntraprocessYoVariableLogger(intraprocessYoVariableLogger);
      avatarSimulation.setMasterContext(masterContext);
      avatarSimulation.setControllerThread(controllerThread);
      avatarSimulation.setEstimatorThread(estimatorThread);
      avatarSimulation.setStepGeneratorThread(stepGeneratorThread);
      avatarSimulation.setRobotController(robotController);
      avatarSimulation.setRobot(robot);
      avatarSimulation.setSimulatedRobotTimeProvider(simulatedRobotTimeProvider);
      avatarSimulation.setFullHumanoidRobotModel(controllerThread.getFullRobotModel());
      avatarSimulation.setShowGUI(showGUI.get());
      avatarSimulation.setAutomaticallyStartSimulation(automaticallyStartSimulation.get());

      if (realtimeROS2Node.hasBeenSet())
      {
         avatarSimulation.setRealTimeROS2Node(realtimeROS2Node.get());
      }

      FactoryTools.disposeFactory(this);

      return avatarSimulation;
   }

   private void setupSimulationConstructionSet()
   {
      DRCRobotModel robotModel = this.robotModel.get();

      robotDefinition = robotModel.getRobotDefinition();

      if (useBulletPhysicsEngine.get() && bulletCollisionMutator.hasValue())
      {
         bulletCollisionMutator.get().accept(robotDefinition);
      }

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

      if (!useRobotDefinitionCollisions.get())
      {
         RobotCollisionModel collisionModel = robotModel.getSimulationRobotCollisionModel(collidableHelper, robotCollisionName, terrainCollisionName);
         if (collisionModel != null)
            RobotDefinitionTools.addCollisionsToRobotDefinition(collisionModel.getRobotCollidables(robotModel.createFullRobotModel().getElevator()),
                                                                robotDefinition);
      }

      robotInitialSetup.get().initializeRobotDefinition(robotDefinition);
      Set<String> lastSimulatedJoints = robotModel.getJointMap().getLastSimulatedJoints();
      lastSimulatedJoints.forEach(lastSimulatedJoint -> robotDefinition.addSubtreeJointsToIgnore(lastSimulatedJoint));

      PhysicsEngineFactory physicsEngineFactory;

      if (useImpulseBasedPhysicsEngine.hasValue() && useImpulseBasedPhysicsEngine.get())
      {
         physicsEngineFactory = (inertialFrame, rootRegistry) ->
         {
            ImpulseBasedPhysicsEngine physicsEngine = new ImpulseBasedPhysicsEngine(inertialFrame, rootRegistry);
            if (impulseBasedPhysicsEngineContactParameters.hasValue())
               physicsEngine.setGlobalContactParameters(impulseBasedPhysicsEngineContactParameters.get());
            return physicsEngine;
         };
      }
      else if (useBulletPhysicsEngine.hasValue() && useBulletPhysicsEngine.get())
      {
         physicsEngineFactory = (inertialFrame, rootRegistry) -> new BulletPhysicsEngine(inertialFrame, rootRegistry);
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
      simulationConstructionSet = new SimulationConstructionSet2(name, physicsEngineFactory);
      simulationConstructionSet.initializeBufferSize(simulationDataBufferSize.get());
      simulationConstructionSet.initializeBufferRecordTickPeriod(simulationDataRecordTickPeriod.get());
      if (terrainObjectDefinitions.isEmpty())
         throw new FactoryFieldNotSetException("terrainObjectDefinitions");
      for (TerrainObjectDefinition terrainObjectDefinition : terrainObjectDefinitions)
      {
         simulationConstructionSet.addTerrainObject(terrainObjectDefinition);
      }
      robot = simulationConstructionSet.addRobot(robotDefinition);
      robot.addThrottledController(new SCS2StateEstimatorDebugVariables(simulationConstructionSet.getInertialFrame(),
                                                                        gravity.get(),
                                                                        robotModel.getEstimatorDT(),
                                                                        robot.getControllerManager().getControllerInput()),
                                   robotModel.getEstimatorDT());

      for (Robot secondaryRobot : secondaryRobots.get())
         simulationConstructionSet.addRobot(secondaryRobot);

      simulationConstructionSet.setDT(simulationDT.get());
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

      ROS2Topic<?> outputTopic = null;
      ROS2Topic<?> inputTopic = null;

      if (realtimeROS2Node.hasBeenSet())
      {
         outputTopic = ROS2Tools.getControllerOutputTopic(robotName);
         inputTopic = ROS2Tools.getControllerInputTopic(robotName);
      }

      if (externalPelvisCorrectorSubscriber.hasValue())
      {
         pelvisPoseCorrectionCommunicator = externalPelvisCorrectorSubscriber.get();
      }
      else
      {
         if (realtimeROS2Node.hasBeenSet())
         {
            pelvisPoseCorrectionCommunicator = new PelvisPoseCorrectionCommunicator(realtimeROS2Node.get(), outputTopic);
            ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeROS2Node.get(),
                                                          StampedPosePacket.class,
                                                          inputTopic,
                                                          s -> pelvisPoseCorrectionCommunicator.receivedPacket(s.takeNextData()));
         }
      }

      HumanoidRobotContextDataFactory contextDataFactory = new HumanoidRobotContextDataFactory();
      AvatarEstimatorThreadFactory avatarEstimatorThreadFactory = new AvatarEstimatorThreadFactory();

      if (realtimeROS2Node.hasBeenSet())
      {
         avatarEstimatorThreadFactory.setROS2Info(realtimeROS2Node.get(), robotName);
      }
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

      RealtimeROS2Node ros2Node = null;
      if (realtimeROS2Node.hasBeenSet())
      {
         ros2Node = realtimeROS2Node.get();
      }

      controllerThread = new AvatarControllerThread(robotName,
                                                    robotModel.get(),
                                                    robotInitialSetup.get(),
                                                    robotModel.get().getSensorInformation(),
                                                    highLevelHumanoidControllerFactory.get(),
                                                    contextDataFactory,
                                                    null,
                                                    ros2Node,
                                                    gravity.get(),
                                                    robotModel.get().getEstimatorDT());
      simulationConstructionSet.addYoGraphics(SCS1GraphicConversionTools.toYoGraphicDefinitions(controllerThread.getYoGraphicsListRegistry()));
   }

   private void setupStepGeneratorThread()
   {
      HumanoidRobotContextDataFactory contextDataFactory = new HumanoidRobotContextDataFactory();

      HumanoidSteppingPluginFactory steppingFactory;
      AvatarStepGeneratorEnvironmentUpdatable stepSnapperUpdatable = null;
      boolean useHeadingAndVelocityScript = this.useHeadingAndVelocityScript.hasValue() ? this.useHeadingAndVelocityScript.get() : false;
      HeadingAndVelocityEvaluationScriptParameters parameters = headingAndVelocityEvaluationScriptParameters.hasValue()
            ? headingAndVelocityEvaluationScriptParameters.get()
            : null;
      if (useHeadingAndVelocityScript || parameters != null)
      {
         ComponentBasedFootstepDataMessageGeneratorFactory componentBasedFootstepDataMessageGeneratorFactory = new ComponentBasedFootstepDataMessageGeneratorFactory();
         componentBasedFootstepDataMessageGeneratorFactory.setRegistry();
         componentBasedFootstepDataMessageGeneratorFactory.setUseHeadingAndVelocityScript(useHeadingAndVelocityScript);
         if (parameters != null)
            componentBasedFootstepDataMessageGeneratorFactory.setHeadingAndVelocityEvaluationScriptParameters(parameters);
         if (heightMapForFootstepZ.hasValue())
            componentBasedFootstepDataMessageGeneratorFactory.setFootStepAdjustment(new HeightMapBasedFootstepAdjustment(heightMapForFootstepZ.get()));

         steppingFactory = componentBasedFootstepDataMessageGeneratorFactory;
      }
      else
      {
         JoystickBasedSteppingPluginFactory joystickPluginFactory = new JoystickBasedSteppingPluginFactory();
         if (heightMapForFootstepZ.hasValue())
            joystickPluginFactory.setFootStepAdjustment(new HeightMapBasedFootstepAdjustment(heightMapForFootstepZ.get()));
         else
         {
            stepSnapperUpdatable = new AvatarStepGeneratorEnvironmentUpdatable(robotModel.get().getWalkingControllerParameters().getSteppingParameters(),
                                                                               joystickPluginFactory.getStepGeneratorCommandInputManager());
            stepSnapperUpdatable.setShouldSnapToRegions(true);
         }

         steppingFactory = joystickPluginFactory;
      }

      stepGeneratorThread = new AvatarStepGeneratorThread(steppingFactory,
                                                          contextDataFactory,
                                                          highLevelHumanoidControllerFactory.get().getStatusOutputManager(),
                                                          highLevelHumanoidControllerFactory.get().getCommandInputManager(),
                                                          robotModel.get(),
                                                          stepSnapperUpdatable,
                                                          realtimeROS2Node.get());
   }

   private void setupMultiThreadedRobotController()
   {
      DRCRobotModel robotModel = this.robotModel.get();

      // Create intermediate data buffer for threading.
      FullHumanoidRobotModel masterFullRobotModel = robotModel.createFullRobotModel();
      robotInitialSetup.get().initializeFullRobotModel(masterFullRobotModel);
      masterContext = new HumanoidRobotContextData(masterFullRobotModel);

      // Create the tasks that will be run on their own threads.
      int estimatorDivisor = (int) Math.round(robotModel.getEstimatorDT() / simulationDT.get());
      int controllerDivisor = (int) Math.round(robotModel.getControllerDT() / simulationDT.get());
      int stepGeneratorDivisor = (int) Math.round(robotModel.getStepGeneratorDT() / robotModel.getSimulateDT());
      int handControlDivisor = (int) Math.round(robotModel.getSimulatedHandControlDT() / simulationDT.get());
      HumanoidRobotControlTask estimatorTask = new EstimatorTask(estimatorThread, estimatorDivisor, simulationDT.get(), masterFullRobotModel);
      HumanoidRobotControlTask controllerTask = new ControllerTask("Controller", controllerThread, controllerDivisor, simulationDT.get(), masterFullRobotModel);
      HumanoidRobotControlTask stepGeneratorTask = new StepGeneratorTask("StepGenerator",
                                                                         stepGeneratorThread,
                                                                         stepGeneratorDivisor,
                                                                         simulationDT.get(),
                                                                         masterFullRobotModel);

      SimulatedHandControlTask handControlTask = null;
      AvatarSimulatedHandControlThread handControlThread = null;

      if (realtimeROS2Node.hasBeenSet())
      {
         handControlThread = robotModel.createSimulatedHandController(realtimeROS2Node.get());

         if (handControlThread != null)
         {
            List<String> fingerJointNames = handControlThread.getControlledOneDoFJoints().stream().map(JointReadOnly::getName).collect(Collectors.toList());
            SimulatedHandSensorReader handSensorReader = new SCS2SimulatedHandSensorReader(robot.getControllerManager().getControllerInput(), fingerJointNames);
            SimulatedHandOutputWriter handOutputWriter = new SCS2SimulatedHandOutputWriter(robot.getControllerManager().getControllerInput(),
                                                                                           robot.getControllerManager().getControllerOutput());
            handControlTask = new SimulatedHandControlTask(handSensorReader, handControlThread, handOutputWriter, handControlDivisor, simulationDT.get());
         }
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
      tasks.add(stepGeneratorTask);
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
         builders.add(new RegistrySendBufferBuilder(stepGeneratorThread.getYoVariableRegistry(), stepGeneratorThread.getYoGraphicsListRegistry()));
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
         yoVariableServer.addRegistry(stepGeneratorThread.getYoVariableRegistry(), stepGeneratorThread.getYoGraphicsListRegistry());
         stepGeneratorTask.addRunnableOnTaskThread(() -> yoVariableServer.update(stepGeneratorThread.getHumanoidRobotContextData().getTimestamp(),
                                                                                 stepGeneratorThread.getYoVariableRegistry()));
      }

      List<MirroredYoVariableRegistry> mirroredRegistries = new ArrayList<>();
      mirroredRegistries.add(setupWithMirroredRegistry(estimatorThread.getYoRegistry(), estimatorTask, robotController.getYoRegistry()));
      mirroredRegistries.add(setupWithMirroredRegistry(controllerThread.getYoVariableRegistry(), controllerTask, robotController.getYoRegistry()));
      mirroredRegistries.add(setupWithMirroredRegistry(stepGeneratorThread.getYoVariableRegistry(), stepGeneratorTask, robotController.getYoRegistry()));
      if (handControlThread != null)
         mirroredRegistries.add(setupWithMirroredRegistry(handControlThread.getYoVariableRegistry(), handControlTask, robotController.getYoRegistry()));
      robot.getRegistry().addChild(robotController.getYoRegistry());
      robot.getControllerManager().addController(new Controller()
      {
         @Override
         public void initialize()
         {
            // We splitting steps the MirroredYoVariableRegistry#updateMirror() here to avoid values from the threads getting overridden.
            // Maybe this behavior could be considered a bug...
            mirroredRegistries.forEach(mirror -> mirror.updateChangedValues()); // Pulling values from the simulation's variables

            FloatingJointBasics rootJoint = (FloatingJointBasics) robot.getRootBody().getChildrenJoints().get(0);
            RigidBodyTransform rootJointTransform = new RigidBodyTransform(rootJoint.getJointPose().getOrientation(), rootJoint.getJointPose().getPosition());

            TObjectDoubleMap<String> jointPositions = new TObjectDoubleHashMap<>();
            SubtreeStreams.fromChildren(OneDoFJointBasics.class, robot.getRootBody()).forEach(joint -> jointPositions.put(joint.getName(), joint.getQ()));
            estimatorThread.initializeStateEstimators(rootJointTransform, jointPositions);
            controllerThread.initialize();
            stepGeneratorThread.initialize();
            masterContext.set(estimatorThread.getHumanoidRobotContextData());

            robotController.initialize();
            mirroredRegistries.forEach(mirror -> mirror.updateValuesFromOriginal()); // Pushing the tasks values to the simulation's variables
         }

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

   private static MirroredYoVariableRegistry setupWithMirroredRegistry(YoRegistry registry, HumanoidRobotControlTask owner, YoRegistry schedulerRegistry)
   {
      MirroredYoVariableRegistry mirroredRegistry = new MirroredYoVariableRegistry(registry);
      owner.addRunnableOnSchedulerThread(() ->
      {
         // Reversed the ordering of MirroredYoVariableRegistry#updateMirror() to inverse priority when variables are changed on both sides.
         // This way, changes on the controller side take priority over changes in SCS.
         mirroredRegistry.updateValuesFromOriginal();
         mirroredRegistry.updateChangedValues();
      });
      schedulerRegistry.addChild(mirroredRegistry);
      return mirroredRegistry;
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
                                                                                                    copTrajectoryParameters,
                                                                                                    robotModel.getSplitFractionCalculatorParameters());
      HighLevelControllerName fallbackControllerState = highLevelControllerParameters.getFallbackControllerState();
      controllerFactory.useDefaultDoNothingControlState();
      controllerFactory.useDefaultWalkingControlState();
      if (pushRecoveryControllerParameters != null)
         controllerFactory.useDefaultPushRecoveryControlState();

      controllerFactory.addRequestableTransition(DO_NOTHING_BEHAVIOR, WALKING);
      controllerFactory.addRequestableTransition(WALKING, DO_NOTHING_BEHAVIOR);
      controllerFactory.addRequestableTransition(WALKING, PUSH_RECOVERY);
      controllerFactory.addFinishedTransition(PUSH_RECOVERY, WALKING);

      controllerFactory.addControllerFailureTransition(DO_NOTHING_BEHAVIOR, fallbackControllerState);
      controllerFactory.addControllerFailureTransition(WALKING, fallbackControllerState);

      controllerFactory.setInitialState(HighLevelControllerName.WALKING);

      if (realtimeROS2Node.hasBeenSet())
      {
         controllerFactory.createControllerNetworkSubscriber(robotModel.getSimpleRobotName(), realtimeROS2Node.get());
      }

      setHighLevelHumanoidControllerFactory(controllerFactory);
      return controllerFactory;
   }

   public HighLevelHumanoidControllerFactory setDefaultHighLevelHumanoidControllerFactory(boolean useVelocityAndHeadingScript,
                                                                                          HeadingAndVelocityEvaluationScriptParameters walkingScriptParameters)
   {
      HighLevelHumanoidControllerFactory controllerFactory;

      if (highLevelHumanoidControllerFactory.hasBeenSet())
         controllerFactory = highLevelHumanoidControllerFactory.get();
      else
         controllerFactory = setDefaultHighLevelHumanoidControllerFactory();
      setComponentBasedFootstepDataMessageGeneratorParameters(useVelocityAndHeadingScript, walkingScriptParameters);
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

   public void addTerrainObjectDefinition(TerrainObjectDefinition terrainObjectDefinition)
   {
      terrainObjectDefinitions.add(terrainObjectDefinition);
   }

   public void setCommonAvatarEnvrionmentInterface(CommonAvatarEnvironmentInterface environment)
   {
      addTerrainObjectDefinition(TerrainObjectDefinitionTools.toTerrainObjectDefinition(environment,
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

   public void setUseImpulseBasedPhysicsEngine(boolean useImpulseBasedPhysicsEngine)
   {
      this.useImpulseBasedPhysicsEngine.set(useImpulseBasedPhysicsEngine);
   }

   public void setImpulseBasedPhysicsEngineContactParameters(ContactParametersReadOnly contactParameters)
   {
      this.impulseBasedPhysicsEngineContactParameters.set(contactParameters);
   }

   public void setUseBulletPhysicsEngine(boolean useBulletPhysicsEngine)
   {
      this.useBulletPhysicsEngine.set(useBulletPhysicsEngine);
   }

   public void setBulletCollisionMutator(Consumer<RobotDefinition> bulletCollisionMutator)
   {
      this.bulletCollisionMutator.set(bulletCollisionMutator);
   }

   public void setEnableSimulatedRobotDamping(boolean enableSimulatedRobotDamping)
   {
      this.enableSimulatedRobotDamping.set(enableSimulatedRobotDamping);
   }

   public void setUseRobotDefinitionCollisions(boolean useRobotDefinitionCollisions)
   {
      this.useRobotDefinitionCollisions.set(useRobotDefinitionCollisions);
   }

   public void addSecondaryRobot(Robot secondaryRobot)
   {
      this.secondaryRobots.get().add(secondaryRobot);
   }

   public void setComponentBasedFootstepDataMessageGeneratorParameters(boolean useHeadingAndVelocityScript,
                                                                       HeadingAndVelocityEvaluationScriptParameters headingAndVelocityEvaluationScriptParameters)
   {
      setComponentBasedFootstepDataMessageGeneratorParameters(useHeadingAndVelocityScript, null, headingAndVelocityEvaluationScriptParameters);
   }

   public void setComponentBasedFootstepDataMessageGeneratorParameters(boolean useHeadingAndVelocityScript,
                                                                       HeightMap heightMapForFootstepZ,
                                                                       HeadingAndVelocityEvaluationScriptParameters headingAndVelocityEvaluationScriptParameters)
   {
      this.useHeadingAndVelocityScript.set(useHeadingAndVelocityScript);
      this.heightMapForFootstepZ.set(heightMapForFootstepZ);
      this.headingAndVelocityEvaluationScriptParameters.set(headingAndVelocityEvaluationScriptParameters);
   }

   public AvatarStepGeneratorThread getStepGeneratorThread()
   {
      return stepGeneratorThread;
   }
}
