package us.ihmc.avatar.testTools;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.fail;

import java.io.InputStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentLinkedQueue;

import controller_msgs.msg.dds.MessageCollection;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.factory.AvatarSimulation;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.avatar.obstacleCourseTests.ForceSensorHysteresisCreator;
import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerStateTransitionFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControllerStateFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.net.LocalObjectCommunicator;
import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.behaviors.scripts.engine.ScriptBasedControllerCommandGenerator;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculatorListener;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.simulationTesting.NothingChangedVerifier;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public class DRCSimulationTestHelper
{
   private SimulationConstructionSet scs;
   private HumanoidFloatingRootJointRobot sdfRobot;
   private AvatarSimulation avatarSimulation;

   private CommonAvatarEnvironmentInterface testEnvironment = new DefaultCommonAvatarEnvironment();

   private final SimulationTestingParameters simulationTestingParameters;

   private final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.INTRAPROCESS, "ihmc_simulation_test_helper");

   private NothingChangedVerifier nothingChangedVerifier;
   private BlockingSimulationRunner blockingSimulationRunner;
   private final WalkingControllerParameters walkingControlParameters;

   private final DRCRobotModel robotModel;
   private final FullHumanoidRobotModel fullRobotModel;
   private final ScriptedFootstepGenerator scriptedFootstepGenerator;

   private DRCNetworkModuleParameters networkProcessorParameters = new DRCNetworkModuleParameters();
   private DRCSimulationStarter simulationStarter;
   private Exception caughtException;

   private OffsetAndYawRobotInitialSetup startingLocation = new OffsetAndYawRobotInitialSetup();
   private boolean addFootstepMessageGenerator = false;
   private boolean useHeadingAndVelocityScript = false;
   private boolean cheatWithGroundHeightAtFootstep = false;
   private HighLevelControllerStateFactory controllerStateFactory = null;
   private ControllerStateTransitionFactory<HighLevelControllerName> controllerStateTransitionFactory = null;
   private DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> initialSetup = null;
   private HeadingAndVelocityEvaluationScriptParameters walkingScriptParameters = null;
   private final DRCGuiInitialSetup guiInitialSetup;

   private final boolean checkIfDesiredICPHasBeenInvalid = true;
   protected final String robotName;

   @SuppressWarnings("rawtypes")
   private final Map<Class<?>, IHMCROS2Publisher> defaultControllerPublishers = new HashMap<>();

   public DRCSimulationTestHelper(SimulationTestingParameters simulationTestParameters, DRCRobotModel robotModel)
   {
      this(simulationTestParameters, robotModel, null);
   }

   public DRCSimulationTestHelper(SimulationTestingParameters simulationTestParameters, DRCRobotModel robotModel,
                                  CommonAvatarEnvironmentInterface testEnvironment)
   {
      this.robotModel = robotModel;
      this.walkingControlParameters = robotModel.getWalkingControllerParameters();
      this.simulationTestingParameters = simulationTestParameters;

      robotName = robotModel.getSimpleRobotName();

      if (testEnvironment != null)
         this.testEnvironment = testEnvironment;
      simulationStarter = new DRCSimulationStarter(robotModel, this.testEnvironment);

      fullRobotModel = robotModel.createFullRobotModel();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      scriptedFootstepGenerator = new ScriptedFootstepGenerator(referenceFrames, fullRobotModel, walkingControlParameters);

      guiInitialSetup = new DRCGuiInitialSetup(false, false, simulationTestingParameters);

      networkProcessorParameters.enableNetworkProcessor(false);
      networkProcessorParameters.enableLocalControllerCommunicator(true);

      List<Class<? extends Command<?, ?>>> controllerSupportedCommands = ControllerAPIDefinition.getControllerSupportedCommands();

      for (Class<? extends Command<?, ?>> command : controllerSupportedCommands)
      {
         Class<?> messageClass = ROS2Tools.newMessageInstance(command).getMessageClass();
         IHMCROS2Publisher<?> defaultPublisher = createPublisherForController(messageClass);
         defaultControllerPublishers.put(messageClass, defaultPublisher);
      }

      defaultControllerPublishers.put(WholeBodyTrajectoryMessage.class, createPublisherForController(WholeBodyTrajectoryMessage.class));
      defaultControllerPublishers.put(MessageCollection.class, createPublisherForController(MessageCollection.class));
   }

   /**
    * Use
    * {@link #DRCSimulationTestHelper(SimulationTestingParameters, DRCRobotModel, CommonAvatarEnvironmentInterface)}
    * instead.
    */
   @Deprecated
   public void setTestEnvironment(CommonAvatarEnvironmentInterface testEnvironment)
   {
      this.testEnvironment = testEnvironment;
      simulationStarter = new DRCSimulationStarter(robotModel, testEnvironment);
   }

   public void createSimulation(String name)
   {
      createSimulation(name, true, true);
   }

   public void createSimulation(String name, boolean automaticallySpawnSimulation, boolean useBlockingSimulationRunner)
   {
      simulationStarter.setRunMultiThreaded(simulationTestingParameters.getRunMultiThreaded());
      simulationStarter.setUsePerfectSensors(simulationTestingParameters.getUsePefectSensors());
      if (controllerStateFactory != null)
         simulationStarter.registerHighLevelControllerState(controllerStateFactory);
      if (controllerStateTransitionFactory != null)
         simulationStarter.registerControllerStateTransition(controllerStateTransitionFactory);
      if (initialSetup != null)
         simulationStarter.setRobotInitialSetup(initialSetup);
      simulationStarter.setStartingLocationOffset(startingLocation);
      simulationStarter.setGuiInitialSetup(guiInitialSetup);
      simulationStarter.setInitializeEstimatorToActual(true);
      simulationStarter.setFlatGroundWalkingScriptParameters(walkingScriptParameters);

      if (addFootstepMessageGenerator)
         simulationStarter.addFootstepMessageGenerator(useHeadingAndVelocityScript, cheatWithGroundHeightAtFootstep);

      networkProcessorParameters.enableLocalControllerCommunicator(true);
      simulationStarter.createSimulation(networkProcessorParameters, automaticallySpawnSimulation, false);

      scs = simulationStarter.getSimulationConstructionSet();
      sdfRobot = simulationStarter.getSDFRobot();
      avatarSimulation = simulationStarter.getAvatarSimulation();
      if (useBlockingSimulationRunner)
      {
         blockingSimulationRunner = new BlockingSimulationRunner(scs, 60.0 * 10.0);
         simulationStarter.attachControllerFailureListener(direction -> blockingSimulationRunner.notifyControllerHasFailed());
         blockingSimulationRunner.createValidDesiredICPListener();
         blockingSimulationRunner.setCheckDesiredICPPosition(checkIfDesiredICPHasBeenInvalid);
      }

      if (simulationTestingParameters.getCheckNothingChangedInSimulation())
      {
         nothingChangedVerifier = new NothingChangedVerifier(name, scs);
      }
      else
      {
         nothingChangedVerifier = null;
      }
   }

   public YoVariable<?> getYoVariable(String name)
   {
      return scs.getVariable(name);
   }

   public YoVariable<?> getYoVariable(String nameSpace, String name)
   {
      return scs.getVariable(nameSpace, name);
   }

   public void loadScriptFile(InputStream scriptInputStream, ReferenceFrame referenceFrame)
   {
      ScriptBasedControllerCommandGenerator scriptBasedControllerCommandGenerator = simulationStarter.getScriptBasedControllerCommandGenerator();
      scriptBasedControllerCommandGenerator.loadScriptFile(scriptInputStream, referenceFrame);
   }

   public DRCSCSInitialSetup getSCSInitialSetup()
   {
      return simulationStarter.getSCSInitialSetup();
   }

   public ConcurrentLinkedQueue<Command<?, ?>> getQueuedControllerCommands()
   {
      return simulationStarter.getQueuedControllerCommands();
   }

   public BlockingSimulationRunner getBlockingSimulationRunner()
   {
      return blockingSimulationRunner;
   }

   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return scs;
   }

   public AvatarSimulation getAvatarSimulation()
   {
      return avatarSimulation;
   }

   public void setInverseDynamicsCalculatorListener(InverseDynamicsCalculatorListener inverseDynamicsCalculatorListener)
   {
      HighLevelHumanoidControllerFactory controllerFactory = avatarSimulation.getHighLevelHumanoidControllerFactory();
      controllerFactory.setInverseDynamicsCalculatorListener(inverseDynamicsCalculatorListener);
   }

   public FullHumanoidRobotModel getControllerFullRobotModel()
   {
      return avatarSimulation.getControllerFullRobotModel();
   }

   public FullHumanoidRobotModel getSDFFullRobotModel()
   {
      return fullRobotModel;
   }

   public CommonHumanoidReferenceFrames getReferenceFrames()
   {
      HighLevelHumanoidControllerFactory momentumBasedControllerFactory = avatarSimulation.getHighLevelHumanoidControllerFactory();
      HighLevelHumanoidControllerToolbox highLevelHumanoidControllerToolbox = momentumBasedControllerFactory.getHighLevelHumanoidControllerToolbox();
      return highLevelHumanoidControllerToolbox.getReferenceFrames();
   }

   /**
    * For unit testing only
    *
    * @param controller
    */
   public void addRobotControllerOnControllerThread(RobotController controller)
   {
      avatarSimulation.addRobotControllerOnControllerThread(controller);
   }

   /**
    * For unit testing only
    *
    * @param controller
    */
   public void addRobotControllerOnEstimatorThread(RobotController controller)
   {
      avatarSimulation.addRobotControllerOnEstimatorThread(controller);
   }

   public CommonAvatarEnvironmentInterface getTestEnvironment()
   {
      return testEnvironment;
   }

   public ScriptedFootstepGenerator createScriptedFootstepGenerator()
   {
      return scriptedFootstepGenerator;
   }

   public void checkNothingChanged()
   {
      if (simulationTestingParameters.getCheckNothingChangedInSimulation())
      {
         ThreadTools.sleep(1000);

         ArrayList<String> stringsToIgnore = createVariableNamesStringsToIgnore();

         boolean writeNewBaseFile = nothingChangedVerifier.getWriteNewBaseFile();

         double maxPercentDifference = 0.001;
         nothingChangedVerifier.verifySameResultsAsPreviously(maxPercentDifference, stringsToIgnore);
         assertFalse("Had to write new base file. On next run nothing should change", writeNewBaseFile);
      }
   }

   public HumanoidFloatingRootJointRobot getRobot()
   {
      return sdfRobot;
   }

   public void simulateAndBlock(double simulationTime) throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      if (blockingSimulationRunner != null)
      {
         blockingSimulationRunner.simulateAndBlock(simulationTime);
      }
   }

   public void destroySimulation()
   {
      if (blockingSimulationRunner != null)
      {
         blockingSimulationRunner.destroySimulation();
      }
      blockingSimulationRunner = null;
      if (avatarSimulation != null)
      {
         avatarSimulation.dispose();
      }

      if (networkProcessorParameters != null)
      {
         LocalObjectCommunicator simulatedSensorCommunicator = networkProcessorParameters.getSimulatedSensorCommunicator();
         if (simulatedSensorCommunicator != null)
         {
            simulatedSensorCommunicator.disconnect();
         }
      }

      if (simulationStarter != null)
         simulationStarter.close();
      simulationStarter = null;

      ros2Node.destroy();
      ReferenceFrameTools.clearWorldFrameTree();
   }

   public boolean simulateAndBlockAndCatchExceptions(double simulationTime) throws SimulationExceededMaximumTimeException
   {
      try
      {
         simulateAndBlock(simulationTime);
         return true;
      }
      catch (Exception e)
      {
         this.caughtException = e;
         PrintTools.error(this, e.getMessage());
         return false;
      }
   }

   public void addChildRegistry(YoVariableRegistry childRegistry)
   {
      scs.getRootRegistry().addChild(childRegistry);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return scs.getRootRegistry();
   }

   public void createVideo(String simplifiedRobotModelName, int callStackHeight)
   {
      if (simulationTestingParameters.getCreateSCSVideos())
      {
         BambooTools.createVideoWithDateTimeClassMethodAndShareOnSharedDriveIfAvailable(simplifiedRobotModelName, scs, callStackHeight);
      }
   }

   public void createVideo(String videoName)
   {
      if (simulationTestingParameters.getCreateSCSVideos())
      {
         BambooTools.createVideoWithDateTimeAndStoreInDefaultDirectory(scs, videoName);
      }
   }

   public RobotSide[] createRobotSidesStartingFrom(RobotSide robotSide, int length)
   {
      RobotSide[] ret = new RobotSide[length];

      for (int i = 0; i < length; i++)
      {
         ret[i] = robotSide;
         robotSide = robotSide.getOppositeSide();
      }

      return ret;
   }

   public void setupCameraForUnitTest(Point3D cameraFix, Point3D cameraPosition)
   {
      CameraConfiguration cameraConfiguration = new CameraConfiguration("testCamera");

      cameraConfiguration.setCameraFix(cameraFix);
      cameraConfiguration.setCameraPosition(cameraPosition);
      cameraConfiguration.setCameraTracking(false, true, true, false);
      cameraConfiguration.setCameraDolly(false, true, true, false);
      scs.setupCamera(cameraConfiguration);
      scs.selectCamera("testCamera");
   }

   public void setupCameraForUnitTest(boolean enableTracking, Point3D cameraFix, Point3D cameraPosition)
   {
      CameraConfiguration cameraConfiguration = new CameraConfiguration("testCamera");

      cameraConfiguration.setCameraFix(cameraFix);
      cameraConfiguration.setCameraPosition(cameraPosition);
      cameraConfiguration.setCameraTracking(enableTracking, true, true, false);
      cameraConfiguration.setCameraDolly(false, true, true, false);
      scs.setupCamera(cameraConfiguration);
      scs.selectCamera("testCamera");
   }

   public void assertRobotsRootJointIsInBoundingBox(BoundingBox3D boundingBox)
   {
      assertRobotsRootJointIsInBoundingBox(boundingBox, getRobot());
   }

   public static void assertRobotsRootJointIsInBoundingBox(BoundingBox3D boundingBox, HumanoidFloatingRootJointRobot robot)
   {
      Point3D position = new Point3D();
      robot.getRootJoint().getPosition(position);
      boolean inside = boundingBox.isInsideInclusive(position);
      if (!inside)
      {
         fail("Joint was at " + position + ". Expecting it to be inside boundingBox " + boundingBox);
      }
   }

   public static ArrayList<String> createVariableNamesStringsToIgnore()
   {
      ArrayList<String> exceptions = new ArrayList<String>();
      exceptions.add("nano");
      exceptions.add("milli");
      exceptions.add("Timer");
      exceptions.add("startTime");
      exceptions.add("actualEstimatorDT");
      exceptions.add("nextExecutionTime");
      exceptions.add("totalDelay");
      exceptions.add("lastEstimatorClockStartTime");
      exceptions.add("lastControllerClockTime");
      exceptions.add("controllerStartTime");
      exceptions.add("actualControlDT");
      exceptions.add("timePassed");

      //    exceptions.add("gc_");
      //    exceptions.add("toolFrame");
      //    exceptions.add("ef_");
      //    exceptions.add("kp_");

      return exceptions;
   }

   public Exception getCaughtException()
   {
      return caughtException;
   }

   public ArrayList<RobotController> getFootForceSensorHysteresisCreators()
   {
      SideDependentList<ArrayList<WrenchCalculatorInterface>> footForceSensors = new SideDependentList<ArrayList<WrenchCalculatorInterface>>();
      packFootForceSensors(footForceSensors);

      ArrayList<RobotController> footForceSensorSignalCorruptors = new ArrayList<RobotController>();

      for (RobotSide robotSide : RobotSide.values)
      {
         for (int i = 0; i < footForceSensors.get(robotSide).size(); i++)
         {
            ForceSensorHysteresisCreator forceSensorSignalCorruptor = new ForceSensorHysteresisCreator(sdfRobot.computeCenterOfMass(new Point3D()),
                                                                                                       footForceSensors.get(robotSide).get(i).getName(),
                                                                                                       footForceSensors.get(robotSide).get(i));

            footForceSensorSignalCorruptors.add(forceSensorSignalCorruptor);
         }
      }

      return footForceSensorSignalCorruptors;
   }

   public void packFootForceSensors(SideDependentList<ArrayList<WrenchCalculatorInterface>> footForceSensors)
   {
      ArrayList<WrenchCalculatorInterface> forceSensors = new ArrayList<WrenchCalculatorInterface>();
      sdfRobot.getForceSensors(forceSensors);

      SideDependentList<String> jointNamesBeforeFeet = sdfRobot.getJointNamesBeforeFeet();

      for (RobotSide robotSide : RobotSide.values)
      {
         footForceSensors.put(robotSide, new ArrayList<WrenchCalculatorInterface>());
         for (int i = 0; i < forceSensors.size(); i++)
         {
            if (forceSensors.get(i).getJoint().getName().equals(jointNamesBeforeFeet.get(robotSide)))
            {
               footForceSensors.get(robotSide).add(forceSensors.get(i));
            }
         }
      }
   }

   public void setStartingLocation(DRCStartingLocation startingLocation)
   {
      if (startingLocation != null)
      {
         this.startingLocation = startingLocation.getStartingLocationOffset();
      }
   }

   public void setStartingLocation(OffsetAndYawRobotInitialSetup startingLocation)
   {
      this.startingLocation = startingLocation;
   }

   public void setAddFootstepMessageGenerator(boolean addFootstepMessageGenerator)
   {
      this.addFootstepMessageGenerator = addFootstepMessageGenerator;
   }

   public void setUseHeadingAndVelocityScript(boolean useHeadingAndVelocityScript)
   {
      this.useHeadingAndVelocityScript = useHeadingAndVelocityScript;
   }

   public void setCheatWithGroundHeightAtFootstep(boolean cheatWithGroundHeightAtFootstep)
   {
      this.cheatWithGroundHeightAtFootstep = cheatWithGroundHeightAtFootstep;
   }

   public void setHighLevelControllerStateFactory(HighLevelControllerStateFactory controllerStateFactory)
   {
      this.controllerStateFactory = controllerStateFactory;
   }

   public void setInitialSetup(DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> initialSetup)
   {
      this.initialSetup = initialSetup;
   }

   public void setWalkingScriptParameters(HeadingAndVelocityEvaluationScriptParameters walkingScriptParameters)
   {
      this.walkingScriptParameters = walkingScriptParameters;
   }

   public void setNetworkProcessorParameters(DRCNetworkModuleParameters networkProcessorParameters)
   {
      this.networkProcessorParameters = networkProcessorParameters;
   }

   public String getRobotName()
   {
      return robotName;
   }

   public Ros2Node getRos2Node()
   {
      return ros2Node;
   }

   @SuppressWarnings("unchecked")
   public void publishToController(Object message)
   {
      defaultControllerPublishers.get(message.getClass()).publish(message);
   }

   public <T> IHMCROS2Publisher<T> createPublisherForController(Class<T> messageType)
   {
      return createPublisher(messageType, ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName));
   }

   public <T> IHMCROS2Publisher<T> createPublisher(Class<T> messageType, MessageTopicNameGenerator generator)
   {
      return ROS2Tools.createPublisher(ros2Node, messageType, generator);
   }

   public <T> IHMCROS2Publisher<T> createPublisher(Class<T> messageType, String topicName)
   {
      return ROS2Tools.createPublisher(ros2Node, messageType, topicName);
   }

   public <T> void createSubscriberFromController(Class<T> messageType, ObjectConsumer<T> consumer)
   {
      createSubscriber(messageType, ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName), consumer);
   }

   public <T> void createSubscriber(Class<T> messageType, MessageTopicNameGenerator generator, ObjectConsumer<T> consumer)
   {
      ROS2Tools.createCallbackSubscription(ros2Node, messageType, generator, s -> consumer.consumeObject(s.takeNextData()));
   }

   public <T> void createSubscriber(Class<T> messageType, String topicName, ObjectConsumer<T> consumer)
   {
      ROS2Tools.createCallbackSubscription(ros2Node, messageType, topicName, s -> consumer.consumeObject(s.takeNextData()));
   }
}
