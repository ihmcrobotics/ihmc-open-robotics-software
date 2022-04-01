package us.ihmc.avatar.testTools.scs2;

import static us.ihmc.robotics.Assert.assertTrue;
import static us.ihmc.robotics.Assert.fail;

import java.io.File;
import java.io.InputStream;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import org.apache.commons.lang3.mutable.MutableInt;

import javafx.scene.control.Alert;
import javafx.scene.control.Alert.AlertType;
import javafx.scene.control.ButtonType;
import us.ihmc.avatar.scs2.SCS2AvatarSimulation;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.humanoidBehaviors.behaviors.scripts.engine.ScriptBasedControllerCommandGenerator;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.scs2.definition.state.interfaces.SixDoFJointStateBasics;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerControls;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.SimulationSessionControls;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.RobotInterface;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools.VideoAndDataExporter;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoNamespace;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.registry.YoVariableHolder;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class SCS2AvatarTestingSimulation implements YoVariableHolder
{
   private static final double CAMERA_PITCH_FROM_ROBOT = Math.toRadians(-15.0);
   private static final double CAMERA_YAW_FROM_ROBOT = Math.toRadians(15.0);
   private static final double CAMERA_DISTANCE_FROM_ROBOT = 6.0;

   private final SCS2AvatarSimulation avatarSimulation;

   private ROS2Node ros2Node;
   @SuppressWarnings("rawtypes")
   private Map<Class<?>, IHMCROS2Publisher> defaultControllerPublishers;

   private final SimulationSessionControls simulationSessionControls;
   private final AtomicReference<Throwable> lastThrowable = new AtomicReference<>();

   private SessionVisualizerControls sessionVisualizerControls;

   private boolean createVideo = false;
   private boolean keepSCSUp = false;

   public SCS2AvatarTestingSimulation(SCS2AvatarSimulation avatarSimulation)
   {
      this.avatarSimulation = avatarSimulation;
      simulationSessionControls = avatarSimulation.getSimulationSession().getSimulationSessionControls();
      simulationSessionControls.addSimulationThrowableListener(lastThrowable::set);

      AtomicBoolean controllerFailed = new AtomicBoolean(false);
      avatarSimulation.getHighLevelHumanoidControllerFactory().attachControllerFailureListener(fallingDirection -> controllerFailed.set(true));
      simulationSessionControls.addExternalTerminalCondition(() -> controllerFailed.get());
   }

   public void setCreateVideo(boolean createVideo)
   {
      this.createVideo = createVideo;
   }

   public void setKeepSCSUp(boolean keepSCSUp)
   {
      this.keepSCSUp = keepSCSUp;
   }

   public void start()
   {
      // Necessary to be able to restart the GUI during a series of tests.
      avatarSimulation.setSystemExitOnDestroy(false);
      avatarSimulation.setJavaFXThreadImplicitExit(false);

      avatarSimulation.start();
      sessionVisualizerControls = avatarSimulation.getSessionVisualizerControls();
      if (sessionVisualizerControls != null)
      {

         sessionVisualizerControls.waitUntilFullyUp();

         SixDoFJointStateBasics initialRootJointState = (SixDoFJointStateBasics) avatarSimulation.getRobotDefinition().getRootJointDefinitions().get(0)
                                                                                                 .getInitialJointState();
         initializeCamera(initialRootJointState.getOrientation(), initialRootJointState.getPosition());
         requestCameraRigidBodyTracking(avatarSimulation.getRobotModel().getSimpleRobotName(), getControllerFullRobotModel().getPelvis().getName());
      }
   }

   private void initializeCamera(Orientation3DReadOnly robotOrientation, Tuple3DReadOnly robotPosition)
   {
      Point3D focusPosition = new Point3D(robotPosition);
      Point3D cameraPosition = new Point3D(10, 0, 0);
      RotationMatrixTools.applyPitchRotation(CAMERA_PITCH_FROM_ROBOT, cameraPosition, cameraPosition);
      RotationMatrixTools.applyYawRotation(CAMERA_YAW_FROM_ROBOT, cameraPosition, cameraPosition);
      RotationMatrixTools.applyYawRotation(robotOrientation.getYaw(), cameraPosition, cameraPosition);
      cameraPosition.scale(CAMERA_DISTANCE_FROM_ROBOT / cameraPosition.distanceFromOrigin());
      cameraPosition.add(focusPosition);

      setCameraFocusPosition(focusPosition);
      setCameraPosition(cameraPosition);
   }

   // Simulation controls:

   public boolean simulateAndWait(double duration)
   {
      checkSimulationHasStarted();
      lastThrowable.set(null);
      return simulationSessionControls.simulateAndWait(duration);
   }

   public boolean simulateOneBufferRecordPeriodAndWait()
   {
      checkSimulationHasStarted();
      lastThrowable.set(null);
      return simulationSessionControls.simulateAndWait(getSimulationSession().getBufferRecordTickPeriod());
   }

   public Throwable getLastThrownException()
   {
      return lastThrowable.get();
   }

   public void resetRobot(boolean simulateAfterReset)
   {
      avatarSimulation.resetRobot(simulateAfterReset);
   }

   private void checkSimulationHasStarted()
   {
      if (!avatarSimulation.getSimulationSession().hasSessionStarted())
         throw new IllegalStateException("The simulation has not been started.");
   }

   public void assertRobotsRootJointIsInBoundingBox(BoundingBox3DReadOnly boundingBox)
   {
      checkSimulationHasStarted();
      RobotInterface robot = avatarSimulation.getSimulationSession().getPhysicsEngine().getRobots().get(0);
      FloatingJointBasics rootJoint = (FloatingJointBasics) robot.getRootBody().getChildrenJoints().get(0);
      boolean inside = boundingBox.isInsideInclusive(rootJoint.getJointPose().getPosition());
      if (!inside)
      {
         fail("Joint was at " + rootJoint.getJointPose().getPosition() + ". Expecting it to be inside boundingBox " + boundingBox);
      }
   }

   // Buffer controls:
   public void setBufferInPointIndexToCurrent()
   {
      simulationSessionControls.setBufferInPointIndexToCurrent();
   }

   public void setBufferOutPointIndexToCurrent()
   {
      simulationSessionControls.setBufferOutPointIndexToCurrent();
   }

   public void stepBufferIndexForward()
   {
      simulationSessionControls.stepBufferIndexForward();
   }

   public void stepBufferIndexBackward()
   {
      simulationSessionControls.stepBufferIndexBackward();
   }

   // GUI controls:
   public void setCameraZoom(double distanceFromFocus)
   {
      if (sessionVisualizerControls != null)
         sessionVisualizerControls.setCameraZoom(distanceFromFocus);
   }

   public void setCameraFocusPosition(Point3DReadOnly focus)
   {
      setCameraFocusPosition(focus.getX(), focus.getY(), focus.getZ());
   }

   public void setCameraFocusPosition(double x, double y, double z)
   {
      if (sessionVisualizerControls != null)
         sessionVisualizerControls.setCameraFocusPosition(x, y, z);
   }

   public void setCameraPosition(Point3DReadOnly position)
   {
      setCameraPosition(position.getX(), position.getY(), position.getZ());
   }

   public void setCameraPosition(double x, double y, double z)
   {
      if (sessionVisualizerControls != null)
         sessionVisualizerControls.setCameraPosition(x, y, z);
   }

   public void setCamera(Point3DReadOnly cameraFocus, Point3DReadOnly cameraPosition)
   {
      setCameraPosition(cameraPosition);
      setCameraFocusPosition(cameraFocus);
   }

   public void requestCameraRigidBodyTracking(String robotName, String rigidBodyName)
   {
      if (sessionVisualizerControls != null)
         sessionVisualizerControls.requestCameraRigidBodyTracking(robotName, rigidBodyName);
   }

   public void addStaticVisuals(Collection<? extends VisualDefinition> visualDefinitions)
   {
      if (sessionVisualizerControls != null)
         sessionVisualizerControls.addStaticVisuals(visualDefinitions);
   }

   public void addYoGraphicDefinition(YoGraphicDefinition yoGraphicDefinition)
   {
      if (sessionVisualizerControls != null)
         sessionVisualizerControls.addYoGraphic(yoGraphicDefinition);
   }

   public void addYoGraphicDefinition(String namespace, YoGraphicDefinition yoGraphicDefinition)
   {
      if (sessionVisualizerControls != null)
         sessionVisualizerControls.addYoGraphic(namespace, yoGraphicDefinition);
   }

   // Misc.
   public void finishTest()
   {
      finishTest(keepSCSUp);
   }

   public void finishTest(boolean waitUntilGUIIsDone)
   {
      if (waitUntilGUIIsDone && sessionVisualizerControls != null && !avatarSimulation.hasBeenDestroyed())
      {
         JavaFXMissingTools.runAndWait(getClass(), () -> new Alert(AlertType.INFORMATION, "Test complete!", ButtonType.OK).showAndWait());
         sessionVisualizerControls.waitUntilDown();
      }
      else
      {
         destroy();
      }
   }

   public void destroy()
   {
      if (ros2Node != null)
      {
         ros2Node.destroy();
         ros2Node = null;
      }

      avatarSimulation.destroy();
   }

   @SuppressWarnings({"unchecked", "rawtypes"})
   public void publishToController(Object message)
   {
      IHMCROS2Publisher ihmcros2Publisher = defaultControllerPublishers.get(message.getClass());
      ihmcros2Publisher.publish(message);
   }

   public <T> IHMCROS2Publisher<T> createPublisher(Class<T> messageType, ROS2Topic<?> generator)
   {
      return ROS2Tools.createPublisherTypeNamed(ros2Node, messageType, generator);
   }

   public <T> IHMCROS2Publisher<T> createPublisher(Class<T> messageType, String topicName)
   {
      return ROS2Tools.createPublisher(ros2Node, messageType, topicName);
   }

   //   private ScriptedFootstepGenerator scriptedFootstepGenerator;
   //
   //   public ScriptedFootstepGenerator getScriptedFootstepGenerator()
   //   {
   //      if (scriptedFootstepGenerator == null)
   //         scriptedFootstepGenerator = new ScriptedFootstepGenerator(avatarSimulation.getRobotModel().createFullRobotModel());
   //      return scriptedFootstepGenerator;
   //   }

   private ConcurrentLinkedQueue<Command<?, ?>> controllerCommands;

   public ConcurrentLinkedQueue<Command<?, ?>> getQueuedControllerCommands()
   {
      if (controllerCommands == null)
      {
         controllerCommands = new ConcurrentLinkedQueue<>();
         getHighLevelHumanoidControllerFactory().createQueuedControllerCommandGenerator(controllerCommands);
      }

      return controllerCommands;
   }

   private ScriptBasedControllerCommandGenerator scriptBasedControllerCommandGenerator;

   public ScriptBasedControllerCommandGenerator getScriptBasedControllerCommandGenerator()
   {
      if (scriptBasedControllerCommandGenerator == null)
      {
         FullHumanoidRobotModel fullRobotModel = getHighLevelHumanoidControllerFactory().getHighLevelHumanoidControllerToolbox().getFullRobotModel();
         scriptBasedControllerCommandGenerator = new ScriptBasedControllerCommandGenerator(getQueuedControllerCommands(), fullRobotModel);
      }
      return scriptBasedControllerCommandGenerator;
   }

   public void loadScriptFile(InputStream scriptInputStream, ReferenceFrame referenceFrame)
   {
      getScriptBasedControllerCommandGenerator().loadScriptFile(scriptInputStream, referenceFrame);
   }

   public ROS2Node getROS2Node()
   {
      return ros2Node;
   }

   public void setROS2Node(ROS2Node ros2Node)
   {
      this.ros2Node = ros2Node;
   }

   @SuppressWarnings("rawtypes")
   public void setDefaultControllerPublishers(Map<Class<?>, IHMCROS2Publisher> defaultControllerPublishers)
   {
      this.defaultControllerPublishers = defaultControllerPublishers;
   }

   public <T> void createSubscriberFromController(Class<T> messageType, ObjectConsumer<T> consumer)
   {
      createSubscriber(messageType, ROS2Tools.getControllerOutputTopic(avatarSimulation.getRobotModel().getSimpleRobotName()), consumer);
   }

   public <T> void createSubscriber(Class<T> messageType, ROS2Topic<?> generator, ObjectConsumer<T> consumer)
   {
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, messageType, generator, s -> consumer.consumeObject(s.takeNextData()));
   }

   public <T> void createSubscriber(Class<T> messageType, String topicName, ObjectConsumer<T> consumer)
   {
      ROS2Tools.createCallbackSubscription(ros2Node, messageType, topicName, s -> consumer.consumeObject(s.takeNextData()));
   }

   public YoRegistry getEstimatorRegistry()
   {
      return avatarSimulation.getEstimatorThread().getYoRegistry();
   }

   public YoRegistry getControllerRegistry()
   {
      return avatarSimulation.getControllerThread().getYoVariableRegistry();
   }

   public FullHumanoidRobotModel getControllerFullRobotModel()
   {
      return avatarSimulation.getControllerFullRobotModel();
   }

   public CommonHumanoidReferenceFrames getReferenceFrames()
   {
      HighLevelHumanoidControllerFactory momentumBasedControllerFactory = avatarSimulation.getHighLevelHumanoidControllerFactory();
      HighLevelHumanoidControllerToolbox highLevelHumanoidControllerToolbox = momentumBasedControllerFactory.getHighLevelHumanoidControllerToolbox();
      return highLevelHumanoidControllerToolbox.getReferenceFrames();
   }

   public HighLevelHumanoidControllerFactory getHighLevelHumanoidControllerFactory()
   {
      return avatarSimulation.getHighLevelHumanoidControllerFactory();
   }

   /**
    * For unit testing only
    */
   public void addRobotControllerOnControllerThread(RobotController controller)
   {
      avatarSimulation.addRobotControllerOnControllerThread(controller);
   }

   public void addDesiredICPContinuityAssertion(double maxICPPlanError)
   {
      final YoDouble desiredICPX = (YoDouble) findVariable("desiredICPX");
      final YoDouble desiredICPY = (YoDouble) findVariable("desiredICPY");

      final Point2D previousDesiredICP = new Point2D();
      final Point2D desiredICP = new Point2D();

      final int ticksToInitialize = 100;
      final MutableInt xTicks = new MutableInt(0);
      final MutableInt yTicks = new MutableInt(0);

      desiredICPX.addListener(new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
         {
            if (getSimulationSession() == null | getSimulationSession().getActiveMode() != SessionMode.RUNNING)
               return; // Do not perform this check if the sim is not running, so the user can scrub the data when sim is done.

            desiredICP.setX(desiredICPX.getDoubleValue());
            if (xTicks.getValue() > ticksToInitialize && yTicks.getValue() > ticksToInitialize)
            {
               assertTrue("ICP plan desired jumped from " + previousDesiredICP + " to " + desiredICP + " in one control DT.",
                          previousDesiredICP.distance(desiredICP) < maxICPPlanError);
            }
            previousDesiredICP.set(desiredICP);

            xTicks.setValue(xTicks.getValue() + 1);
         }
      });

      desiredICPY.addListener(new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
         {
            if (getSimulationSession() == null | getSimulationSession().getActiveMode() != SessionMode.RUNNING)
               return; // Do not perform this check if the sim is not running, so the user can scrub the data when sim is done.

            desiredICP.setY(desiredICPY.getDoubleValue());
            if (xTicks.getValue() > ticksToInitialize && yTicks.getValue() > ticksToInitialize)
            {
               assertTrue("ICP plan desired jumped from " + previousDesiredICP + " to " + desiredICP + " in one control DT.",
                          previousDesiredICP.distance(desiredICP) < maxICPPlanError);
            }
            previousDesiredICP.set(desiredICP);

            yTicks.setValue(yTicks.getValue() + 1);
         }
      });
   }

   public Robot getRobot()
   {
      return avatarSimulation.getRobot();
   }

   public SimulationSession getSimulationSession()
   {
      return avatarSimulation.getSimulationSession();
   }

   public double getSimulationTime()
   {
      return avatarSimulation.getSimulationSession().getTime().getValue();
   }

   public double getControllerTime()
   {
      return avatarSimulation.getHighLevelHumanoidControllerFactory().getHighLevelHumanoidControllerToolbox().getYoTime().getValue();
   }

   public double getTimePerRecordTick()
   {
      return getSimulationSession().getBufferRecordTimePeriod();
   }

   public void createVideo(String simplifiedRobotModelName, int callStackHeight)
   {
      if (createVideo)
      {
         BambooTools.createVideoWithDateTimeClassMethodAndShareOnSharedDriveIfAvailable(simplifiedRobotModelName,
                                                                                        createBambooToolsVideoAndDataExporter(),
                                                                                        callStackHeight,
                                                                                        avatarSimulation.getShowGUI());
      }
      else
      {
         LogTools.info("Skipping video generation.");
      }
   }

   public void createVideo(String videoName)
   {
      if (createVideo)
      {
         BambooTools.createVideoWithDateTimeAndStoreInDefaultDirectory(createBambooToolsVideoAndDataExporter(), videoName, avatarSimulation.getShowGUI());
      }
      else
      {
         LogTools.info("Skipping video generation.");
      }
   }

   private VideoAndDataExporter createBambooToolsVideoAndDataExporter()
   {
      return new VideoAndDataExporter()
      {

         @Override
         public void writeData(File dataFile)
         {
            // TODO Implement me
         }

         @Override
         public void gotoOutPointNow()
         {
            simulationSessionControls.setBufferCurrentIndexToOutPoint();
         }

         @Override
         public File createVideo(String string)
         {
            File videoFile = new File(string);
            sessionVisualizerControls.exportVideo(videoFile);
            return videoFile;
         }
      };
   }

   public void addRegistry(YoRegistry registry)
   {
      getRootRegistry().addChild(registry);
   }

   public YoRegistry getRootRegistry()
   {
      return getSimulationSession().getRootRegistry();
   }

   @Override
   public YoVariable findVariable(String namespace, String name)
   {
      return getRootRegistry().findVariable(namespace, name);
   }

   @Override
   public List<YoVariable> findVariables(String namespaceEnding, String name)
   {
      return getRootRegistry().findVariables(namespaceEnding, name);
   }

   @Override
   public List<YoVariable> findVariables(YoNamespace namespace)
   {
      return getRootRegistry().findVariables(namespace);
   }

   @Override
   public boolean hasUniqueVariable(String namespaceEnding, String name)
   {
      return getRootRegistry().hasUniqueVariable(namespaceEnding, name);
   }

   @Override
   public List<YoVariable> getVariables()
   {
      return getRootRegistry().getVariables();
   }
}
