package us.ihmc.avatar.testTools.scs2;

import javafx.scene.control.Alert;
import javafx.scene.control.Alert.AlertType;
import javafx.scene.control.ButtonType;
import javafx.stage.Window;
import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.scs2.SCS2AvatarSimulation;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.conversion.YoGraphicConversionTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.behaviors.scripts.engine.ScriptBasedControllerCommandGenerator;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.ControllerFailureListener;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.simulation.SimulationTerminalCondition;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.RobotInterface;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools.VideoAndDataExporter;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoNamespace;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.registry.YoVariableHolder;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

import java.io.File;
import java.io.InputStream;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;

import static us.ihmc.robotics.Assert.assertTrue;
import static us.ihmc.robotics.Assert.fail;

public class SCS2AvatarTestingSimulation implements YoVariableHolder
{
   private final SCS2AvatarSimulation avatarSimulation;

   private final ControllerFailureListener exceptionOnFailureListener = fallingDirection ->
   {
      throw new ControllerFailureRuntimeException("Controller has failed!");
   };

   private ROS2Node ros2Node;
   @SuppressWarnings("rawtypes")
   private Map<Class<?>, ROS2PublisherBasics> defaultControllerPublishers;

   private final AtomicReference<Throwable> lastThrowable = new AtomicReference<>();

   private final AtomicBoolean isVisualizerGoingDown = new AtomicBoolean(false);

   private boolean createVideo = false;
   private boolean keepSCSUp = false;

   /**
    * Constructors for setting up a custom simulation environment for which the default factory isn't
    * suited.
    *
    * @param simulationConstructionSet the simulation to wrap.
    * @param robotModel                the robot model for enabling convenience methods. Can be
    *                                  {@code null}.
    * @param fullRobotModel            the robot to be associated as the controller robot for enabling
    *                                  convenience methods. Can be {@code null}.
    * @param yoGraphicsListRegistry    graphics to be displayed in the GUI. Can be {@code null}.
    */
   public SCS2AvatarTestingSimulation(SimulationConstructionSet2 simulationConstructionSet,
                                      DRCRobotModel robotModel,
                                      FullHumanoidRobotModel fullRobotModel,
                                      YoGraphicsListRegistry yoGraphicsListRegistry,
                                      SimulationTestingParameters parameters)
   {
      this(new SCS2AvatarSimulation());
      avatarSimulation.setSimulationConstructionSet(simulationConstructionSet);
      // Necessary to be able to restart the GUI during a series of tests.
      avatarSimulation.setSystemExitOnDestroy(false);
      avatarSimulation.setJavaFXThreadImplicitExit(false);

      avatarSimulation.setRobot(simulationConstructionSet.getPhysicsEngine().getRobots().get(0));
      if (robotModel != null)
         avatarSimulation.setRobotModel(robotModel);
      if (fullRobotModel != null)
         avatarSimulation.setFullHumanoidRobotModel(fullRobotModel);
      if (yoGraphicsListRegistry != null)
         simulationConstructionSet.addYoGraphics(YoGraphicConversionTools.toYoGraphicDefinitions(yoGraphicsListRegistry));

      if (parameters != null)
      {
         avatarSimulation.setShowGUI(parameters.getCreateGUI());
         simulationConstructionSet.initializeBufferSize(parameters.getDataBufferSize());
         setCreateVideo(parameters.getCreateSCSVideos());
         setKeepSCSUp(parameters.getKeepSCSUp());
      }
   }

   /**
    * Constructor used by the factory {@link SCS2AvatarTestingSimulationFactory}.
    *
    * @param avatarSimulation the simulation setup.
    */
   public SCS2AvatarTestingSimulation(SCS2AvatarSimulation avatarSimulation)
   {
      this.avatarSimulation = avatarSimulation;
      // Necessary to be able to restart the GUI during a series of tests.
      avatarSimulation.setSystemExitOnDestroy(false);
      avatarSimulation.setJavaFXThreadImplicitExit(false);
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
      start(true);
   }

   public void start(boolean cameraTracksPelvis)
   {
      getSimulationConstructionSet().addSimulationThrowableListener(lastThrowable::set);
      enableExceptionControllerFailure();

      avatarSimulation.start();

      if (getSimulationConstructionSet().isVisualizerEnabled())
      {
         getSimulationConstructionSet().waitUntilVisualizerFullyUp();
         getSimulationConstructionSet().addVisualizerShutdownListener(() -> isVisualizerGoingDown.set(true));

         setCameraDefaultRobotView();
         if (cameraTracksPelvis)
            requestCameraRigidBodyTracking(getRobot().getFloatingRootJoint().getSuccessor().getName());
      }

      // We park the simulation thread assuming that the calling test will need to run the simulation in their own thread to keep things synchronous.
      getSimulationConstructionSet().stopSimulationThread();
   }

   public void enableExceptionControllerFailure()
   {
      if (getHighLevelHumanoidControllerFactory() != null)
         getHighLevelHumanoidControllerFactory().attachControllerFailureListener(exceptionOnFailureListener);
   }

   public void disableExceptionControllerFailure()
   {
      if (getHighLevelHumanoidControllerFactory() != null)
         getHighLevelHumanoidControllerFactory().detachControllerFailureListener(exceptionOnFailureListener);
   }

   // Simulation controls:

   /**
    * Adds a terminal condition that will be used in the subsequent simulations to determine when to
    * stop the simulation.
    *
    * @param terminalCondition the new condition used to terminate future simulation.
    */
   public void addSimulationTerminalCondition(BooleanSupplier terminalCondition)
   {
      getSimulationConstructionSet().addExternalTerminalCondition(terminalCondition);
   }

   /**
    * Adds a terminal condition that will be used in the subsequent simulations to determine when to
    * stop the simulation.
    * <p>
    * The condition can be removed with
    * {@link #removeSimulationTerminalCondition(SimulationTerminalCondition)}.
    * </p>
    *
    * @param terminalCondition the new condition used to terminate future simulation.
    */
   public void addSimulationTerminalCondition(SimulationTerminalCondition terminalCondition)
   {
      getSimulationConstructionSet().addExternalTerminalCondition(terminalCondition);
   }

   /**
    * Removes a terminal simulation condition that was previously registered.
    *
    * @param terminalCondition the condition to remove.
    */
   public void removeSimulationTerminalCondition(SimulationTerminalCondition terminalCondition)
   {
      getSimulationConstructionSet().removeExternalTerminalCondition(terminalCondition);
   }

   /**
    * Simulate a single tick.
    * <p>
    * The method returns once the simulation is done.
    * </p>
    * <p>
    * If an exception is thrown during the simulation, it can be retrieved via
    * {@link #getLastThrownException()}.
    * </p>
    *
    * @return {@code true} if the simulation was successful, {@code false} if the simulation failed or
    *       the controller threw an exception.
    */
   public boolean simulateOneTickNow()
   {
      return simulateNow(1);
   }

   /**
    * Simulate for the duration of 1 record period (typically equal to 1 controller period).
    * <p>
    * The method returns once the simulation is done.
    * </p>
    * <p>
    * If an exception is thrown during the simulation, it can be retrieved via
    * {@link #getLastThrownException()}.
    * </p>
    *
    * @return {@code true} if the simulation was successful, {@code false} if the simulation failed or
    *       the controller threw an exception.
    */
   public boolean simulateOneBufferRecordPeriodNow()
   {
      return simulateNow(getSimulationConstructionSet().getBufferRecordTickPeriod());
   }

   /**
    * Simulate for the given duration.
    * <p>
    * The method returns once the simulation is done.
    * </p>
    * <p>
    * If an exception is thrown during the simulation, it can be retrieved via
    * {@link #getLastThrownException()}.
    * </p>
    *
    * @param duration desired simulation duration in seconds.
    * @return {@code true} if the simulation was successful, {@code false} if the simulation failed or
    *       the controller threw an exception.
    */
   public boolean simulateNow(double duration)
   {
      checkSimulationSessionAlive();
      lastThrowable.set(null);
      return getSimulationConstructionSet().simulateNow(duration);
   }

   /**
    * Simulate for the given number of ticks.
    * <p>
    * The method returns once the simulation is done.
    * </p>
    * <p>
    * If an exception is thrown during the simulation, it can be retrieved via
    * {@link #getLastThrownException()}.
    * </p>
    *
    * @param numberOfSimulationTicks desired number of simulation ticks.
    * @return {@code true} if the simulation was successful, {@code false} if the simulation failed or
    *       the controller threw an exception.
    */
   public boolean simulateNow(long numberOfSimulationTicks)
   {
      checkSimulationSessionAlive();
      lastThrowable.set(null);
      return getSimulationConstructionSet().simulateNow(numberOfSimulationTicks);
   }

   /**
    * Simulate indefinitely.
    * <p>
    * WARNING: This will block the calling thread indefinitely. The caller needs to add an additional
    * terminal condition beforehand to eventually stop the simulation.
    * </p>
    * <p>
    * The method returns once the simulation is done.
    * </p>
    * <p>
    * If an exception is thrown during the simulation, it can be retrieved via
    * {@link #getLastThrownException()}.
    * </p>
    *
    * @param duration desired simulation duration in seconds.
    * @return {@code true} if the simulation was successful, {@code false} if the simulation failed or
    *       the controller threw an exception.
    * @see #addSimulationTerminalCondition(BooleanSupplier)
    */
   public boolean simulateNow()
   {
      checkSimulationSessionAlive();
      lastThrowable.set(null);
      return getSimulationConstructionSet().simulateNow();
   }

   /**
    * Gets the throwable (if any) that was thrown during the last simulation.
    *
    * @return the exception thrown during the last simulation, or {@code null} if none was thrown.
    */
   public Throwable getLastThrownException()
   {
      return lastThrowable.get();
   }

   public void resetRobot(boolean simulateAfterReset)
   {
      checkSimulationSessionAlive();
      avatarSimulation.resetRobot(simulateAfterReset);
   }

   private void checkSimulationSessionAlive()
   {
      if (getSimulationConstructionSet().isSessionShutdown())
         throw new IllegalStateException("Simulation has been shutdown");
   }

   public void assertRobotsRootJointIsInBoundingBox(BoundingBox3DReadOnly boundingBox)
   {
      checkSimulationSessionAlive();
      RobotInterface robot = getSimulationConstructionSet().getPhysicsEngine().getRobots().get(0);
      FloatingJointBasics rootJoint = (FloatingJointBasics) robot.getRootBody().getChildrenJoints().get(0);
      boolean inside = boundingBox.isInsideInclusive(rootJoint.getJointPose().getPosition());
      if (!inside)
      {
         fail("Joint was at " + rootJoint.getJointPose().getPosition() + ". Expecting it to be inside boundingBox " + boundingBox);
      }
   }

   // Buffer controls:
   public void cropBuffer()
   {
      checkSimulationSessionAlive();
      getSimulationConstructionSet().cropBuffer();
   }

   public void setInPoint()
   {
      checkSimulationSessionAlive();
      getSimulationConstructionSet().setBufferInPoint();
   }

   public void setOutPoint()
   {
      checkSimulationSessionAlive();
      getSimulationConstructionSet().setBufferOutPoint();
   }

   public void gotoInPoint()
   {
      checkSimulationSessionAlive();
      getSimulationConstructionSet().gotoBufferInPoint();
   }

   public void gotoOutPoint()
   {
      checkSimulationSessionAlive();
      getSimulationConstructionSet().gotoBufferOutPoint();
   }

   public void stepBufferIndexForward()
   {
      checkSimulationSessionAlive();
      getSimulationConstructionSet().stepBufferIndexForward();
   }

   public void stepBufferIndexBackward()
   {
      checkSimulationSessionAlive();
      getSimulationConstructionSet().stepBufferIndexBackward();
   }

   // GUI controls:
   public void setCameraDefaultRobotView()
   {
      getAvatarSimulation().setCameraDefaultRobotView();
   }

   public void setCameraZoom(double distanceFromFocus)
   {
      getAvatarSimulation().setCameraZoom(distanceFromFocus);
   }

   /**
    * Sets the new focus point the camera is looking at.
    * <p>
    * The camera is rotated during this operation, its position remains unchanged.
    * </p>
    * <p>
    * Note that calling this method will cancel the camera tracking of a node.
    * </p>
    *
    * @param focus the new focus position.
    */
   public void setCameraFocusPosition(Point3DReadOnly focus)
   {
      getAvatarSimulation().setCameraFocusPosition(focus);
   }

   /**
    * Sets the new focus point the camera is looking at.
    * <p>
    * The camera is rotated during this operation, its position remains unchanged.
    * </p>
    * <p>
    * Note that calling this method will cancel the camera tracking of a node.
    * </p>
    *
    * @param x the x-coordinate of the new focus location.
    * @param y the y-coordinate of the new focus location.
    * @param z the z-coordinate of the new focus location.
    */
   public void setCameraFocusPosition(double x, double y, double z)
   {
      getAvatarSimulation().setCameraFocusPosition(x, y, z);
   }

   /**
    * Sets the new camera position.
    * <p>
    * The camera is rotated during this operation such that the focus point remains unchanged.
    * </p>
    *
    * @param position the new camera position.
    */
   public void setCameraPosition(Point3DReadOnly position)
   {
      getAvatarSimulation().setCameraPosition(position);
   }

   /**
    * Sets the new camera position.
    * <p>
    * The camera is rotated during this operation such that the focus point remains unchanged.
    * </p>
    *
    * @param x the x-coordinate of the new camera location.
    * @param y the y-coordinate of the new camera location.
    * @param z the z-coordinate of the new camera location.
    */
   public void setCameraPosition(double x, double y, double z)
   {
      getAvatarSimulation().setCameraPosition(x, y, z);
   }

   /**
    * Sets the camera configuration.
    * <p>
    * Note that calling this method will cancel the camera tracking of a node.
    * </p>
    *
    * @param cameraFocus    the new focus position (where the camera is looking at).
    * @param cameraPosition the new camera position.
    */
   public void setCamera(Point3DReadOnly cameraFocus, Point3DReadOnly cameraPosition)
   {
      getAvatarSimulation().setCamera(cameraFocus, cameraPosition);
   }

   public void requestCameraRigidBodyTracking(String rigidBodyName)
   {
      getAvatarSimulation().requestCameraRigidBodyTracking(rigidBodyName);
   }

   public void requestCameraRigidBodyTracking(String robotName, String rigidBodyName)
   {
      getAvatarSimulation().requestCameraRigidBodyTracking(robotName, rigidBodyName);
   }

   public void addStaticVisuals(Collection<? extends VisualDefinition> visualDefinitions)
   {
      checkSimulationSessionAlive();
      getSimulationConstructionSet().addStaticVisuals(visualDefinitions);
   }

   public void addYoGraphicDefinition(YoGraphicDefinition yoGraphicDefinition)
   {
      checkSimulationSessionAlive();
      getSimulationConstructionSet().addYoGraphic(yoGraphicDefinition);
   }

   public void addYoGraphicDefinition(String namespace, YoGraphicDefinition yoGraphicDefinition)
   {
      checkSimulationSessionAlive();
      getSimulationConstructionSet().addYoGraphic(namespace, yoGraphicDefinition);
   }

   public void addYoGraphicsListRegistry(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      checkSimulationSessionAlive();
      YoGraphicConversionTools.toYoGraphicDefinitions(yoGraphicsListRegistry).forEach(this::addYoGraphicDefinition);
   }

   public void addYoGraphic(YoGraphic yoGraphic)
   {
      checkSimulationSessionAlive();
      addYoGraphicDefinition(YoGraphicConversionTools.toYoGraphicDefinition(yoGraphic));
   }

   // Misc.
   public void finishTest()
   {
      finishTest(keepSCSUp);
   }

   public void finishTest(boolean waitUntilGUIIsDone)
   {
      if (waitUntilGUIIsDone && getSimulationConstructionSet() != null && getSimulationConstructionSet().isVisualizerEnabled()
          && getSimulationConstructionSet().getPrimaryGUIWindow() != null && !avatarSimulation.hasBeenDestroyed())
      {
         getSimulationConstructionSet().pause();
         getSimulationConstructionSet().startSimulationThread();

         JavaFXMissingTools.runAndWait(getClass(), () ->
         {
            if (!isVisualizerGoingDown.get())
            {
               Window primaryWindow = getSimulationConstructionSet().getPrimaryGUIWindow();
               primaryWindow.requestFocus();
               Alert alert = new Alert(AlertType.INFORMATION, "Test complete!", ButtonType.OK);
               SessionVisualizerIOTools.addSCSIconToDialog(alert);
               alert.initOwner(primaryWindow);
               JavaFXMissingTools.centerDialogInOwner(alert);
               alert.showAndWait();
            }
         });
         getSimulationConstructionSet().waitUntilVisualizerDown();
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
      ROS2PublisherBasics ROS2PublisherBasics = defaultControllerPublishers.get(message.getClass());
      ROS2PublisherBasics.publish(message);
   }

   public <T> ROS2PublisherBasics<T> createPublisherForController(Class<T> messageType)
   {
      return createPublisher(messageType, HumanoidControllerAPI.getInputTopic(getRobotModel().getSimpleRobotName()));
   }

   public <T> ROS2PublisherBasics<T> createPublisher(Class<T> messageType, ROS2Topic<?> generator)
   {
      return ros2Node.createPublisher(generator.withTypeName(messageType));
   }

   public <T> ROS2PublisherBasics<T> createPublisher(Class<T> messageType, String topicName)
   {
      return ros2Node.createPublisher(messageType, topicName);
   }

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
         scriptBasedControllerCommandGenerator = new ScriptBasedControllerCommandGenerator(getQueuedControllerCommands(), getControllerFullRobotModel());
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
   public void setDefaultControllerPublishers(Map<Class<?>, ROS2PublisherBasics> defaultControllerPublishers)
   {
      this.defaultControllerPublishers = defaultControllerPublishers;
   }

   public <T> void createSubscriberFromController(Class<T> messageType, ObjectConsumer<T> consumer)
   {
      createSubscriber(messageType, HumanoidControllerAPI.getOutputTopic(getRobotModel().getSimpleRobotName()), consumer);
   }

   public <T> void createSubscriber(Class<T> messageType, ROS2Topic<?> generator, ObjectConsumer<T> consumer)
   {
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, messageType, generator, s -> consumer.consumeObject(s.takeNextData()));
   }

   public <T> void createSubscriber(Class<T> messageType, String topicName, ObjectConsumer<T> consumer)
   {
      ros2Node.createSubscription(messageType, s -> consumer.consumeObject(s.takeNextData()), topicName);
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

   public HighLevelHumanoidControllerFactory getHighLevelHumanoidControllerFactory()
   {
      return avatarSimulation.getHighLevelHumanoidControllerFactory();
   }

   public HighLevelHumanoidControllerToolbox getHighLevelHumanoidControllerToolbox()
   {
      return getHighLevelHumanoidControllerFactory().getHighLevelHumanoidControllerToolbox();
   }

   public CommonHumanoidReferenceFrames getControllerReferenceFrames()
   {
      return getHighLevelHumanoidControllerToolbox().getReferenceFrames();
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
            if (getSimulationConstructionSet() == null || !getSimulationConstructionSet().isSimulating())
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
            if (getSimulationConstructionSet() == null || !getSimulationConstructionSet().isSimulating())
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

   public SCS2AvatarSimulation getAvatarSimulation()
   {
      return avatarSimulation;
   }

   public DRCRobotModel getRobotModel()
   {
      return avatarSimulation.getRobotModel();
   }

   public String getRobotName()
   {
      return getRobotModel().getSimpleRobotName();
   }

   public double getSimulationDT()
   {
      return getSimulationConstructionSet().getDT();
   }

   public Robot getRobot()
   {
      return avatarSimulation.getRobot();
   }

   public RobotDefinition getRobotDefinition()
   {
      return avatarSimulation.getRobotDefinition();
   }

   public SimulationConstructionSet2 getSimulationConstructionSet()
   {
      return avatarSimulation.getSimulationConstructionSet();
   }

   public double getSimulationTime()
   {
      return avatarSimulation.getSimulationConstructionSet().getTime().getValue();
   }

   public double getControllerTime()
   {
      return avatarSimulation.getHighLevelHumanoidControllerFactory().getHighLevelHumanoidControllerToolbox().getYoTime().getValue();
   }

   public double getTimePerRecordTick()
   {
      return getSimulationConstructionSet().getBufferRecordTimePeriod();
   }

   public void createBambooVideo(String simplifiedRobotModelName, int callStackHeight)
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

   public void createBambooVideo(String videoName)
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
            getSimulationConstructionSet().gotoBufferOutPoint();
         }

         @Override
         public File createVideo(String string)
         {
            File videoFile = new File(string);
            exportVideo(videoFile);
            return videoFile;
         }
      };
   }

   public void exportVideo(File videoFile)
   {
      getSimulationConstructionSet().startSimulationThread();
      getSimulationConstructionSet().exportVideo(videoFile);
   }

   public void addRegistry(YoRegistry registry)
   {
      getRootRegistry().addChild(registry);
   }

   public YoRegistry getRootRegistry()
   {
      return getSimulationConstructionSet().getRootRegistry();
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

   public static class ControllerFailureRuntimeException extends RuntimeException
   {
      private static final long serialVersionUID = 2279107051689445347L;

      public ControllerFailureRuntimeException(String message, Throwable cause)
      {
         super(message, cause);
      }

      public ControllerFailureRuntimeException(String message)
      {
         super(message);
      }
   }
}
