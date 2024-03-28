package us.ihmc.avatar.testTools.scs2;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.Optional;

import toolbox_msgs.msg.dds.BehaviorControlModePacket;
import controller_msgs.msg.dds.CapturabilityBasedStatus;
import toolbox_msgs.msg.dds.HumanoidBehaviorTypePacket;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.IHMCHumanoidBehaviorManager;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.dispatcher.BehaviorControlModeSubscriber;
import us.ihmc.humanoidBehaviors.dispatcher.BehaviorDispatcher;
import us.ihmc.humanoidBehaviors.dispatcher.HumanoidBehaviorTypeSubscriber;
import us.ihmc.humanoidBehaviors.utilities.CapturePointUpdatable;
import us.ihmc.humanoidBehaviors.utilities.StopThreadUpdatable;
import us.ihmc.humanoidBehaviors.utilities.TimeBasedStopThreadUpdatable;
import us.ihmc.humanoidBehaviors.utilities.TrajectoryBasedStopThreadUpdatable;
import us.ihmc.humanoidBehaviors.utilities.WristForceSensorFilteredUpdatable;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.humanoidRobotics.communication.subscribers.CapturabilityBasedStatusSubscriber;
import us.ihmc.humanoidRobotics.communication.subscribers.HumanoidRobotDataReceiver;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.yoVariables.registry.YoNamespace;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.registry.YoVariableHolder;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

@SuppressWarnings("rawtypes")
public class SCS2BehaviorTestHelper implements YoVariableHolder
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoDouble yoTimeRobot;
   private final YoDouble yoTimeBehaviorDispatcher;
   private final YoDouble yoTimeLastFullRobotModelUpdate;

   private final DRCRobotModel drcRobotModel;
   private final FullHumanoidRobotModel fullRobotModel;

   private final HumanoidRobotDataReceiver robotDataReceiver;
   private final HumanoidReferenceFrames referenceFrames;

   private final ArrayList<Updatable> updatables = new ArrayList<Updatable>();
   private final CapturePointUpdatable capturePointUpdatable;
   private final SideDependentList<WristForceSensorFilteredUpdatable> wristForceSensorUpdatables;

   private final BehaviorDispatcher behaviorDispatcher;

   private final ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.INTRAPROCESS, "ihmc_behavior_test_helper");
   private final ROS2PublisherBasics<HumanoidBehaviorTypePacket> humanoidBehabiorTypePublisher;
   private final String robotName;
   private SCS2AvatarTestingSimulation avatarTestingSimulation;

   public SCS2BehaviorTestHelper(SCS2AvatarTestingSimulation avatarTestingSimulation)
   {
      this.avatarTestingSimulation = avatarTestingSimulation;
      yoTimeRobot = avatarTestingSimulation.getSimulationConstructionSet().getTime();
      yoTimeBehaviorDispatcher = new YoDouble("yoTimeBehaviorDispatcher", registry);

      this.drcRobotModel = avatarTestingSimulation.getRobotModel();
      robotName = drcRobotModel.getSimpleRobotName();
      this.fullRobotModel = drcRobotModel.createFullRobotModel(false);
      yoTimeLastFullRobotModelUpdate = new YoDouble("yoTimeRobotModelUpdate", registry);

      ForceSensorDataHolder forceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(fullRobotModel.getForceSensorDefinitions()));
      robotDataReceiver = new HumanoidRobotDataReceiver(fullRobotModel, forceSensorDataHolder);
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    RobotConfigurationData.class,
                                                    HumanoidControllerAPI.getOutputTopic(robotName),
                                                    s -> robotDataReceiver.receivedPacket(s.takeNextData()));

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      capturePointUpdatable = createCapturePointUpdateable(yoGraphicsListRegistry);
      updatables.add(capturePointUpdatable);

      if (drcRobotModel.getSensorInformation().getWristForceSensorNames() != null && !drcRobotModel.getSensorInformation().getWristForceSensorNames().isEmpty())
      {
         wristForceSensorUpdatables = createWristForceSensorUpdateables();
         updatables.add(wristForceSensorUpdatables.get(RobotSide.LEFT));
         updatables.add(wristForceSensorUpdatables.get(RobotSide.RIGHT));
      }
      else
      {
         wristForceSensorUpdatables = null;
      }

      behaviorDispatcher = setupBehaviorDispatcher(fullRobotModel, ros2Node, robotDataReceiver, yoGraphicsListRegistry);

      referenceFrames = robotDataReceiver.getReferenceFrames();
      humanoidBehabiorTypePublisher = ros2Node.createPublisher(ROS2Tools.typeNamedTopic(HumanoidBehaviorTypePacket.class).withTopic(IHMCHumanoidBehaviorManager.getInputTopic(robotName)));
   }

   public ROS2Node getROS2Node()
   {
      return ros2Node;
   }

   public FullHumanoidRobotModel getSDFFullRobotModel()
   {
      boolean robotModelIsUpToDate = yoTimeRobot.getDoubleValue() == yoTimeLastFullRobotModelUpdate.getDoubleValue();

      if (!robotModelIsUpToDate)
      {
         updateRobotModel();
      }

      return fullRobotModel;
   }

   public HumanoidReferenceFrames getReferenceFrames()
   {
      return referenceFrames;
   }

   public BehaviorDispatcher getBehaviorDisptacher()
   {
      return behaviorDispatcher;
   }

   public HumanoidRobotDataReceiver getRobotDataReceiver()
   {
      return robotDataReceiver;
   }

   public YoDouble getYoTime()
   {
      return yoTimeRobot;
   }

   public CapturePointUpdatable getCapturePointUpdatable()
   {
      return capturePointUpdatable;
   }

   public Optional<WristForceSensorFilteredUpdatable> getWristForceSensorUpdatable(RobotSide robotSide)
   {
      if (wristForceSensorUpdatables == null)
         return Optional.empty();

      return Optional.ofNullable(wristForceSensorUpdatables.get(robotSide));
   }

   public Optional<SideDependentList<WristForceSensorFilteredUpdatable>> getWristForceSensorUpdatableSideDependentList()
   {
      return Optional.ofNullable(wristForceSensorUpdatables);
   }

   public void updateRobotModel()
   {
      yoTimeLastFullRobotModelUpdate.set(yoTimeRobot.getDoubleValue());
      robotDataReceiver.updateRobotModel();
   }

   @SuppressWarnings("unchecked")
   public void dispatchBehavior(AbstractBehavior behaviorToTest)
   {
      HumanoidBehaviorType testBehaviorType = HumanoidBehaviorType.TEST;
      behaviorDispatcher.addBehavior(testBehaviorType, behaviorToTest);

      behaviorDispatcher.start();

      HumanoidBehaviorTypePacket requestTestBehaviorPacket = HumanoidMessageTools.createHumanoidBehaviorTypePacket(testBehaviorType);
      humanoidBehabiorTypePublisher.publish(requestTestBehaviorPacket);

      boolean success = avatarTestingSimulation.simulateNow(1.0);
      assertTrue("Caught an exception when testing the behavior, the robot probably fell.", success);
   }

   @SuppressWarnings("unchecked")
   public void sendBehaviorToDispatcher(AbstractBehavior behaviorToTest)
   {
      HumanoidBehaviorType testBehaviorType = HumanoidBehaviorType.TEST;
      behaviorDispatcher.addBehavior(testBehaviorType, behaviorToTest);

      HumanoidBehaviorTypePacket requestTestBehaviorPacket = HumanoidMessageTools.createHumanoidBehaviorTypePacket(testBehaviorType);
      humanoidBehabiorTypePublisher.publish(requestTestBehaviorPacket);
   }

   private BehaviorDispatcher setupBehaviorDispatcher(FullRobotModel fullRobotModel,
                                                      ROS2Node ros2Node,
                                                      HumanoidRobotDataReceiver robotDataReceiver,
                                                      YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      BehaviorControlModeSubscriber desiredBehaviorControlSubscriber = new BehaviorControlModeSubscriber();
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    BehaviorControlModePacket.class,
                                                    IHMCHumanoidBehaviorManager.getInputTopic(robotName),
                                                    s -> desiredBehaviorControlSubscriber.receivedPacket(s.takeNextData()));

      HumanoidBehaviorTypeSubscriber desiredBehaviorSubscriber = new HumanoidBehaviorTypeSubscriber();
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    HumanoidBehaviorTypePacket.class,
                                                    IHMCHumanoidBehaviorManager.getInputTopic(robotName),
                                                    s -> desiredBehaviorSubscriber.receivedPacket(s.takeNextData()));

      YoVariableServer yoVariableServer = null;
      yoGraphicsListRegistry.setYoGraphicsUpdatedRemotely(false);

      BehaviorDispatcher<HumanoidBehaviorType> ret = new BehaviorDispatcher<>(robotName,
                                                                              yoTimeBehaviorDispatcher,
                                                                              robotDataReceiver,
                                                                              desiredBehaviorControlSubscriber,
                                                                              desiredBehaviorSubscriber,
                                                                              ros2Node,
                                                                              yoVariableServer,
                                                                              HumanoidBehaviorType.class,
                                                                              HumanoidBehaviorType.STOP,
                                                                              registry,
                                                                              yoGraphicsListRegistry);

      ret.addUpdatable(capturePointUpdatable);

      if (wristForceSensorUpdatables != null)
      {
         ret.addUpdatable(wristForceSensorUpdatables.get(RobotSide.LEFT));
         ret.addUpdatable(wristForceSensorUpdatables.get(RobotSide.RIGHT));
      }

      ret.finalizeStateMachine();

      return ret;
   }

   private SideDependentList<WristForceSensorFilteredUpdatable> createWristForceSensorUpdateables()
   {
      SideDependentList<WristForceSensorFilteredUpdatable> ret = new SideDependentList<WristForceSensorFilteredUpdatable>();

      for (RobotSide robotSide : RobotSide.values)
      {
         WristForceSensorFilteredUpdatable wristSensorUpdatable = new WristForceSensorFilteredUpdatable(drcRobotModel.getSimpleRobotName(),
                                                                                                        robotSide,
                                                                                                        fullRobotModel,
                                                                                                        drcRobotModel.getSensorInformation(),
                                                                                                        robotDataReceiver.getForceSensorDataHolder(),
                                                                                                        IHMCHumanoidBehaviorManager.BEHAVIOR_YO_VARIABLE_SERVER_DT,
                                                                                                        ros2Node,
                                                                                                        registry);

         ret.put(robotSide, wristSensorUpdatable);
      }

      return ret;
   }

   private CapturePointUpdatable createCapturePointUpdateable(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      CapturabilityBasedStatusSubscriber capturabilityBasedStatusSubsrciber = new CapturabilityBasedStatusSubscriber();
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    CapturabilityBasedStatus.class,
                                                    HumanoidControllerAPI.getOutputTopic(robotName),
                                                    s -> capturabilityBasedStatusSubsrciber.receivedPacket(s.takeNextData()));

      CapturePointUpdatable ret = new CapturePointUpdatable(capturabilityBasedStatusSubsrciber, yoGraphicsListRegistry, registry);

      return ret;
   }

   public void finishTest()
   {
      if (avatarTestingSimulation != null)
      {
         avatarTestingSimulation.finishTest();
         avatarTestingSimulation = null;
      }

      if (behaviorDispatcher != null)
      {
         behaviorDispatcher.closeAndDispose();
      }
   }

   public boolean executeBehaviorsSimulateAndBlockAndCatchExceptions(final SideDependentList<AbstractBehavior> behaviors, double simulationRunTime)

   {
      ArrayList<AbstractBehavior> behaviorArrayList = new ArrayList<AbstractBehavior>();

      for (RobotSide robotSide : RobotSide.values)
      {
         behaviorArrayList.add(behaviors.get(robotSide));
      }

      boolean ret = executeBehaviorsSimulateAndBlockAndCatchExceptions(behaviorArrayList, simulationRunTime);
      return ret;
   }

   public boolean executeBehaviorsSimulateAndBlockAndCatchExceptions(final ArrayList<AbstractBehavior> behaviors, double simulationRunTime)

   {
      BehaviorRunner behaviorRunner = startNewBehaviorRunnerThread(behaviors);

      boolean ret = avatarTestingSimulation.simulateNow(simulationRunTime);
      behaviorRunner.closeAndDispose();

      return ret;
   }

   public StopThreadUpdatable executeBehaviorPauseAndResumeOrStop(final AbstractBehavior behavior,
                                                                  double pausePercent,
                                                                  double pauseDuration,
                                                                  double stopPercent,
                                                                  FramePose3D poseAtTrajectoryEnd,
                                                                  ReferenceFrame frameToKeepTrackOf)

   {
      StopThreadUpdatable stopThreadUpdatable = new TrajectoryBasedStopThreadUpdatable(robotDataReceiver,
                                                                                       behavior,
                                                                                       pausePercent,
                                                                                       pauseDuration,
                                                                                       stopPercent,
                                                                                       poseAtTrajectoryEnd,
                                                                                       frameToKeepTrackOf);

      boolean success = executeBehaviorPauseAndResumeOrStop(behavior, stopThreadUpdatable);
      assertTrue(success);

      return stopThreadUpdatable;
   }

   public StopThreadUpdatable executeBehaviorPauseAndResumeOrStop(final AbstractBehavior behavior,
                                                                  double pauseTime,
                                                                  double pauseDuration,
                                                                  double stopTime,
                                                                  ReferenceFrame frameToKeepTrackOf)

   {
      StopThreadUpdatable stopThreadUpdatable = new TimeBasedStopThreadUpdatable(robotDataReceiver,
                                                                                 behavior,
                                                                                 pauseTime,
                                                                                 pauseDuration,
                                                                                 stopTime,
                                                                                 frameToKeepTrackOf);

      boolean success = executeBehaviorPauseAndResumeOrStop(behavior, stopThreadUpdatable);
      assertTrue(success);

      return stopThreadUpdatable;
   }

   public boolean executeBehaviorPauseAndResumeOrStop(AbstractBehavior behavior, StopThreadUpdatable stopThreadUpdatable)

   {
      StoppableBehaviorRunner behaviorRunner = new StoppableBehaviorRunner(behavior, stopThreadUpdatable);
      Thread behaviorThread = new Thread(behaviorRunner);
      behaviorThread.start();

      boolean success = true;
      while (!stopThreadUpdatable.shouldBehaviorRunnerBeStopped() && success)
      {
         success = avatarTestingSimulation.simulateNow(1.0);
      }
      behaviorRunner.closeAndDispose();

      assertTrue(success);

      return success;
   }

   public boolean executeBehaviorSimulateAndBlockAndCatchExceptions(final AbstractBehavior behavior, double simulationRunTime)

   {
      BehaviorRunner behaviorRunner = startNewBehaviorRunnerThread(behavior);

      boolean ret = avatarTestingSimulation.simulateNow(simulationRunTime);
      behaviorRunner.closeAndDispose();

      return ret;
   }

   public boolean executeBehaviorUntilDone(final AbstractBehavior behavior)
   {
      BehaviorRunner behaviorRunner = startNewBehaviorRunnerThread(behavior);

      boolean success = true;
      while (!behavior.isDone() && success)
      {
         success = avatarTestingSimulation.simulateNow(1.0);
      }

      behaviorRunner.closeAndDispose();

      return success;
   }

   public boolean executeBehaviorUntilDoneUsingBehaviorDispatcher(final AbstractBehavior behavior)
   {
      behaviorDispatcher.start();

      boolean success = true;
      success = avatarTestingSimulation.simulateNow(0.1);
      sendBehaviorToDispatcher(behavior);

      while (!behavior.isDone() && success)
      {
         success = avatarTestingSimulation.simulateNow(1.0);
      }

      return success;
   }

   private BehaviorRunner startNewBehaviorRunnerThread(final ArrayList<AbstractBehavior> behaviors)
   {
      BehaviorRunner behaviorRunner = new BehaviorRunner(behaviors);
      Thread behaviorThread = new Thread(behaviorRunner);
      behaviorThread.start();

      return behaviorRunner;
   }

   private BehaviorRunner startNewBehaviorRunnerThread(final AbstractBehavior behavior)
   {
      BehaviorRunner behaviorRunner = new BehaviorRunner(behavior);
      Thread behaviorThread = new Thread(behaviorRunner);
      behaviorThread.start();

      return behaviorRunner;
   }

   private class BehaviorRunner implements Runnable
   {
      protected boolean isRunning = true;
      protected final ArrayList<AbstractBehavior> behaviors;

      public BehaviorRunner(AbstractBehavior behavior)
      {
         this.behaviors = new ArrayList<AbstractBehavior>();
         this.behaviors.add(behavior);
      }

      public BehaviorRunner(ArrayList<AbstractBehavior> behaviors)
      {
         this.behaviors = behaviors;

      }

      public void run()
      {
         while (isRunning)
         {
            robotDataReceiver.updateRobotModel();

            for (AbstractBehavior behavior : behaviors)
            {
               behavior.doControl();
            }
            for (Updatable updatable : updatables)
            {
               updatable.update(yoTimeRobot.getDoubleValue());
            }

            ThreadTools.sleep(1);
         }
      }

      public void closeAndDispose()
      {
         isRunning = false;
      }
   }

   private class StoppableBehaviorRunner extends BehaviorRunner
   {
      private final StopThreadUpdatable stopThreadUpdatable;

      private BehaviorControlModeEnum currentControlMode = BehaviorControlModeEnum.RESUME;

      public StoppableBehaviorRunner(AbstractBehavior behavior, StopThreadUpdatable stopThreadUpdatable)
      {
         super(behavior);
         this.stopThreadUpdatable = stopThreadUpdatable;
      }

      public void run()
      {
         while (isRunning)
         {
            robotDataReceiver.updateRobotModel();

            for (AbstractBehavior behavior : behaviors)
            {
               behavior.doControl();
            }

            for (Updatable updatable : updatables)
            {
               updatable.update(yoTimeRobot.getDoubleValue());
            }

            stopThreadUpdatable.update(yoTimeRobot.getDoubleValue());

            BehaviorControlModeEnum requestedControlMode = stopThreadUpdatable.getRequestedBehaviorControlMode();

            if (stopThreadUpdatable.shouldBehaviorRunnerBeStopped())
            {
               LogTools.debug("Stopping Thread!");
               isRunning = false;
            }
            else if (requestedControlMode.equals(BehaviorControlModeEnum.PAUSE) && !currentControlMode.equals(BehaviorControlModeEnum.PAUSE))
            {
               for (AbstractBehavior behavior : behaviors)
               {
                  behavior.pause();
               }
               currentControlMode = BehaviorControlModeEnum.PAUSE;
            }
            else if (requestedControlMode.equals(BehaviorControlModeEnum.STOP) && !currentControlMode.equals(BehaviorControlModeEnum.STOP))
            {
               for (AbstractBehavior behavior : behaviors)
               {
                  behavior.abort();
               }
               currentControlMode = BehaviorControlModeEnum.STOP;
            }
            else if (requestedControlMode.equals(BehaviorControlModeEnum.RESUME) && !currentControlMode.equals(BehaviorControlModeEnum.RESUME))
            {
               for (AbstractBehavior behavior : behaviors)
               {
                  behavior.resume();
               }
               currentControlMode = BehaviorControlModeEnum.RESUME;
            }

            ThreadTools.sleep(1);
         }
      }
   }

   public FullHumanoidRobotModel getControllerFullRobotModel()
   {
      return avatarTestingSimulation.getControllerFullRobotModel();
   }

   public boolean simulateNow(double duration)
   {
      return avatarTestingSimulation.simulateNow(duration);
   }

   public String getRobotName()
   {
      return avatarTestingSimulation.getRobotName();
   }

   public Robot getRobot()
   {
      return avatarTestingSimulation.getRobot();
   }

   public HighLevelHumanoidControllerFactory getHighLevelHumanoidControllerFactory()
   {
      return avatarTestingSimulation.getHighLevelHumanoidControllerFactory();
   }

   public YoRegistry getRootRegistry()
   {
      return avatarTestingSimulation.getRootRegistry();
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

   public void addStaticVisuals(Collection<? extends VisualDefinition> visualDefinitions)
   {
      avatarTestingSimulation.addStaticVisuals(visualDefinitions);
   }

   public void setCamera(Point3DReadOnly cameraFocus, Point3DReadOnly cameraPosition)
   {
      avatarTestingSimulation.setCamera(cameraFocus, cameraPosition);
   }
}
