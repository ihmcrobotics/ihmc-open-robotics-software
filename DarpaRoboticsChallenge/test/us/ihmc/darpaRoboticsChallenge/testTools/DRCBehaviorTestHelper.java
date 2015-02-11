package us.ihmc.darpaRoboticsChallenge.testTools;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.communication.NetworkProcessor;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoPacketCommunicator;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorControlModePacket;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorControlModePacket.HumanoidBehaviorControlModeEnum;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorTypePacket;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.communication.subscribers.CapturabilityBasedStatusSubscriber;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.darpaRoboticsChallenge.DRCStartingLocation;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.environment.CommonAvatarEnvironmentInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.humanoidBehaviors.IHMCHumanoidBehaviorManager;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.humanoidBehaviors.dispatcher.BehaviorDisptacher;
import us.ihmc.humanoidBehaviors.dispatcher.HumanoidBehaviorControlModeSubscriber;
import us.ihmc.humanoidBehaviors.dispatcher.HumanoidBehaviorTypeSubscriber;
import us.ihmc.humanoidBehaviors.utilities.CapturePointUpdatable;
import us.ihmc.humanoidBehaviors.utilities.StopThreadUpdatable;
import us.ihmc.humanoidBehaviors.utilities.TimeBasedStopThreadUpdatable;
import us.ihmc.humanoidBehaviors.utilities.TrajectoryBasedStopThreadUpdatable;
import us.ihmc.humanoidBehaviors.utilities.WristForceSensorFilteredUpdatable;
import us.ihmc.robotDataCommunication.YoVariableServer;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.io.printing.SysoutTool;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

public class DRCBehaviorTestHelper extends DRCSimulationTestHelper
{
   private final boolean DEBUG = false;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleYoVariable yoTimeRobot;
   private final DoubleYoVariable yoTimeBehaviorDispatcher;
   private final DoubleYoVariable yoTimeLastFullRobotModelUpdate;

   private final DRCRobotModel drcRobotModel;
   private final FullRobotModel fullRobotModel;

   private final NetworkProcessor networkProcessor;
   private final KryoPacketCommunicator networkObjectCommunicator;
   private final KryoPacketCommunicator controllerCommunicator;
   private final KryoLocalPacketCommunicator behaviorCommunicator;
   private final BehaviorCommunicationBridge behaviorCommunicationBridge;
   private final RobotDataReceiver robotDataReceiver;
   private final ReferenceFrames referenceFrames;

   private final ArrayList<Updatable> updatables = new ArrayList<Updatable>();
   private final CapturePointUpdatable capturePointUpdatable;
   private final SideDependentList<WristForceSensorFilteredUpdatable> wristForceSensorUpdatables;

   private final BehaviorDisptacher behaviorDispatcher;

   public DRCBehaviorTestHelper(String name, String scriptFileName, DRCStartingLocation selectedLocation,
         SimulationTestingParameters simulationTestingParameters, DRCRobotModel robotModel, KryoPacketCommunicator networkObjectCommunicator,
         KryoPacketCommunicator controllerCommunicator)
   {
      this(new DRCDemo01NavigationEnvironment(), networkObjectCommunicator, name, scriptFileName, selectedLocation, simulationTestingParameters, robotModel,
            controllerCommunicator);
   }

   public DRCBehaviorTestHelper(CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface, String name, String scriptFileName,
         DRCStartingLocation selectedLocation, SimulationTestingParameters simulationTestingParameters, DRCRobotModel robotModel,
         KryoPacketCommunicator networkObjectCommunicator, KryoPacketCommunicator controllerCommunicator)
   {
      this(commonAvatarEnvironmentInterface, networkObjectCommunicator, name, scriptFileName, selectedLocation, simulationTestingParameters, robotModel,
            controllerCommunicator);
   }

   public DRCBehaviorTestHelper(CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface, KryoPacketCommunicator networkObjectCommunicator,
         String name, String scriptFileName, DRCStartingLocation selectedLocation, SimulationTestingParameters simulationTestingParameters,
        DRCRobotModel robotModel, KryoPacketCommunicator controllerCommunicator)
   {
      super(commonAvatarEnvironmentInterface, controllerCommunicator, name, scriptFileName, selectedLocation, simulationTestingParameters,
            false, robotModel);

      yoTimeRobot = getRobot().getYoTime();
      yoTimeBehaviorDispatcher = new DoubleYoVariable("yoTimeBehaviorDispatcher", registry);

      this.drcRobotModel = robotModel;
      this.fullRobotModel = robotModel.createFullRobotModel();
      yoTimeLastFullRobotModelUpdate = new DoubleYoVariable("yoTimeRobotModelUpdate", registry);

      this.networkObjectCommunicator = networkObjectCommunicator;
      this.controllerCommunicator = controllerCommunicator;

      behaviorCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(),
            PacketDestination.BEHAVIOR_MODULE.ordinal(), "behvaiorCommunicator");
      
      networkProcessor = new NetworkProcessor();
      networkProcessor.attachPacketCommunicator(networkObjectCommunicator);
      networkProcessor.attachPacketCommunicator(controllerCommunicator);
      networkProcessor.attachPacketCommunicator(behaviorCommunicator);
      
      
      
      behaviorCommunicationBridge = new BehaviorCommunicationBridge(behaviorCommunicator, registry);

      ForceSensorDataHolder forceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(fullRobotModel.getForceSensorDefinitions()));
      robotDataReceiver = new RobotDataReceiver(fullRobotModel, forceSensorDataHolder);
      controllerCommunicator.attachListener(RobotConfigurationData.class, robotDataReceiver);

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      capturePointUpdatable = createCapturePointUpdateable(yoGraphicsListRegistry);
      wristForceSensorUpdatables = createWristForceSensorUpdateables();
      updatables.add(capturePointUpdatable);
      updatables.add(wristForceSensorUpdatables.get(RobotSide.LEFT));
      updatables.add(wristForceSensorUpdatables.get(RobotSide.RIGHT));

      behaviorDispatcher = setupBehaviorDispatcher(fullRobotModel, behaviorCommunicator, robotDataReceiver, yoGraphicsListRegistry);

      referenceFrames = robotDataReceiver.getReferenceFrames();
   }

   public FullRobotModel getFullRobotModel()
   {
      boolean robotModelIsUpToDate = yoTimeRobot.getDoubleValue() == yoTimeLastFullRobotModelUpdate.getDoubleValue();

      if (!robotModelIsUpToDate)
      {
         updateRobotModel();
      }

      return fullRobotModel;
   }

   public ReferenceFrames getReferenceFrames()
   {
      return referenceFrames;
   }

   public BehaviorDisptacher getBehaviorDisptacher()
   {
      return behaviorDispatcher;
   }

   public RobotDataReceiver getRobotDataReceiver()
   {
      return robotDataReceiver;
   }

   public DoubleYoVariable getYoTime()
   {
      return yoTimeRobot;
   }

   public CapturePointUpdatable getCapturePointUpdatable()
   {
      return capturePointUpdatable;
   }

   public WristForceSensorFilteredUpdatable getWristForceSensorUpdatable(RobotSide robotSide)
   {
      return wristForceSensorUpdatables.get(robotSide);
   }

   public BehaviorCommunicationBridge getBehaviorCommunicationBridge()
   {
      return behaviorCommunicationBridge;
   }

   public void updateRobotModel()
   {
      yoTimeLastFullRobotModelUpdate.set(yoTimeRobot.getDoubleValue());
      robotDataReceiver.updateRobotModel();
   }

   public void dispatchBehavior(BehaviorInterface behaviorToTest) throws SimulationExceededMaximumTimeException
   {
      HumanoidBehaviorType testBehaviorType = HumanoidBehaviorType.TEST;
      behaviorDispatcher.addHumanoidBehavior(testBehaviorType, behaviorToTest);

      Thread dispatcherThread = new Thread(behaviorDispatcher, "BehaviorDispatcher");
      dispatcherThread.start();

      HumanoidBehaviorTypePacket requestTestBehaviorPacket = new HumanoidBehaviorTypePacket(testBehaviorType);
      networkObjectCommunicator.send(requestTestBehaviorPacket);

      boolean success = simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
   }

   private BehaviorDisptacher setupBehaviorDispatcher(FullRobotModel fullRobotModel, PacketCommunicator behaviorCommunicator,
         RobotDataReceiver robotDataReceiver, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      HumanoidBehaviorControlModeSubscriber desiredBehaviorControlSubscriber = new HumanoidBehaviorControlModeSubscriber();
      behaviorCommunicator.attachListener(HumanoidBehaviorControlModePacket.class, desiredBehaviorControlSubscriber);

      HumanoidBehaviorTypeSubscriber desiredBehaviorSubscriber = new HumanoidBehaviorTypeSubscriber();
      behaviorCommunicator.attachListener(HumanoidBehaviorTypePacket.class, desiredBehaviorSubscriber);

      YoVariableServer yoVariableServer = null;
      yoGraphicsListRegistry.setYoGraphicsUpdatedRemotely(false);

      BehaviorDisptacher ret = new BehaviorDisptacher(yoTimeBehaviorDispatcher, robotDataReceiver, desiredBehaviorControlSubscriber, desiredBehaviorSubscriber,
            behaviorCommunicationBridge, yoVariableServer, registry, yoGraphicsListRegistry);

      ret.addUpdatable(capturePointUpdatable);
      ret.addUpdatable(wristForceSensorUpdatables.get(RobotSide.LEFT));
      ret.addUpdatable(wristForceSensorUpdatables.get(RobotSide.RIGHT));

      return ret;
   }

   private SideDependentList<WristForceSensorFilteredUpdatable> createWristForceSensorUpdateables()
   {
      SideDependentList<WristForceSensorFilteredUpdatable> ret = new SideDependentList<WristForceSensorFilteredUpdatable>();

      for (RobotSide robotSide : RobotSide.values)
      {
         WristForceSensorFilteredUpdatable wristSensorUpdatable = new WristForceSensorFilteredUpdatable(robotSide, fullRobotModel,
               drcRobotModel.getSensorInformation(), robotDataReceiver.getForceSensorDataHolder(), IHMCHumanoidBehaviorManager.BEHAVIOR_YO_VARIABLE_SERVER_DT,
               controllerCommunicator, registry);

         ret.put(robotSide, wristSensorUpdatable);
      }

      return ret;
   }

   private CapturePointUpdatable createCapturePointUpdateable(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      CapturabilityBasedStatusSubscriber capturabilityBasedStatusSubsrciber = new CapturabilityBasedStatusSubscriber();
      controllerCommunicator.attachListener(CapturabilityBasedStatus.class, capturabilityBasedStatusSubsrciber);

      CapturePointUpdatable ret = new CapturePointUpdatable(capturabilityBasedStatusSubsrciber, yoGraphicsListRegistry, registry);

      return ret;
   }

   public void closeAndDispose()
   {
      if (behaviorDispatcher != null)
      {
         behaviorDispatcher.closeAndDispose();
      }

      if (networkObjectCommunicator != null)
      {
         networkObjectCommunicator.close();
         networkObjectCommunicator.closeAndDispose();
      }

      if (behaviorCommunicator != null)
      {
         behaviorCommunicator.close();
         behaviorCommunicator.closeAndDispose();
      }
      
      if (behaviorCommunicationBridge != null)
      {
         behaviorCommunicationBridge.closeAndDispose();
      }

      if (controllerCommunicator != null)
      {
         controllerCommunicator.close();
         controllerCommunicator.closeAndDispose();
      }
      
      super.destroySimulation();
   }

   public boolean executeBehaviorsSimulateAndBlockAndCatchExceptions(final SideDependentList<BehaviorInterface> behaviors, double simulationRunTime)
         throws SimulationExceededMaximumTimeException
   {
      ArrayList<BehaviorInterface> behaviorArrayList = new ArrayList<BehaviorInterface>();

      for (RobotSide robotSide : RobotSide.values)
      {
         behaviorArrayList.add(behaviors.get(robotSide));
      }

      boolean ret = executeBehaviorsSimulateAndBlockAndCatchExceptions(behaviorArrayList, simulationRunTime);
      return ret;
   }

   public boolean executeBehaviorsSimulateAndBlockAndCatchExceptions(final ArrayList<BehaviorInterface> behaviors, double simulationRunTime)
         throws SimulationExceededMaximumTimeException
   {
      ArrayList<BehaviorRunner> behaviorRunners = new ArrayList<DRCBehaviorTestHelper.BehaviorRunner>();

      for (BehaviorInterface behavior : behaviors)
      {
         BehaviorRunner behaviorRunner = startNewBehaviorRunnerThread(behavior);
         behaviorRunners.add(behaviorRunner);
      }

      boolean ret = simulateAndBlockAndCatchExceptions(simulationRunTime);

      for (BehaviorRunner behaviorRunner : behaviorRunners)
      {
         behaviorRunner.closeAndDispose();
      }

      return ret;
   }

   public StopThreadUpdatable executeBehaviorPauseAndResumeOrStop(final BehaviorInterface behavior, double pausePercent, double pauseDuration,
         double stopPercent, FramePose poseAtTrajectoryEnd, ReferenceFrame frameToKeepTrackOf) throws SimulationExceededMaximumTimeException
   {
      StopThreadUpdatable stopThreadUpdatable = new TrajectoryBasedStopThreadUpdatable(robotDataReceiver, behavior, pausePercent, pauseDuration, stopPercent,
            poseAtTrajectoryEnd, frameToKeepTrackOf);

      boolean success = executeBehaviorPauseAndResumeOrStop(behavior, stopThreadUpdatable);
      assertTrue(success);

      return stopThreadUpdatable;
   }

   public StopThreadUpdatable executeBehaviorPauseAndResumeOrStop(final BehaviorInterface behavior, double pauseTime, double resumeTime, double stopTime,
         ReferenceFrame frameToKeepTrackOf) throws SimulationExceededMaximumTimeException
   {
      StopThreadUpdatable stopThreadUpdatable = new TimeBasedStopThreadUpdatable(robotDataReceiver, behavior, pauseTime, resumeTime, stopTime,
            frameToKeepTrackOf);

      boolean success = executeBehaviorPauseAndResumeOrStop(behavior, stopThreadUpdatable);
      assertTrue(success);

      return stopThreadUpdatable;
   }

   public boolean executeBehaviorPauseAndResumeOrStop(BehaviorInterface behavior, StopThreadUpdatable stopThreadUpdatable)
         throws SimulationExceededMaximumTimeException
   {
      StoppableBehaviorRunner behaviorRunner = new StoppableBehaviorRunner(behavior, stopThreadUpdatable);
      Thread behaviorThread = new Thread(behaviorRunner);
      behaviorThread.start();

      boolean success = true;
      while (!stopThreadUpdatable.shouldBehaviorRunnerBeStopped())
      {
         success &= simulateAndBlockAndCatchExceptions(1.0);
      }
      behaviorRunner.closeAndDispose();

      assertTrue(success);

      return success;
   }

   public boolean executeBehaviorSimulateAndBlockAndCatchExceptions(final BehaviorInterface behavior, double simulationRunTime)
         throws SimulationExceededMaximumTimeException
   {
      BehaviorRunner behaviorRunner = startNewBehaviorRunnerThread(behavior);

      boolean ret = simulateAndBlockAndCatchExceptions(simulationRunTime);
      behaviorRunner.closeAndDispose();

      return ret;
   }

   public boolean executeBehaviorUntilDone(final BehaviorInterface behavior) throws SimulationExceededMaximumTimeException
   {
      BehaviorRunner behaviorRunner = startNewBehaviorRunnerThread(behavior);

      boolean ret = true;
      while (!behavior.isDone())
      {
         ret = simulateAndBlockAndCatchExceptions(1.0);
      }

      behaviorRunner.closeAndDispose();

      return ret;
   }

   private BehaviorRunner startNewBehaviorRunnerThread(final BehaviorInterface behavior)
   {
      BehaviorRunner behaviorRunner = new BehaviorRunner(behavior);
      Thread behaviorThread = new Thread(behaviorRunner);
      behaviorThread.start();

      return behaviorRunner;
   }

   private class BehaviorRunner implements Runnable
   {
      protected boolean isRunning = true;
      protected final BehaviorInterface behavior;

      public BehaviorRunner(BehaviorInterface behavior)
      {
         this.behavior = behavior;
         behaviorCommunicationBridge.attachGlobalListener(behavior.getControllerGlobalPacketConsumer());
      }

      public void run()
      {
         while (isRunning)
         {
            behavior.doControl();

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

      private HumanoidBehaviorControlModeEnum currentControlMode = HumanoidBehaviorControlModeEnum.RESUME;

      public StoppableBehaviorRunner(BehaviorInterface behavior, StopThreadUpdatable stopThreadUpdatable)
      {
         super(behavior);
         this.stopThreadUpdatable = stopThreadUpdatable;
      }

      public void run()
      {
         while (isRunning)
         {
            behavior.doControl();
            
            for (Updatable updatable : updatables)
            {
               updatable.update(yoTimeRobot.getDoubleValue());
            }
            
            stopThreadUpdatable.update(yoTimeRobot.getDoubleValue());

            HumanoidBehaviorControlModeEnum requestedControlMode = stopThreadUpdatable.getRequestedBehaviorControlMode();

            if (stopThreadUpdatable.shouldBehaviorRunnerBeStopped())
            {
               SysoutTool.println("Stopping Thread!", DEBUG);
               isRunning = false;
            }
            else if (requestedControlMode.equals(HumanoidBehaviorControlModeEnum.PAUSE) && !currentControlMode.equals(HumanoidBehaviorControlModeEnum.PAUSE))
            {
               behavior.pause();
               currentControlMode = HumanoidBehaviorControlModeEnum.PAUSE;
            }
            else if (requestedControlMode.equals(HumanoidBehaviorControlModeEnum.STOP) && !currentControlMode.equals(HumanoidBehaviorControlModeEnum.STOP))
            {
               behavior.stop();
               currentControlMode = HumanoidBehaviorControlModeEnum.STOP;
            }
            else if (requestedControlMode.equals(HumanoidBehaviorControlModeEnum.RESUME) && !currentControlMode.equals(HumanoidBehaviorControlModeEnum.RESUME))
            {
               behavior.resume();
               currentControlMode = HumanoidBehaviorControlModeEnum.RESUME;
            }
            
            ThreadTools.sleep(1);
         }
      }
   }
}
