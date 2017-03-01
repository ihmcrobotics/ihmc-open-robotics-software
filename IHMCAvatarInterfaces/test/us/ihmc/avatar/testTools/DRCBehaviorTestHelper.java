package us.ihmc.avatar.testTools;

import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;

import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.communication.PacketRouter;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.IHMCHumanoidBehaviorManager;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.dispatcher.BehaviorControlModeSubscriber;
import us.ihmc.humanoidBehaviors.dispatcher.BehaviorDispatcher;
import us.ihmc.humanoidBehaviors.dispatcher.HumanoidBehaviorTypeSubscriber;
import us.ihmc.humanoidBehaviors.utilities.CapturePointUpdatable;
import us.ihmc.humanoidBehaviors.utilities.StopThreadUpdatable;
import us.ihmc.humanoidBehaviors.utilities.TimeBasedStopThreadUpdatable;
import us.ihmc.humanoidBehaviors.utilities.TrajectoryBasedStopThreadUpdatable;
import us.ihmc.humanoidBehaviors.utilities.WristForceSensorFilteredUpdatable;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModePacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModePacket.BehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorTypePacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.humanoidRobotics.communication.subscribers.CapturabilityBasedStatusSubscriber;
import us.ihmc.humanoidRobotics.communication.subscribers.HumanoidRobotDataReceiver;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationconstructionset.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

/** 
 * Do not execute more than one behavior thread at a time.  Instead, run multiple behaviors in a single thread.
 * @author cschmidt
 *
 */
public class DRCBehaviorTestHelper extends DRCSimulationTestHelper
{
   private static final IHMCCommunicationKryoNetClassList NET_CLASS_LIST = new IHMCCommunicationKryoNetClassList();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleYoVariable yoTimeRobot;
   private final DoubleYoVariable yoTimeBehaviorDispatcher;
   private final DoubleYoVariable yoTimeLastFullRobotModelUpdate;

   private final DRCRobotModel drcRobotModel;
   private final FullHumanoidRobotModel fullRobotModel;

   private final PacketRouter<PacketDestination> networkProcessor;
   private final PacketCommunicator mockUIPacketCommunicatorServer;//send packets as if it was sent from the UI
   private final PacketCommunicator behaviorCommunicatorServer;
   private final CommunicationBridge behaviorCommunicationBridge;
   private final HumanoidRobotDataReceiver robotDataReceiver;
   private final HumanoidReferenceFrames referenceFrames;

   private final ArrayList<Updatable> updatables = new ArrayList<Updatable>();
   private final CapturePointUpdatable capturePointUpdatable;
   private final SideDependentList<WristForceSensorFilteredUpdatable> wristForceSensorUpdatables;

   private final BehaviorDispatcher behaviorDispatcher;

   
   private final PacketCommunicator behaviorCommunicatorClient;
   private final PacketCommunicator mockUIPacketCommunicatorClient;

   public DRCBehaviorTestHelper(String name, DRCStartingLocation selectedLocation,
         SimulationTestingParameters simulationTestingParameters, DRCRobotModel robotModel)
   {
      this(new DefaultCommonAvatarEnvironment(), name, selectedLocation, simulationTestingParameters, robotModel, true);
   }

   public DRCBehaviorTestHelper(CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface,
                                String name, DRCStartingLocation selectedLocation, SimulationTestingParameters simulationTestingParameters,
                                DRCRobotModel robotModel)
   {
      this(commonAvatarEnvironmentInterface, name, selectedLocation, simulationTestingParameters, robotModel, true);
   }

   public DRCBehaviorTestHelper(CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface,
         String name, DRCStartingLocation selectedLocation, SimulationTestingParameters simulationTestingParameters,
         DRCRobotModel robotModel, boolean automaticallySimulate)
   {
      super(commonAvatarEnvironmentInterface, name, selectedLocation, simulationTestingParameters, robotModel, automaticallySimulate);

      yoTimeRobot = getRobot().getYoTime();
      yoTimeBehaviorDispatcher = new DoubleYoVariable("yoTimeBehaviorDispatcher", registry);

      this.drcRobotModel = robotModel;
      this.fullRobotModel = robotModel.createFullRobotModel();
      yoTimeLastFullRobotModelUpdate = new DoubleYoVariable("yoTimeRobotModelUpdate", registry);

      
      this.mockUIPacketCommunicatorServer = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.UI_MODULE, NET_CLASS_LIST);
      mockUIPacketCommunicatorClient = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.UI_MODULE, NET_CLASS_LIST);
      
      behaviorCommunicatorServer = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.BEHAVIOUR_MODULE_PORT, NET_CLASS_LIST);
      behaviorCommunicatorClient = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.BEHAVIOUR_MODULE_PORT, NET_CLASS_LIST);

      try
      {
         behaviorCommunicatorClient.connect();
         behaviorCommunicatorServer.connect();
         mockUIPacketCommunicatorServer.connect();
         mockUIPacketCommunicatorClient.connect();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
      
      networkProcessor = new PacketRouter<>(PacketDestination.class);
      networkProcessor.attachPacketCommunicator(PacketDestination.UI, mockUIPacketCommunicatorClient);
      networkProcessor.attachPacketCommunicator(PacketDestination.CONTROLLER, controllerCommunicator);
      networkProcessor.attachPacketCommunicator(PacketDestination.BEHAVIOR_MODULE, behaviorCommunicatorClient);

      behaviorCommunicationBridge = new CommunicationBridge(behaviorCommunicatorServer);

      ForceSensorDataHolder forceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(fullRobotModel.getForceSensorDefinitions()));
      robotDataReceiver = new HumanoidRobotDataReceiver(fullRobotModel, forceSensorDataHolder);
      mockUIPacketCommunicatorServer.attachListener(RobotConfigurationData.class, robotDataReceiver);

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      capturePointUpdatable = createCapturePointUpdateable(yoGraphicsListRegistry);
      wristForceSensorUpdatables = createWristForceSensorUpdateables();
      updatables.add(capturePointUpdatable);
      updatables.add(wristForceSensorUpdatables.get(RobotSide.LEFT));
      updatables.add(wristForceSensorUpdatables.get(RobotSide.RIGHT));

      behaviorDispatcher = setupBehaviorDispatcher(fullRobotModel, behaviorCommunicatorServer, robotDataReceiver, yoGraphicsListRegistry);

      referenceFrames = robotDataReceiver.getReferenceFrames();
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
   
   public SideDependentList<WristForceSensorFilteredUpdatable> getWristForceSensorUpdatableSideDependentList()
   {
      return wristForceSensorUpdatables;
   }

   public CommunicationBridge getBehaviorCommunicationBridge()
   {
      return behaviorCommunicationBridge;
   }

   public void updateRobotModel()
   {
      yoTimeLastFullRobotModelUpdate.set(yoTimeRobot.getDoubleValue());
      robotDataReceiver.updateRobotModel();
   }

   public PacketCommunicator createAndStartPacketCommunicator(NetworkPorts port, PacketDestination destination) throws IOException
   {
      PacketCommunicator packetCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(port, NET_CLASS_LIST);
      networkProcessor.attachPacketCommunicator(destination, packetCommunicator);
      packetCommunicator.connect();
      return packetCommunicator;
   }

   public void dispatchBehavior(AbstractBehavior behaviorToTest) throws SimulationExceededMaximumTimeException
   {
      HumanoidBehaviorType testBehaviorType = HumanoidBehaviorType.TEST;
      behaviorDispatcher.addBehavior(testBehaviorType, behaviorToTest);

      behaviorDispatcher.start();


      HumanoidBehaviorTypePacket requestTestBehaviorPacket = new HumanoidBehaviorTypePacket(testBehaviorType);
      mockUIPacketCommunicatorServer.send(requestTestBehaviorPacket);

      boolean success = simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
   }
   
   public void sendBehaviorToDispatcher(AbstractBehavior behaviorToTest) throws SimulationExceededMaximumTimeException
   {
      HumanoidBehaviorType testBehaviorType = HumanoidBehaviorType.TEST;
      behaviorDispatcher.addBehavior(testBehaviorType, behaviorToTest);
      
      HumanoidBehaviorTypePacket requestTestBehaviorPacket = new HumanoidBehaviorTypePacket(testBehaviorType);
      mockUIPacketCommunicatorServer.send(requestTestBehaviorPacket);
   }

   private BehaviorDispatcher setupBehaviorDispatcher(FullRobotModel fullRobotModel, PacketCommunicator behaviorCommunicator,
         HumanoidRobotDataReceiver robotDataReceiver, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      BehaviorControlModeSubscriber desiredBehaviorControlSubscriber = new BehaviorControlModeSubscriber();
      behaviorCommunicator.attachListener(BehaviorControlModePacket.class, desiredBehaviorControlSubscriber);

      HumanoidBehaviorTypeSubscriber desiredBehaviorSubscriber = new HumanoidBehaviorTypeSubscriber();
      behaviorCommunicator.attachListener(HumanoidBehaviorTypePacket.class, desiredBehaviorSubscriber);

      YoVariableServer yoVariableServer = null;
      yoGraphicsListRegistry.setYoGraphicsUpdatedRemotely(false);

      BehaviorDispatcher<HumanoidBehaviorType> ret = new BehaviorDispatcher<>(yoTimeBehaviorDispatcher, robotDataReceiver, desiredBehaviorControlSubscriber, desiredBehaviorSubscriber,
            behaviorCommunicationBridge, yoVariableServer, HumanoidBehaviorType.class, HumanoidBehaviorType.STOP, registry, yoGraphicsListRegistry);

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

      if (mockUIPacketCommunicatorServer != null)
      {
         mockUIPacketCommunicatorServer.close();
      }
      
      if (mockUIPacketCommunicatorClient != null)
      {
         mockUIPacketCommunicatorClient.close();
      }
      

      if (behaviorCommunicatorClient != null)
      {
         behaviorCommunicatorClient.close();
      }
      
      if (behaviorCommunicatorServer != null)
      {
         behaviorCommunicatorServer.close();
      }

      if (controllerCommunicator != null)
      {
         controllerCommunicator.close();
      }

      super.destroySimulation();
   }

   public boolean executeBehaviorsSimulateAndBlockAndCatchExceptions(final SideDependentList<AbstractBehavior> behaviors, double simulationRunTime)
         throws SimulationExceededMaximumTimeException
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
         throws SimulationExceededMaximumTimeException
   {
      BehaviorRunner behaviorRunner = startNewBehaviorRunnerThread(behaviors);

      boolean ret = simulateAndBlockAndCatchExceptions(simulationRunTime);
      behaviorRunner.closeAndDispose();

      return ret;
   }

   public StopThreadUpdatable executeBehaviorPauseAndResumeOrStop(final AbstractBehavior behavior, double pausePercent, double pauseDuration,
         double stopPercent, FramePose poseAtTrajectoryEnd, ReferenceFrame frameToKeepTrackOf) throws SimulationExceededMaximumTimeException
   {
      StopThreadUpdatable stopThreadUpdatable = new TrajectoryBasedStopThreadUpdatable(robotDataReceiver, behavior, pausePercent, pauseDuration, stopPercent,
            poseAtTrajectoryEnd, frameToKeepTrackOf);

      boolean success = executeBehaviorPauseAndResumeOrStop(behavior, stopThreadUpdatable);
      assertTrue(success);

      return stopThreadUpdatable;
   }

   public StopThreadUpdatable executeBehaviorPauseAndResumeOrStop(final AbstractBehavior behavior, double pauseTime, double pauseDuration, double stopTime,
         ReferenceFrame frameToKeepTrackOf) throws SimulationExceededMaximumTimeException
   {
      StopThreadUpdatable stopThreadUpdatable = new TimeBasedStopThreadUpdatable(robotDataReceiver, behavior, pauseTime, pauseDuration, stopTime,
            frameToKeepTrackOf);

      boolean success = executeBehaviorPauseAndResumeOrStop(behavior, stopThreadUpdatable);
      assertTrue(success);

      return stopThreadUpdatable;
   }

   public boolean executeBehaviorPauseAndResumeOrStop(AbstractBehavior behavior, StopThreadUpdatable stopThreadUpdatable)
         throws SimulationExceededMaximumTimeException
   {
      StoppableBehaviorRunner behaviorRunner = new StoppableBehaviorRunner(behavior, stopThreadUpdatable);
      Thread behaviorThread = new Thread(behaviorRunner);
      behaviorThread.start();

      boolean success = true;
      while (!stopThreadUpdatable.shouldBehaviorRunnerBeStopped() && success)
      {
         success = simulateAndBlockAndCatchExceptions(1.0);
      }
      behaviorRunner.closeAndDispose();

      assertTrue(success);

      return success;
   }

   public boolean executeBehaviorSimulateAndBlockAndCatchExceptions(final AbstractBehavior behavior, double simulationRunTime)
         throws SimulationExceededMaximumTimeException
   {
      BehaviorRunner behaviorRunner = startNewBehaviorRunnerThread(behavior);

      boolean ret = simulateAndBlockAndCatchExceptions(simulationRunTime);
      behaviorRunner.closeAndDispose();

      return ret;
   }

   public boolean executeBehaviorUntilDone(final AbstractBehavior behavior) throws SimulationExceededMaximumTimeException
   {
      BehaviorRunner behaviorRunner = startNewBehaviorRunnerThread(behavior);

      boolean success = true;
      while (!behavior.isDone() && success)
      {
         success = simulateAndBlockAndCatchExceptions(1.0);
      }

      behaviorRunner.closeAndDispose();

      return success;
   }
   
   public boolean executeBehaviorUntilDoneUsingBehaviorDispatcher(final AbstractBehavior behavior) throws SimulationExceededMaximumTimeException
   {
      behaviorDispatcher.start();

      
      boolean success = true;
      success = simulateAndBlockAndCatchExceptions(0.1);
      sendBehaviorToDispatcher(behavior);
      
      while (!behavior.isDone() && success)
      {
         success = simulateAndBlockAndCatchExceptions(1.0);
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
               PrintTools.debug(this, "Stopping Thread!");
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

   public void sendPacketAsIfItWasFromUI(Packet<?> packet)
   {
      mockUIPacketCommunicatorServer.send(packet);
   }
}
