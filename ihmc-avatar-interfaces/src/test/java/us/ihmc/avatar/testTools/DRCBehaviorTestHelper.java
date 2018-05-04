package us.ihmc.avatar.testTools;

import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;

import controller_msgs.msg.dds.BehaviorControlModePacket;
import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.HumanoidBehaviorTypePacket;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.PacketRouter;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
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
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.humanoidRobotics.communication.subscribers.CapturabilityBasedStatusSubscriber;
import us.ihmc.humanoidRobotics.communication.subscribers.HumanoidRobotDataReceiver;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/** 
 * Do not execute more than one behavior thread at a time.  Instead, run multiple behaviors in a single thread.
 * @author cschmidt
 *
 */
public class DRCBehaviorTestHelper extends DRCSimulationTestHelper
{
   private static final IHMCCommunicationKryoNetClassList NET_CLASS_LIST = new IHMCCommunicationKryoNetClassList();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoDouble yoTimeRobot;
   private final YoDouble yoTimeBehaviorDispatcher;
   private final YoDouble yoTimeLastFullRobotModelUpdate;

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
	   this(commonAvatarEnvironmentInterface, name, selectedLocation, simulationTestingParameters, robotModel, null, automaticallySimulate);
   }
   
   public DRCBehaviorTestHelper(CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface,
	         String name, DRCStartingLocation selectedLocation, SimulationTestingParameters simulationTestingParameters,
	         DRCRobotModel robotModel, DRCNetworkModuleParameters networkModuleParameters, boolean automaticallySimulate)	   
   {
      super(simulationTestingParameters, robotModel);
      super.setTestEnvironment(commonAvatarEnvironmentInterface);
      super.setStartingLocation(selectedLocation);
      if (networkModuleParameters == null)
      {
         networkModuleParameters = new DRCNetworkModuleParameters();
         networkModuleParameters.enableNetworkProcessor(false);
      }
      super.setNetworkProcessorParameters(networkModuleParameters);
      super.createSimulation(name, automaticallySimulate, true);

      yoTimeRobot = getRobot().getYoTime();
      yoTimeBehaviorDispatcher = new YoDouble("yoTimeBehaviorDispatcher", registry);

      this.drcRobotModel = robotModel;
      this.fullRobotModel = robotModel.createFullRobotModel();
      yoTimeLastFullRobotModelUpdate = new YoDouble("yoTimeRobotModelUpdate", registry);

      
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

   public YoDouble getYoTime()
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


      HumanoidBehaviorTypePacket requestTestBehaviorPacket = HumanoidMessageTools.createHumanoidBehaviorTypePacket(testBehaviorType);
      mockUIPacketCommunicatorServer.send(requestTestBehaviorPacket);

      boolean success = simulateAndBlockAndCatchExceptions(1.0);
      assertTrue("Caught an exception when testing the behavior, the robot probably fell.", success);
   }
   
   public void sendBehaviorToDispatcher(AbstractBehavior behaviorToTest) throws SimulationExceededMaximumTimeException
   {
      HumanoidBehaviorType testBehaviorType = HumanoidBehaviorType.TEST;
      behaviorDispatcher.addBehavior(testBehaviorType, behaviorToTest);
      
      HumanoidBehaviorTypePacket requestTestBehaviorPacket = HumanoidMessageTools.createHumanoidBehaviorTypePacket(testBehaviorType);
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
         mockUIPacketCommunicatorServer.disconnect();
      }
      
      if (mockUIPacketCommunicatorClient != null)
      {
         mockUIPacketCommunicatorClient.disconnect();
      }
      

      if (behaviorCommunicatorClient != null)
      {
         behaviorCommunicatorClient.disconnect();
      }
      
      if (behaviorCommunicatorServer != null)
      {
         behaviorCommunicatorServer.disconnect();
      }

      if (controllerCommunicator != null)
      {
         controllerCommunicator.disconnect();
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
         double stopPercent, FramePose3D poseAtTrajectoryEnd, ReferenceFrame frameToKeepTrackOf) throws SimulationExceededMaximumTimeException
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
