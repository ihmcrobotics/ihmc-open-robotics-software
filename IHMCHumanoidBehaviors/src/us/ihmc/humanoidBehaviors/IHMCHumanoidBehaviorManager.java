package us.ihmc.humanoidBehaviors;

import java.util.Arrays;

import us.ihmc.communication.packets.behaviors.HumanoidBehaviorControlModePacket;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorTypePacket;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.packets.walking.CapturabilityBasedStatus;
import us.ihmc.communication.subscribers.CapturabilityBasedStatusSubsrciber;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.humanoidBehaviors.behaviors.DebrisBehavior_NOPE;
import us.ihmc.humanoidBehaviors.behaviors.LocalizationBehavior;
import us.ihmc.humanoidBehaviors.behaviors.RemoveMultipleDebrisBehavior;
import us.ihmc.humanoidBehaviors.behaviors.TurnValveBehavior;
import us.ihmc.humanoidBehaviors.behaviors.WalkToLocationBehavior;
import us.ihmc.humanoidBehaviors.behaviors.WalkToLocationTrial;
import us.ihmc.humanoidBehaviors.behaviors.scripts.ScriptBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.dispatcher.BehaviorDisptacher;
import us.ihmc.humanoidBehaviors.dispatcher.HumanoidBehaviorControlModeSubscriber;
import us.ihmc.humanoidBehaviors.dispatcher.HumanoidBehaviorTypeSubscriber;
import us.ihmc.humanoidBehaviors.utilities.CapturePointUpdatable;
import us.ihmc.robotDataCommunication.YoVariableServer;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.frames.YoFramePoint2d;

public class IHMCHumanoidBehaviorManager
{
   public static final double BEHAVIOR_YO_VARIABLE_SERVER_DT = 0.006;

   private final YoVariableServer yoVariableServer = new YoVariableServer(NetworkConfigParameters.BEHAVIOR_YO_VARIABLE_SERVER_PORT, BEHAVIOR_YO_VARIABLE_SERVER_DT);
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleYoVariable yoTime = new DoubleYoVariable("yoTime", registry);

   public IHMCHumanoidBehaviorManager(FullRobotModel fullRobotModel, ObjectCommunicator networkProcessorCommunicator, ObjectCommunicator controllerCommunicator)
   {
      System.out.println("[INFO] " + getClass().getSimpleName() + ": Initializing");

      BehaviorCommunicationBridge communicationBridge = new BehaviorCommunicationBridge(networkProcessorCommunicator, controllerCommunicator, registry);

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      yoGraphicsListRegistry.setYoGraphicsUpdatedRemotely(false);
      ForceSensorDataHolder forceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(fullRobotModel.getForceSensorDefinitions()));
      RobotDataReceiver robotDataReceiver = new RobotDataReceiver(fullRobotModel, forceSensorDataHolder);

      ReferenceFrames referenceFrames = robotDataReceiver.getReferenceFrames();
      controllerCommunicator.attachListener(RobotConfigurationData.class, robotDataReceiver);

      HumanoidBehaviorControlModeSubscriber desiredBehaviorControlSubscriber = new HumanoidBehaviorControlModeSubscriber();
      HumanoidBehaviorTypeSubscriber desiredBehaviorSubscriber = new HumanoidBehaviorTypeSubscriber();
      
      BehaviorDisptacher dispatcher = new BehaviorDisptacher(yoTime, robotDataReceiver, desiredBehaviorControlSubscriber,
            desiredBehaviorSubscriber, communicationBridge, yoVariableServer, registry, yoGraphicsListRegistry);

      CapturabilityBasedStatusSubsrciber capturabilityBasedStatusSubsrciber = new CapturabilityBasedStatusSubsrciber();
      controllerCommunicator.attachListener(CapturabilityBasedStatus.class, capturabilityBasedStatusSubsrciber);
      CapturePointUpdatable capturePointUpdatable = new CapturePointUpdatable(capturabilityBasedStatusSubsrciber, yoGraphicsListRegistry, registry);
      dispatcher.addUpdatable(capturePointUpdatable);

      YoFramePoint2d yoCapturePoint = capturePointUpdatable.getYoCapturePoint();
      YoFramePoint2d yoDesiredCapturePoint = capturePointUpdatable.getYoDesiredCapturePoint();
      createAndRegisterBehaviors(dispatcher, fullRobotModel, forceSensorDataHolder, referenceFrames, yoTime, yoCapturePoint, yoDesiredCapturePoint,
            communicationBridge, yoGraphicsListRegistry, BEHAVIOR_YO_VARIABLE_SERVER_DT);

      networkProcessorCommunicator.attachListener(HumanoidBehaviorControlModePacket.class, desiredBehaviorControlSubscriber);
      networkProcessorCommunicator.attachListener(HumanoidBehaviorTypePacket.class, desiredBehaviorSubscriber);

      yoVariableServer.setMainRegistry(registry, fullRobotModel, yoGraphicsListRegistry);
      yoVariableServer.start();

      Thread dispatcherThread = new Thread(dispatcher, "BehaviorDispatcher");
      dispatcherThread.start();
   }

   /**
    * Create the different behaviors and register them in the dispatcher.
    * When creating a new behavior, that's where you need to add it.
    * @param fullRobotModel Holds the robot data (like joint angles). The data is updated in the dispatcher and can be shared with the behaviors.
    * @param forceSensorDataHolder Holds the force sensor data
    * @param referenceFrames Give access to useful references related to the robot. They're automatically updated.
    * @param yoTime Holds the controller time. It is updated in the dispatcher and can be shared with the behaviors.
    * @param outgoingCommunicationBridge used to send packets to the controller.
    * @param yoGraphicsListRegistry Allows to register YoGraphics that will be displayed in SCS.
    */
   private void createAndRegisterBehaviors(BehaviorDisptacher dispatcher, FullRobotModel fullRobotModel, ForceSensorDataHolder forceSensorDataHolder, ReferenceFrames referenceFrames,
         DoubleYoVariable yoTime, YoFramePoint2d yoCapturePoint, YoFramePoint2d yoDesiredCapturePoint, OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, YoGraphicsListRegistry yoGraphicsListRegistry, double DT)
   {
      dispatcher.addHumanoidBehavior(HumanoidBehaviorType.DO_NOTHING, new SimpleDoNothingBehavior(outgoingCommunicationBridge));

      ScriptBehavior scriptBehavior = new ScriptBehavior(outgoingCommunicationBridge, fullRobotModel, yoTime);
      dispatcher.addHumanoidBehavior(HumanoidBehaviorType.SCRIPT, scriptBehavior);
     
      LocalizationBehavior localizationBehavior = new LocalizationBehavior(outgoingCommunicationBridge, fullRobotModel, yoTime);
      dispatcher.addHumanoidBehavior(HumanoidBehaviorType.LOCALIZATION, localizationBehavior);
      
      TurnValveBehavior walkAndTurnValveBehavior = new TurnValveBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrames, yoTime, yoCapturePoint, yoDesiredCapturePoint, forceSensorDataHolder, DT);
      dispatcher.addHumanoidBehavior(HumanoidBehaviorType.WALK_N_TURN_VALVE, walkAndTurnValveBehavior);
      
      RemoveMultipleDebrisBehavior removeDebrisBehavior = new RemoveMultipleDebrisBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrames, yoTime);
      dispatcher.addHumanoidBehavior(HumanoidBehaviorType.DEBRIS_TASK, removeDebrisBehavior);
      
   }
}
