package us.ihmc.humanoidBehaviors;

import us.ihmc.communication.packets.behaviors.HumanoidBehaviorControlModePacket;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorTypePacket;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.humanoidBehaviors.behaviors.scripts.ScriptBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.dispatcher.BehaviorDisptacher;
import us.ihmc.humanoidBehaviors.dispatcher.HumanoidBehaviorControlModeSubscriber;
import us.ihmc.humanoidBehaviors.dispatcher.HumanoidBehaviorTypeSubscriber;
import us.ihmc.robotDataCommunication.YoVariableServer;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

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

      RobotDataReceiver robotDataReceiver = new RobotDataReceiver(fullRobotModel);
      HumanoidBehaviorControlModeSubscriber desiredBehaviorControlSubscriber = new HumanoidBehaviorControlModeSubscriber();
      HumanoidBehaviorTypeSubscriber desiredBehaviorSubscriber = new HumanoidBehaviorTypeSubscriber();

      BehaviorDisptacher dispatcher = new BehaviorDisptacher(yoTime, fullRobotModel, robotDataReceiver, desiredBehaviorControlSubscriber,
            desiredBehaviorSubscriber, communicationBridge, yoVariableServer, registry);

      createAndRegisterBehaviors(dispatcher, fullRobotModel, yoTime, communicationBridge);

      controllerCommunicator.attachListener(RobotConfigurationData.class, robotDataReceiver);
      networkProcessorCommunicator.attachListener(HumanoidBehaviorControlModePacket.class, desiredBehaviorControlSubscriber);
      networkProcessorCommunicator.attachListener(HumanoidBehaviorTypePacket.class, desiredBehaviorSubscriber);
      
      yoVariableServer.setMainRegistry(registry, fullRobotModel, null);
      yoVariableServer.start();

      Thread dispatcherThread = new Thread(dispatcher, "BehaviorDispatcher");
      dispatcherThread.start();
   }

   /**
    * Create the different behaviors and register them in the dispatcher.
    * When creating a new behavior, that's where you need to add it.
    * @param fullRobotModel Holds the robot data (like joint angles). The data is updated in the dispatcher and can be shared with the behaviors.
    * @param yoTime Holds the controller time. It is updated in the dispatcher and can be shared with the behaviors.
    * @param outgoingCommunicationBridge used to send packets to the controller.
    * @return
    */
   private void createAndRegisterBehaviors(BehaviorDisptacher dispatcher, FullRobotModel fullRobotModel,
         DoubleYoVariable yoTime, OutgoingCommunicationBridgeInterface outgoingCommunicationBridge)
   {
      dispatcher.addHumanoidBehavior(HumanoidBehaviorType.DO_NOTHING, new SimpleDoNothingBehavior(outgoingCommunicationBridge));

      ScriptBehavior scriptBehavior = new ScriptBehavior(outgoingCommunicationBridge, fullRobotModel, yoTime);
      dispatcher.addHumanoidBehavior(HumanoidBehaviorType.SCRIPT, scriptBehavior);
   }
}
