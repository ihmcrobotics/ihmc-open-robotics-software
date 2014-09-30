package us.ihmc.humanoidBehaviors.workingArea;

import us.ihmc.communication.packets.behaviors.HumanoidBehaviorControlModePacket;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.communication.packets.behaviors.HumanoidBehaviorTypePacket;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.humanoidBehaviors.workingArea.behaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.workingArea.behaviors.SimpleForwardingBehavior;
import us.ihmc.humanoidBehaviors.workingArea.behaviors.scripts.ScriptBehavior;
import us.ihmc.humanoidBehaviors.workingArea.communication.BehaviorCommunicationBridge;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class IHMCHumanoidBehaviorManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleYoVariable yoTime = new DoubleYoVariable("yoTime", registry);

   public IHMCHumanoidBehaviorManager(FullRobotModel fullRobotModel, ObjectCommunicator networkProcessorCommunicator, ObjectCommunicator controllerCommunicator)
   {
      BehaviorCommunicationBridge communicationBridge = new BehaviorCommunicationBridge(networkProcessorCommunicator, controllerCommunicator, registry);

      RobotDataReceiver robotDataReceiver = new RobotDataReceiver(fullRobotModel);
      HumanoidBehaviorDispatcherControlSubscriber desiredBehaviorControlSubscriber = new HumanoidBehaviorDispatcherControlSubscriber();
      HumanoidBehaviorTypeSubscriber desiredBehaviorSubscriber = new HumanoidBehaviorTypeSubscriber();

      BehaviorDisptacher dispatcher = createBehaviorDispatcher(fullRobotModel, communicationBridge, robotDataReceiver, desiredBehaviorControlSubscriber,
            desiredBehaviorSubscriber);

      controllerCommunicator.attachListener(RobotConfigurationData.class, robotDataReceiver);
      networkProcessorCommunicator.attachListener(HumanoidBehaviorControlModePacket.class, desiredBehaviorControlSubscriber);
      networkProcessorCommunicator.attachListener(HumanoidBehaviorTypePacket.class, desiredBehaviorSubscriber);

      Thread dispatcherThread = new Thread(dispatcher, "BehaviorDispatcher");
      dispatcherThread.start();
   }

   private BehaviorDisptacher createBehaviorDispatcher(FullRobotModel fullRobotModel, BehaviorCommunicationBridge communicationBridge,
         RobotDataReceiver robotDataReceiver, HumanoidBehaviorDispatcherControlSubscriber desiredBehaviorControlSubscriber,
         HumanoidBehaviorTypeSubscriber desiredBehaviorSubscriber)
   {
      BehaviorDisptacher dispatcher = new BehaviorDisptacher(yoTime, fullRobotModel, robotDataReceiver, desiredBehaviorControlSubscriber, desiredBehaviorSubscriber,
            communicationBridge, registry);

      SimpleForwardingBehavior simpleForwardingBehavior = new SimpleForwardingBehavior(communicationBridge);
      simpleForwardingBehavior.attachCommunicationBridge(communicationBridge);

      ScriptBehavior scriptBehavior = new ScriptBehavior(communicationBridge, fullRobotModel, yoTime);

      dispatcher.addHumanoidBehavior(HumanoidBehaviorType.STOP, simpleForwardingBehavior);
      dispatcher.addHumanoidBehavior(HumanoidBehaviorType.DO_NOTHING, new SimpleDoNothingBehavior(communicationBridge));
      dispatcher.addHumanoidBehavior(HumanoidBehaviorType.SCRIPT, scriptBehavior);

      return dispatcher;
   }
}
