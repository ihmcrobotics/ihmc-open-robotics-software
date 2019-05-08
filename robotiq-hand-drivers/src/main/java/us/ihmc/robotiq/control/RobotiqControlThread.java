package us.ihmc.robotiq.control;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;

import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import controller_msgs.msg.dds.HandJointAnglePacket;
import controller_msgs.msg.dds.ManualHandControlPacket;
import us.ihmc.avatar.handControl.HandControlThread;
import us.ihmc.avatar.handControl.packetsAndConsumers.HandJointAngleCommunicator;
import us.ihmc.avatar.handControl.packetsAndConsumers.ManualHandControlProvider;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.subscribers.HandDesiredConfigurationMessageSubscriber;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotiq.RobotiqHandCommunicator;
import us.ihmc.robotiq.data.RobotiqHandSensorData;

public class RobotiqControlThread extends HandControlThread
{
   private final boolean CALIBRATE_ON_CONNECT = false;

   private final RobotSide robotSide;
   private final RobotiqHandCommunicator robotiqHand;
   private final HandDesiredConfigurationMessageSubscriber handDesiredConfigurationMessageSubscriber;
   private final ManualHandControlProvider manualHandControlProvider;
   private final HandJointAngleCommunicator jointAngleCommunicator;
   private RobotiqHandSensorData handStatus;

   public RobotiqControlThread(String robotName, RobotSide robotSide)
   {
      super(robotSide);
      this.robotSide = robotSide;
      robotiqHand = new RobotiqHandCommunicator(robotSide);
      handDesiredConfigurationMessageSubscriber = new HandDesiredConfigurationMessageSubscriber(robotSide);
      manualHandControlProvider = new ManualHandControlProvider(robotSide);

      MessageTopicNameGenerator publisherTopicNameGenerator = ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      MessageTopicNameGenerator subscriberTopicNameGenerator = ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);

      IHMCRealtimeROS2Publisher<HandJointAnglePacket> handJointAnglePublisher = ROS2Tools.createPublisher(realtimeRos2Node, HandJointAnglePacket.class,
                                                                                                          publisherTopicNameGenerator);
      jointAngleCommunicator = new HandJointAngleCommunicator(robotSide, handJointAnglePublisher);
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, HandDesiredConfigurationMessage.class, subscriberTopicNameGenerator,
                                           handDesiredConfigurationMessageSubscriber);
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, ManualHandControlPacket.class, subscriberTopicNameGenerator, manualHandControlProvider);
      realtimeRos2Node.spin();
   }

   private void updateHandData()
   {
      handStatus = robotiqHand.getHandSensorData();
      jointAngleCommunicator.updateHandAngles(handStatus);
      jointAngleCommunicator.write();
   }

   @Override
   public void run()
   {
      if (CALIBRATE_ON_CONNECT)
         handDesiredConfigurationMessageSubscriber.receivedPacket(HumanoidMessageTools.createHandDesiredConfigurationMessage(robotSide,
                                                                                                                             HandConfiguration.CALIBRATE));

      while (true)
      {
         robotiqHand.read();

         updateHandData();

         if (handStatus.hasError())
         {
            //            System.out.println(handStatus.getFaultStatus().name());
         }

         if (handDesiredConfigurationMessageSubscriber.isNewDesiredConfigurationAvailable())
         {
            HandDesiredConfigurationMessage packet = handDesiredConfigurationMessageSubscriber.pollMessage();
            HandConfiguration state = HandConfiguration.fromByte(packet.getDesiredHandConfiguration());

            switch (state)
            {
            case CALIBRATE:
               robotiqHand.initialize();
               break;
            case OPEN:
            case CLOSE:
            case CRUSH:
            case BASIC_GRIP:
            case PINCH_GRIP:
            case WIDE_GRIP:
            case SCISSOR_GRIP:
               if (robotiqHand.isConnected())
                  robotiqHand.sendHandCommand(state);
               break;
            case OPEN_FINGERS:
            case CLOSE_FINGERS:
               if (robotiqHand.isConnected())
                  robotiqHand.sendFingersCommand(state);
               break;
            case CLOSE_THUMB:
            case OPEN_THUMB:
               if (robotiqHand.isConnected())
                  robotiqHand.sendThumbCommand(state);
               break;
            case RESET:
               if (robotiqHand.isConnected())
                  robotiqHand.reset();
               break;
            case CONNECT:
               robotiqHand.reconnect();
               break;
            case HOOK:
               //TODO
               break;
            case HALF_CLOSE:
               //TODO
               break;
            default:
               break;
            }
         }

         if (manualHandControlProvider.isNewPacketAvailable())
         {
            //TODO
         }

         ThreadTools.sleep(10);
      }
   }

   public static void main(String[] args)
   {
      JSAP jsap = new JSAP();

      FlaggedOption robotSide = new FlaggedOption("robotSide").setRequired(true).setLongFlag("robotSide").setShortFlag('r').setStringParser(JSAP.STRING_PARSER);

      try
      {
         jsap.registerParameter(robotSide);
         JSAPResult config = jsap.parse(args);

         if (config.success())
         {
            RobotiqControlThread controlThread = new RobotiqControlThread("atlas", RobotSide.valueOf(config.getString("robotSide").toUpperCase()));
            controlThread.run();
         }
      }
      catch (JSAPException e)
      {
         e.printStackTrace();
      }
   }
}