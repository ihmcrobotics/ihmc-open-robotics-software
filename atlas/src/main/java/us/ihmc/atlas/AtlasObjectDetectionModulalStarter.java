package us.ihmc.atlas;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;

import controller_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.fiducialDetectorToolBox.FiducialDetectorToolboxModule;
import us.ihmc.avatar.networkProcessor.objectDetectorToolBox.ObjectDetectorToolboxModule;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.PausablePeriodicThread;

public class AtlasObjectDetectionModulalStarter
{
   public static void main(String[] args) throws JSAPException
   {
      JSAP jsap = new JSAP();

      FlaggedOption robotModelFlag = new FlaggedOption("robotModel").setLongFlag("model").setShortFlag('m').setRequired(true)
                                                                    .setStringParser(JSAP.STRING_PARSER);

      robotModelFlag.setHelp("Robot models: " + AtlasRobotModelFactory.robotModelsToString());
      jsap.registerParameter(robotModelFlag);

      JSAPResult config = jsap.parse(args);

      DRCRobotModel robotModel;

      try
      {
         robotModel = AtlasRobotModelFactory.createDRCRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ.toString(), RobotTarget.SCS, false);
      }
      catch (IllegalArgumentException e)
      {
         System.err.println("Incorrect robot model " + config.getString("robotModel"));
         System.out.println(jsap.getHelp());

         return;
      }

      boolean startYoVariableServer = true;
      PubSubImplementation pubSubImplementation = PubSubImplementation.FAST_RTPS;
      new ObjectDetectorToolboxModule(robotModel.getSimpleRobotName(),robotModel.createFullRobotModel(),robotModel.getLogModelProvider(),pubSubImplementation);

      // Start object detector toolbox
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "toolboxes");
      String robotName = robotModel.getSimpleRobotName();
      IHMCROS2Publisher<ToolboxStateMessage> fiducialDetectorPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node,
                                                                                                            ToolboxStateMessage.class,
                                                                                                            FiducialDetectorToolboxModule.getInputTopic(
                                                                                                                  robotName));
      IHMCROS2Publisher<ToolboxStateMessage> objectDetectorPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node,
                                                                                                          ToolboxStateMessage.class,
                                                                                                          ObjectDetectorToolboxModule.getInputTopic(robotName));

      new PausablePeriodicThread("ToolboxWaker", 1.0, () ->
      {
         ToolboxStateMessage wakeUpMessage = new ToolboxStateMessage();
         wakeUpMessage.setRequestedToolboxState(ToolboxStateMessage.WAKE_UP);
         fiducialDetectorPublisher.publish(wakeUpMessage);
         objectDetectorPublisher.publish(wakeUpMessage);
      }).start();

   }
}
