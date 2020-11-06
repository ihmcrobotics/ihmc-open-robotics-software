package us.ihmc.atlas.behaviors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidBehaviors.BehaviorRegistry;
import us.ihmc.humanoidBehaviors.RemoteBehaviorInterface;
import us.ihmc.humanoidBehaviors.ui.behaviors.LookAndStepRemoteVisualizer;
import us.ihmc.pubsub.DomainFactory;

public class AtlasLookAndStepRemoteVisualizer
{
   public static void main(String[] args)
   {
      new LookAndStepRemoteVisualizer(new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS),
                                                          ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "look_and_step_visualizer"),
                                                          RemoteBehaviorInterface.createForUI(BehaviorRegistry.DEFAULT_BEHAVIORS, "127.0.0.1"));
   }
}
