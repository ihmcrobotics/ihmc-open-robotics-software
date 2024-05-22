package us.ihmc.rdx.ui.graphics.ros2;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.RigidBodyTransformMessage;
import imgui.ImGui;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.RDXTrajectoryGraphic;
import us.ihmc.rdx.ui.graphics.RDXVisualizer;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.string.StringTools;

import java.util.Set;
import java.util.concurrent.ConcurrentLinkedQueue;

public class RDXROS2TrajectoryVisualizer extends RDXVisualizer
{
   private static final int DEFAULT_HISTORY_LENGTH = 50;

   private final String titleBeforeAdditions;
   private final PubSubImplementation pubSubImplementation;
   private final ROS2Topic<RigidBodyTransformMessage> topic;
   private RealtimeROS2Node realtimeROS2Node;

   private final RDXTrajectoryGraphic trajectoryGraphic = new RDXTrajectoryGraphic();
   private final ConcurrentLinkedQueue<RigidBodyTransform> trajectories;

   private final ImInt historyLength = new ImInt(50);
   private final ImFloat lineWidth = new ImFloat(0.01f);

   public RDXROS2TrajectoryVisualizer(String title, ROS2Topic<RigidBodyTransformMessage> topic)
   {
      this(title, PubSubImplementation.FAST_RTPS, topic, DEFAULT_HISTORY_LENGTH);
   }

   public RDXROS2TrajectoryVisualizer(String title, PubSubImplementation pubSubImplementation, ROS2Topic<RigidBodyTransformMessage> topic)
   {
      this(title, pubSubImplementation, topic, DEFAULT_HISTORY_LENGTH);
   }

   public RDXROS2TrajectoryVisualizer(String title, PubSubImplementation pubSubImplementation, ROS2Topic<RigidBodyTransformMessage> topic, int historyLength)
   {
      super(title);

      this.titleBeforeAdditions = title;
      this.pubSubImplementation = pubSubImplementation;
      this.topic = topic;

      trajectories = new ConcurrentLinkedQueue<>();

      setActivenessChangeCallback(isActive ->
      {
         if (isActive && realtimeROS2Node == null)
            subscribe();
         else if (!isActive && realtimeROS2Node != null)
            unsubscribe();
      });
   }

   private void subscribe()
   {
      // if already subscribed, unsubscribe
      if (realtimeROS2Node != null)
         unsubscribe();

      // subscribe
      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(pubSubImplementation, StringTools.titleToSnakeCase(titleBeforeAdditions));
      realtimeROS2Node.createSubscription(topic, this::updateGraphic);
      realtimeROS2Node.spin();
   }

   private void unsubscribe()
   {
      if (realtimeROS2Node != null)
      {
         realtimeROS2Node.destroy();
         realtimeROS2Node = null;
      }
      trajectories.clear();
   }

   private void updateGraphic(Subscriber<RigidBodyTransformMessage> subscriber)
   {
      if (subscriber.isAvailable())
      {
         RigidBodyTransform newTransform = MessageTools.toEuclid(subscriber.takeNextData());

         // Remove old trajectories over the history length and add new trajectory
         while (trajectories.size() >= historyLength.get())
            trajectories.poll();
         trajectories.offer(newTransform);
      }
   }

   @Override
   public void update()
   {
      super.update();

      trajectoryGraphic.update(lineWidth.get(), trajectories);
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();

      ImGui.inputInt("History Lenght", historyLength);
      ImGui.inputFloat("Line width", lineWidth);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      super.getRenderables(renderables, pool, sceneLevels);

      trajectoryGraphic.getRenderables(renderables, pool);
   }

   @Override
   public void destroy()
   {
      unsubscribe();
      super.destroy();
   }
}
