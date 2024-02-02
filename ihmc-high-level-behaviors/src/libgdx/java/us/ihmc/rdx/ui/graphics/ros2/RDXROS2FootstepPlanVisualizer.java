package us.ihmc.rdx.ui.graphics.ros2;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.FootstepDataListMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.RDXVisualizer;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.string.StringTools;

import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;

/**
* This class is an RDX global visualizer that is supposed to subscribe to a footstep data list message and visualize it in the UI.
 * Useful when any remote module is performing footstep planning to help the human operator understand the plan. It can function as a standalone visualizer
 * for any footstep plan based FootstepDataListMessage.
* */
public class RDXROS2FootstepPlanVisualizer extends RDXVisualizer
{
   private final String titleBeforeAdditions;
   private final DomainFactory.PubSubImplementation pubSubImplementation;
   private final ROS2Topic<FootstepDataListMessage> topic;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private boolean subscribed = false;
   private ROS2Node ros2Node;
   private final Object syncObject = new Object();
   private AtomicReference<FootstepDataListMessage> footstepDataListMessage = new AtomicReference<>(null);

   public RDXROS2FootstepPlanVisualizer(String title, DomainFactory.PubSubImplementation pubSubImplementation, ROS2Topic<FootstepDataListMessage> topic)
   {
      super(title + " (ROS 2)");
      titleBeforeAdditions = title;
      this.pubSubImplementation = pubSubImplementation;
      this.topic = topic;

      setActivenessChangeCallback(isActive ->
      {
         if (isActive && ros2Node == null)
         {
            subscribe();
         }
         else if (!isActive && ros2Node != null)
         {
            unsubscribe();
         }
      });
   }

   private void subscribe()
   {
      subscribed = true;
      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, StringTools.titleToSnakeCase(titleBeforeAdditions));
      ROS2Tools.createCallbackSubscription(ros2Node, this.topic, this::queueFootstepDataListMessage);
   }

   private void queueFootstepDataListMessage(Subscriber<FootstepDataListMessage> subscriber)
   {
      synchronized (syncObject)
      {

      }
   }

   @Override
   public void create()
   {
      super.create();
   }

   @Override
   public void update()
   {

   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();
      ImGui.text(topic.getName());
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (isActive() && sceneLevelCheck(sceneLevels))
      {

      }
   }

   @Override
   public void destroy()
   {
      unsubscribe();
      super.destroy();
   }

   private void unsubscribe()
   {
      subscribed = false;
      if (ros2Node != null)
      {
         ros2Node.destroy();
         ros2Node = null;
      }
   }

   public boolean isSubscribed()
   {
      return subscribed;
   }
}
