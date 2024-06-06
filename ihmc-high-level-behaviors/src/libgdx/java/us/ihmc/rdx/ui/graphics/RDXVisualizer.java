package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import std_msgs.msg.dds.Empty;
import us.ihmc.communication.ros2.ROS2Heartbeat;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.sceneManager.RDXRenderableProvider;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2MultiTopicVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2RobotVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2SingleTopicVisualizer;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

import javax.annotation.Nullable;
import java.util.Set;
import java.util.function.Consumer;

public abstract class RDXVisualizer implements RDXRenderableProvider
{
   private final ImBoolean active = new ImBoolean(false);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final String title;
   private boolean createdYet = false;
   private Set<RDXSceneLevel> sceneLevels = Set.of(RDXSceneLevel.MODEL);
   private Consumer<Boolean> activenessChangeCallback;
   private boolean wasActive = false;

   @Nullable
   private ROS2Heartbeat heartbeat;

   // If this visualizer should be at the top of the list in all visualizers
   private boolean pinned;

   public RDXVisualizer(String title)
   {
      this.title = ImGuiTools.uniqueLabel(title);
   }

   public void createRequestHeartbeat(ROS2Node node, ROS2Topic<Empty> heartbeatTopic)
   {
      heartbeat = new ROS2Heartbeat(node, heartbeatTopic);
      heartbeat.setAlive(isActive());
   }

   public boolean hasHeartbeat()
   {
      return heartbeat != null;
   }

   @Nullable
   public ROS2Heartbeat getHeartbeat()
   {
      return heartbeat;
   }

   public void create()
   {
      createdYet = true;
      if (getPanel() != null)
      {
         setActive(getPanel().getIsShowing().get());
      }
   }

   protected void renderMenuEntry()
   {
      ImGui.tableNextRow();
      ImGui.tableNextColumn();

      StringBuilder tooltip = new StringBuilder();
      if (this instanceof RDXROS2SingleTopicVisualizer<?> topicVisualizer)
      {
         tooltip = new StringBuilder("ROS\n" + topicVisualizer.getTopic().getName());
         topicVisualizer.getFrequency().render();
         ImGui.sameLine();
      }
      else if (this instanceof RDXROS2MultiTopicVisualizer multiTopicVisualizer)
      {
         tooltip = new StringBuilder("ROS\n");
         for (ROS2Topic<?> topic : multiTopicVisualizer.getTopics())
         {
            tooltip.append(topic.getName()).append("\n");
            multiTopicVisualizer.getFrequency(topic).render();
            ImGui.sameLine();
         }
      }
      if (tooltip.length() > 0)
      {
         ImGuiTools.previousWidgetTooltip(tooltip.toString());
      }

      ImGui.tableNextColumn();
      if (ImGui.checkbox(labels.get("##Active"), active))
      {
         setActive(active.get());
         if (this instanceof RDXROS2RobotVisualizer robotVisualizer)
         {
            robotVisualizer.visualizeSensors(active.get());
         }

         if (getPanel() != null)
            getPanel().getIsShowing().set(active.get());

         System.out.println("set panel");
      }
      ImGui.sameLine();

      if (ImGui.collapsingHeader(labels.get(title)))
      {
         renderImGuiWidgets();
      }

      if (getPanel() != null)
      {
         if (!getPanel().getIsShowing().get())
            setActive(false);
      }

      if (heartbeat != null)
         heartbeat.setAlive(isActive());
   }

   public abstract void renderImGuiWidgets();

   /**
    * Only called when active.
    */
   public void update()
   {
      if (!createdYet)
      {
         create();
      }
   }

   /**
    * It is assumed by extending classes that this will be called when the active
    * state changes.
    */
   public void setActive(boolean active)
   {
      this.active.set(active);

      if (activenessChangeCallback != null && active != wasActive)
      {
         wasActive = active;
         activenessChangeCallback.accept(active);
      }
   }

   public boolean isActive()
   {
      return active.get();
   }

   /**
    * Only called when active.
    */
   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {

   }

   public void setSceneLevels(RDXSceneLevel... sceneLevels)
   {
      this.sceneLevels = Set.of(sceneLevels);
   }

   public boolean sceneLevelCheck(Set<RDXSceneLevel> sceneLevels)
   {
      for (RDXSceneLevel sceneLevel : this.sceneLevels)
         if (sceneLevels.contains(sceneLevel))
            return true;
      return false;
   }

   public void setActivenessChangeCallback(Consumer<Boolean> activenessChangeCallback)
   {
      this.activenessChangeCallback = activenessChangeCallback;
   }

   @Nullable
   public RDXPanel getPanel()
   {
      return null;
   }

   public void destroy()
   {
      if (heartbeat != null)
         heartbeat.destroy();
   }

   public String getTitle()
   {
      return title;
   }

   public void setPinned(boolean pinned)
   {
      this.pinned = pinned;
   }

   public boolean isPinned()
   {
      return pinned;
   }
}
