package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiCond;
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
      this.title = title;
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

   protected void renderMenuEntry(boolean collapse, boolean renderRightContext)
   {
      StringBuilder rightAlignedContextText = new StringBuilder();
      StringBuilder rightAlignedContextTextTooltip = new StringBuilder();

      if (this instanceof RDXROS2SingleTopicVisualizer<?> topicVisualizer)
      {
         rightAlignedContextText.append(topicVisualizer.getFrequency().getText());
         rightAlignedContextTextTooltip.append(topicVisualizer.getTopic().getName())
                                       .append(" (")
                                       .append(topicVisualizer.getTopic().getQoS().getReliabilityKind().name())
                                       .append(")");
      }
      else if (this instanceof RDXROS2MultiTopicVisualizer multiTopicVisualizer)
      {
         for (ROS2Topic<?> topic : multiTopicVisualizer.getTopics())
         {
            rightAlignedContextText.append(multiTopicVisualizer.getFrequency(topic).getText());
            rightAlignedContextTextTooltip.append(topic.getName())
                                          .append(" (")
                                          .append(topic.getQoS().getReliabilityKind().name())
                                          .append(")\n");
         }
      }

      if (ImGui.checkbox(labels.get("##Active"), active))
      {
         setActive(active.get());
         if (this instanceof RDXROS2RobotVisualizer robotVisualizer)
         {
            robotVisualizer.visualizeSensors(active.get());
         }

         if (getPanel() != null)
            getPanel().getIsShowing().set(active.get());
      }
      ImGui.sameLine();

      if (collapse)
         ImGui.setNextItemOpen(false, ImGuiCond.Always);
      float preHeaderCursorY = ImGui.getCursorPosY();
      if (ImGui.collapsingHeader(labels.get(title)))
      {
         renderImGuiWidgets();
      }
      float postHeaderCursorY = ImGui.getCursorPosY();
      if (renderRightContext)
      {
         ImGui.setCursorPosY(preHeaderCursorY + (ImGui.getTextLineHeight() / 2) - 2);
         ImGuiTools.rightAlignText(rightAlignedContextText.toString());
         ImGui.setCursorPosY(postHeaderCursorY);
         ImGuiTools.previousWidgetTooltip(rightAlignedContextTextTooltip.toString());
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
