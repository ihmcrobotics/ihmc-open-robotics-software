package us.ihmc.gdx.ui.visualizers;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosTools;

import java.util.ArrayList;

public class ImGuiGDXGlobalVisualizersPanel implements RenderableProvider
{
   private static final String WINDOW_NAME = "Global Visualizers";

   private final RemoteSyncedRobotModel syncedRobot;
   private final ArrayList<ImGuiGDXVisualizer> visualizers = new ArrayList<>();

   private RosMainNode ros1Node;

   public ImGuiGDXGlobalVisualizersPanel(DRCRobotModel robotModel, ROS2Node ros2Node)
   {
      syncedRobot = new RemoteSyncedRobotModel(robotModel, ros2Node); // TODO: Only create this when enabled
   }

   public void addVisualizer(ImGuiGDXVisualizer visualizer)
   {
      visualizers.add(visualizer);
   }

   public void create()
   {
      for (ImGuiGDXVisualizer visualizer : visualizers)
      {
         visualizer.create();
      }

      recreateRos1Node();
   }

   public void recreateRos1Node()
   {
      if (ros1Node != null)
         ros1Node.shutdown();

      ros1Node = RosTools.createRosNode(NetworkParameters.getROSURI(), "global_visualizers");

      for (ImGuiGDXVisualizer visualizer : visualizers)
      {
         if (visualizer instanceof ImGuiGDXROS1Visualizer && visualizer.isActive())
         {
            ((ImGuiGDXROS1Visualizer) visualizer).subscribe(ros1Node);
         }
      }

      ros1Node.execute();
   }

   public void render()
   {
      if (syncedRobot != null)
         syncedRobot.update();

      ImGui.begin(WINDOW_NAME);

      boolean anyNewROS1Enabled = false;

      for (ImGuiGDXVisualizer visualizer : visualizers)
      {
         visualizer.renderImGuiWidgets();
         if (visualizer instanceof ImGuiGDXROS1Visualizer)
         {
            anyNewROS1Enabled |= visualizer.getActiveChanged() && visualizer.isActive();
            if (visualizer.getActiveChanged() && !visualizer.isActive())
            {
               ((ImGuiGDXROS1Visualizer) visualizer).unsubscribe(ros1Node);
            }
         }
         ImGui.separator();
      }

      ImGui.end();

      if (anyNewROS1Enabled)
      {
         recreateRos1Node();
      }

      for (ImGuiGDXVisualizer visualizer : visualizers)
      {
         if (visualizer.isActive())
         {
            visualizer.renderGraphics();
         }
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (ImGuiGDXVisualizer visualizer : visualizers)
      {
         if (visualizer.isActive())
         {
            visualizer.getRenderables(renderables, pool);
         }
      }
   }

   // TODO: Remove, this is temporary
   public RemoteSyncedRobotModel getSyncedRobot()
   {
      return syncedRobot;
   }

   public void destroy()
   {
      for (ImGuiGDXVisualizer visualizer : visualizers)
      {
         visualizer.destroy();
      }
   }

   public String getWindowName()
   {
      return WINDOW_NAME;
   }
}