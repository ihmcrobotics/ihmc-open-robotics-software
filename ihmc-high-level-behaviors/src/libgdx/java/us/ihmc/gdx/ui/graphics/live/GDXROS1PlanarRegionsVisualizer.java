package us.ihmc.gdx.ui.graphics.live;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.sensors.realsense.DelayFixedPlanarRegionsSubscription;
import us.ihmc.avatar.sensors.realsense.MapsenseTools;
import us.ihmc.gdx.imgui.ImGuiPlot;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXROS1Visualizer;
import us.ihmc.gdx.visualizers.GDXPlanarRegionsGraphic;
import us.ihmc.robotEnvironmentAwareness.updaters.GPUPlanarRegionUpdater;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.utilities.ros.RosNodeInterface;

public class GDXROS1PlanarRegionsVisualizer extends ImGuiGDXROS1Visualizer implements RenderableProvider
{
   private final GDXPlanarRegionsGraphic planarRegionsGraphic = new GDXPlanarRegionsGraphic();
   private final String topic;

   private long receivedCount = 0;
   private final ImGuiPlot receivedPlot = new ImGuiPlot("Received", 1000, 230, 20);
   private final ImGuiPlot delayPlot = new ImGuiPlot("Delay", 1000, 230, 20);

   private final GPUPlanarRegionUpdater gpuPlanarRegionUpdater = new GPUPlanarRegionUpdater();
   private DelayFixedPlanarRegionsSubscription delayFixedPlanarRegionsSubscription;

   public GDXROS1PlanarRegionsVisualizer(String title, ROS2NodeInterface ros2Node, DRCRobotModel robotModel, String topic)
   {
      super(title);
      this.topic = topic;

      delayFixedPlanarRegionsSubscription = MapsenseTools.subscribeToPlanarRegionsWithDelayCompensation(ros2Node, robotModel, topic, planarRegionsList ->
      {
         ++receivedCount;
         planarRegionsGraphic.generateMeshes(planarRegionsList);
      });
      delayFixedPlanarRegionsSubscription.setEnabled(isActive());

      gpuPlanarRegionUpdater.attachROS2Tuner(ros2Node);
   }

   @Override
   public void subscribe(RosNodeInterface ros1Node)
   {
      delayFixedPlanarRegionsSubscription.subscribe(ros1Node);
   }

   @Override
   public void unsubscribe(RosNodeInterface ros1Node)
   {
      delayFixedPlanarRegionsSubscription.unsubscribe(ros1Node);
   }

   @Override
   public void setActive(boolean active)
   {
      super.setActive(active);
      delayFixedPlanarRegionsSubscription.setEnabled(active);
   }

   @Override
   public void renderGraphics()
   {
      super.renderGraphics();
      if (isActive())
      {
         planarRegionsGraphic.render();
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();
      delayFixedPlanarRegionsSubscription.setEnabled(isActive());
      ImGui.text(topic);
      receivedPlot.render(receivedCount);
      ImGui.text("Delay:");
      delayPlot.render((float) delayFixedPlanarRegionsSubscription.getDelay());
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (isActive())
      {
         planarRegionsGraphic.getRenderables(renderables, pool);
      }
   }

   public void destroy()
   {
      delayFixedPlanarRegionsSubscription.destroy();
      planarRegionsGraphic.destroy();
   }

   public GPUPlanarRegionUpdater getGpuPlanarRegionUpdater()
   {
      return gpuPlanarRegionUpdater;
   }
}
