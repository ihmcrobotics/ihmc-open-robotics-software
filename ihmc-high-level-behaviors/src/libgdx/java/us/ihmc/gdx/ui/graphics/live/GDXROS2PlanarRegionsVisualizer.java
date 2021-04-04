package us.ihmc.gdx.ui.graphics.live;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import imgui.internal.ImGui;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.gdx.imgui.ImGuiPlot;
import us.ihmc.gdx.visualizers.GDXPlanarRegionsGraphic;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.MissingThreadTools;

import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

public class GDXROS2PlanarRegionsVisualizer implements RenderableProvider
{
   private final GDXPlanarRegionsGraphic planarRegionsGraphic = new GDXPlanarRegionsGraphic();
   private final ResettableExceptionHandlingExecutorService executorService;
   private final ROS2Topic<PlanarRegionsListMessage> topic;
   private boolean enabled = false;

   private long receivedCount = 0;
   private final ImGuiPlot receivedPlot = new ImGuiPlot("", 1000, 230, 20);

   public GDXROS2PlanarRegionsVisualizer(ROS2NodeInterface ros2Node, ROS2Topic<PlanarRegionsListMessage> topic)
   {
      this.topic = topic;
      new IHMCROS2Callback<>(ros2Node, topic, this::acceptMessage);

      boolean daemon = true;
      int queueSize = 1;
      executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), daemon, queueSize);
   }

   private void acceptMessage(PlanarRegionsListMessage planarRegionsListMessage)
   {
      ++receivedCount;
      if (enabled)
      {
         executorService.clearQueueAndExecute(() ->
         {
            PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage);
            planarRegionsGraphic.generateMeshes(planarRegionsList);
         });
      }
   }

   public void setEnabled(boolean enabled)
   {
      this.enabled = enabled;

      if (!enabled)
      {
         executorService.interruptAndReset();
      }
   }

   public void render()
   {
      if (enabled)
      {
         planarRegionsGraphic.render();
      }

      ImGui.text(topic.getName());
      receivedPlot.render(receivedCount);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (enabled)
      {
         planarRegionsGraphic.getRenderables(renderables, pool);
      }
   }

   public void destroy()
   {
      planarRegionsGraphic.destroy();
   }
}
