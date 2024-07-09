package us.ihmc.rdx.ui.graphics.ros2;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.type.ImDouble;
import perception_msgs.msg.dds.FramePlanarRegionsListMessage;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.rdx.imgui.ImGuiPlot;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.Set;

public class RDXROS2FramePlanarRegionsVisualizer extends RDXROS2SingleTopicVisualizer<FramePlanarRegionsListMessage>
{
   private final ROS2Topic<FramePlanarRegionsListMessage> topic;

   private final RDXPlanarRegionsGraphic planarRegionsGraphic = new RDXPlanarRegionsGraphic();
   private final ResettableExceptionHandlingExecutorService executorService;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiPlot numberOfRegionsPlot = new ImGuiPlot("# Regions", 1000, 230, 20);
   private int numberOfPlanarRegions = 0;
   private final ImDouble opacity = new ImDouble(0.7);

   public RDXROS2FramePlanarRegionsVisualizer(String title, ROS2NodeInterface ros2Node, ROS2Topic<FramePlanarRegionsListMessage> topic)
   {
      super(title);

      this.topic = topic;

      boolean daemon = true;
      int queueSize = 1;
      executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), daemon, queueSize);

      ros2Node.createSubscription2(topic, this::acceptMessage);
   }

   private void acceptMessage(FramePlanarRegionsListMessage framePlanarRegionsListMessage)
   {
      if (isActive())
      {
         executorService.clearQueueAndExecute(() ->
         {
            PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsListInWorld(framePlanarRegionsListMessage);
            numberOfPlanarRegions = planarRegionsList.getNumberOfPlanarRegions();
            planarRegionsGraphic.generateMeshes(planarRegionsList);
         });
      }

      getFrequency().ping();
   }

   @Override
   public void setActive(boolean active)
   {
      super.setActive(active);

      if (!isActive())
      {
         executorService.interruptAndReset();
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      if (!isActive())
      {
         executorService.interruptAndReset();
      }
      numberOfRegionsPlot.render(numberOfPlanarRegions);
      ImGuiTools.sliderDouble(labels.get("Opacity"), opacity, 0.1, 1.0);
      planarRegionsGraphic.setBlendOpacity((float) opacity.get());
   }

   @Override
   public void update()
   {
      super.update();

      if (isActive())
      {
         planarRegionsGraphic.update();
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (isActive() && sceneLevelCheck(sceneLevels))
      {
         planarRegionsGraphic.getRenderables(renderables, pool);
      }
   }

   @Override
   public ROS2Topic<FramePlanarRegionsListMessage> getTopic()
   {
      return topic;
   }

   @Override
   public void destroy()
   {
      super.destroy();

      planarRegionsGraphic.destroy();
   }
}
