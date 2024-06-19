package us.ihmc.rdx.ui.graphics.ros2;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import perception_msgs.msg.dds.FramePlanarRegionsListMessage;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.RDXPlanarRegionsVisualizer;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;

import java.util.Set;

public class RDXROS2FramePlanarRegionsVisualizer extends RDXROS2SingleTopicVisualizer<FramePlanarRegionsListMessage>
{
   private final RDXPlanarRegionsVisualizer planarRegionsVisualizer;
   private final ROS2Topic<FramePlanarRegionsListMessage> topic;
   private PlanarRegionsList planarRegionsList;

   public RDXROS2FramePlanarRegionsVisualizer(String title, ROS2NodeInterface ros2Node, ROS2Topic<FramePlanarRegionsListMessage> topic)
   {
      super(title);
      planarRegionsVisualizer = new RDXPlanarRegionsVisualizer(title);
      this.topic = topic;
      ros2Node.createSubscription2(topic, this::acceptMessage);
   }

   private void acceptMessage(FramePlanarRegionsListMessage framePlanarRegionsListMessage)
   {
      if (isActive())
      {
         planarRegionsVisualizer.getExecutorService().clearQueueAndExecute(() ->
         {
            planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsListInWorld(framePlanarRegionsListMessage);
            planarRegionsVisualizer.setNumberOfPlanarRegions(planarRegionsList.getNumberOfPlanarRegions());
            planarRegionsVisualizer.getPlanarRegionsGraphic().generateMeshes(planarRegionsList);
         });
      }

      getFrequency().ping();
   }

   @Override
   public void renderImGuiWidgets()
   {
      ImGui.text("Number of planar regions: %d".formatted(planarRegionsList.getNumberOfPlanarRegions()));
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (isActive())
      {
         planarRegionsVisualizer.getRenderables(renderables, pool, sceneLevels);
      }
   }

   @Override
   public ROS2Topic<FramePlanarRegionsListMessage> getTopic()
   {
      return topic;
   }
}
