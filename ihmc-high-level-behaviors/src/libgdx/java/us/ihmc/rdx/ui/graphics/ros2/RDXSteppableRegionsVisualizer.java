package us.ihmc.rdx.ui.graphics.ros2;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import perception_msgs.msg.dds.SteppableRegionsListCollectionMessage;
import us.ihmc.perception.steppableRegions.SteppableRegionsAPI;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.RDXSteppableRegionGraphic;
import us.ihmc.ros2.ROS2Callback;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.Set;

public class RDXSteppableRegionsVisualizer extends RDXROS2SingleTopicVisualizer<SteppableRegionsListCollectionMessage>
{
   private final RDXSteppableRegionGraphic steppableRegionGraphic = new RDXSteppableRegionGraphic();

   private final ResettableExceptionHandlingExecutorService executorService;
   private final ImInt yawToShow = new ImInt(0);
   private final ImBoolean renderHeightMap = new ImBoolean(true);
   private final ImBoolean renderPlanes = new ImBoolean(false);
   private int receivedRegions = -1;

   public RDXSteppableRegionsVisualizer(String title)
   {
      super(title);

      boolean daemon = true;
      int queueSize = 1;
      executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), daemon, queueSize);
   }

   @Override
   public ROS2Topic<SteppableRegionsListCollectionMessage> getTopic()
   {
      return SteppableRegionsAPI.STEPPABLE_REGIONS_OUTPUT;
   }

   @Override
   public void create()
   {
      super.create();

      setActive(true);
   }

   public void setUpForNetworking(ROS2Node ros2Node)
   {
      new ROS2Callback<>(ros2Node, SteppableRegionsAPI.STEPPABLE_REGIONS_OUTPUT, this::acceptSteppableRegionsCollection);
   }

   public void acceptSteppableRegionsCollection(SteppableRegionsListCollectionMessage steppableRegionsListCollection)
   {
      if (isActive())
      {
         if (steppableRegionsListCollection == null)
            return;

         receivedRegions = steppableRegionsListCollection.getRegionsPerYaw().get(yawToShow.get());
         executorService.clearQueueAndExecute(() ->
         {
            steppableRegionGraphic.setRenderHeightMap(renderHeightMap.get());
            steppableRegionGraphic.setRenderPlanes(renderPlanes.get());
            steppableRegionGraphic.generateMeshesAsync(steppableRegionsListCollection, yawToShow.get());
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
      ImGui.checkbox("Render Height Map", renderHeightMap);
      ImGui.checkbox("Render Planes", renderPlanes);

      if (!isActive())
      {
         executorService.interruptAndReset();
      }
      ImGui.text("Regions rendered: " + receivedRegions);
      ImGui.sliderInt("Yaw to show", yawToShow.getData(), 0, 4);
   }

   @Override
   public void update()
   {
      super.update();

      if (isActive())
      {
         steppableRegionGraphic.update();
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (isActive() && sceneLevelCheck(sceneLevels))
      {
         steppableRegionGraphic.getRenderables(renderables, pool);
      }
   }

   public void destroy()
   {
      steppableRegionGraphic.destroy();
   }
}
