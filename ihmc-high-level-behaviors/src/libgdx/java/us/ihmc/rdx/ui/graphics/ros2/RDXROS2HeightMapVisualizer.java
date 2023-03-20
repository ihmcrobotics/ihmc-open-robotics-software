package us.ihmc.rdx.ui.graphics.ros2;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.visualizers.ImGuiFrequencyPlot;
import us.ihmc.rdx.ui.visualizers.RDXVisualizer;
import us.ihmc.rdx.visualizers.RDXGridMapGraphic;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.Set;

public class RDXROS2HeightMapVisualizer extends RDXVisualizer
{
   private final RDXGridMapGraphic gridMapGraphic = new RDXGridMapGraphic();
   private final ResettableExceptionHandlingExecutorService executorService;
   private final ImGuiFrequencyPlot frequencyPlot = new ImGuiFrequencyPlot();
   private final ImBoolean inPaintHeight = new ImBoolean(false);
   private final ImBoolean renderGroundPlane = new ImBoolean(false);
   private final ImBoolean renderGroundCells = new ImBoolean(false);

   public RDXROS2HeightMapVisualizer(ROS2PublishSubscribeAPI ros2)
   {
      super("Height Map");

      boolean daemon = true;
      int queueSize = 1;
      executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), daemon, queueSize);

      ros2.subscribeViaCallback(ROS2Tools.HEIGHT_MAP_OUTPUT, this::acceptHeightMapMessage);
   }

   public void acceptHeightMapMessage(HeightMapMessage heightMapMessage)
   {
      frequencyPlot.recordEvent();
      if (isActive())
      {
         executorService.clearQueueAndExecute(() ->
         {
            gridMapGraphic.setInPaintHeight(inPaintHeight.get());
            gridMapGraphic.setRenderGroundPlane(renderGroundPlane.get());
            gridMapGraphic.setRenderGroundCells(renderGroundCells.get());
            gridMapGraphic.generateMeshesAsync(heightMapMessage);
         });
      }
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
      super.renderImGuiWidgets();

      ImGui.checkbox("In Paint Height", inPaintHeight);
      ImGui.checkbox("Render Ground Plane", renderGroundPlane);
      ImGui.checkbox("Render Ground Cells", renderGroundCells);

      if (!isActive())
      {
         executorService.interruptAndReset();
      }
//      ImGui.text(topic.getName());
      frequencyPlot.renderImGuiWidgets();
//      numberOfRegionsPlot.render(numberOfPlanarRegions);
   }

   @Override
   public void update()
   {
      super.update();
      boolean isActive = isActive();
      if (isActive)
      {
         gridMapGraphic.update();
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (isActive())
      {
         gridMapGraphic.getRenderables(renderables, pool);
      }
   }

   public void destroy()
   {
      gridMapGraphic.destroy();
   }
}
