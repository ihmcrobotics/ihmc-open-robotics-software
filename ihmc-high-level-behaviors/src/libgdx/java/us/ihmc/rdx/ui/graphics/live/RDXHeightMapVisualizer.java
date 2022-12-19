package us.ihmc.rdx.ui.graphics.live;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.rdx.ui.visualizers.ImGuiFrequencyPlot;
import us.ihmc.rdx.ui.visualizers.RDXVisualizer;
import us.ihmc.rdx.visualizers.RDXHeightMapGraphic;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

public class RDXHeightMapVisualizer extends RDXVisualizer implements RenderableProvider
{
   private final RDXHeightMapGraphic heightMapGraphic = new RDXHeightMapGraphic();
   private final ResettableExceptionHandlingExecutorService executorService;
   private final ImGuiFrequencyPlot frequencyPlot = new ImGuiFrequencyPlot();

   public RDXHeightMapVisualizer(String title)
   {
      super(title);

      boolean daemon = true;
      int queueSize = 1;
      executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), daemon, queueSize);
   }

   public void acceptHeightMapMessage(HeightMapMessage heightMapMessage)
   {
      frequencyPlot.recordEvent();
      if (isActive())
      {
         executorService.clearQueueAndExecute(() ->
                                              {
                                                 heightMapGraphic.generateMeshes(heightMapMessage);
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
      if (isActive())
      {
         heightMapGraphic.update();
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (isActive())
      {
         heightMapGraphic.getRenderables(renderables, pool);
      }
   }

   public void destroy()
   {
      heightMapGraphic.destroy();
   }
}
