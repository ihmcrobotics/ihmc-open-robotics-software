package us.ihmc.rdx.ui.graphics.live;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.rdx.ui.visualizers.ImGuiFrequencyPlot;
import us.ihmc.rdx.ui.visualizers.RDXVisualizer;
import us.ihmc.rdx.visualizers.RDXGridMapGraphic;
import us.ihmc.rdx.visualizers.RDXHeightMapGraphic;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

public class RDXHeightMapVisualizer extends RDXVisualizer implements RenderableProvider
{
   private final RDXGridMapGraphic gridMapGraphic = new RDXGridMapGraphic();
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
         gridMapGraphic.update();
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
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
