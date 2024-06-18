package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.type.ImDouble;
import us.ihmc.rdx.imgui.ImGuiPlot;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.Set;

public class RDXPlanarRegionsVisualizer extends RDXVisualizer
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private final RDXPlanarRegionsGraphic planarRegionsGraphic = new RDXPlanarRegionsGraphic();
   private final ResettableExceptionHandlingExecutorService executorService;

   private final ImGuiPlot numberOfRegionsPlot = new ImGuiPlot("# Regions", 1000, 230, 20);
   private int numberOfPlanarRegions = 0;
   private final ImDouble opacity = new ImDouble(0.7);

   public RDXPlanarRegionsVisualizer(String title)
   {
      super(title);

      boolean daemon = true;
      int queueSize = 1;
      executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), daemon, queueSize);
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

   public void destroy()
   {
      planarRegionsGraphic.destroy();
   }

   public void setNumberOfPlanarRegions(int numberOfPlanarRegions)
   {
      this.numberOfPlanarRegions = numberOfPlanarRegions;
   }

   public ResettableExceptionHandlingExecutorService getExecutorService()
   {
      return executorService;
   }

   public RDXPlanarRegionsGraphic getPlanarRegionsGraphic()
   {
      return planarRegionsGraphic;
   }
}
