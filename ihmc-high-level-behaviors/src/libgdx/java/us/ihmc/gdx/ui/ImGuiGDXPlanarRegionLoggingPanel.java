package us.ihmc.gdx.ui;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import us.ihmc.avatar.logging.PlanarRegionsListBuffer;
import us.ihmc.avatar.logging.PlanarRegionsListLogger;
import us.ihmc.gdx.visualizers.GDXPlanarRegionsGraphic;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class ImGuiGDXPlanarRegionLoggingPanel implements RenderableProvider
{
   public static final String WINDOW_NAME = "Planar Region Logging";

   private final PlanarRegionsListLogger prlLogger;
   private final PlanarRegionsListBuffer prlBuffer;

   private final GDXPlanarRegionsGraphic graphic;

   private final ImBoolean logPlanarRegions = new ImBoolean(false);
   private final ImInt time = new ImInt(0);

   public ImGuiGDXPlanarRegionLoggingPanel() {
      prlLogger = new PlanarRegionsListLogger(this.getClass().getSimpleName(), Integer.MAX_VALUE);
      prlBuffer = new PlanarRegionsListBuffer();

      graphic = new GDXPlanarRegionsGraphic();
   }

   public void create(GDXImGuiBasedUI baseUI) {
      prlLogger.start();
   }

   public void update(long time, PlanarRegionsList prl) {
      if (logPlanarRegions.get())
         prlLogger.update(time, prl);

      prlBuffer.putAndTick(time, prl);
   }

   public void renderImGuiWidgets() {
      ImGui.checkbox("Log Planar Regions", logPlanarRegions);

      ImGui.text("Max: " + (prlBuffer.getEndTime() - prlBuffer.getStartTime()));
      if (ImGui.inputInt("Position", time)) {
         PlanarRegionsList list = prlBuffer.getNearTime(prlBuffer.getStartTime() + time.get());
         if (list != null)
         {
            graphic.generateMeshesAsync(list);
         }
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      graphic.update(); //TODO should only update when something needs to be updated
      graphic.getRenderables(renderables, pool);
   }

   public void destroy() {
      graphic.destroy();
   }

   public String getWindowName()
   {
      return WINDOW_NAME;
   }
}
