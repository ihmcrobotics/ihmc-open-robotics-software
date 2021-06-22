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

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.Objects;

public class ImGuiGDXPlanarRegionLoggingPanel implements RenderableProvider
{
   public static final String WINDOW_NAME = "Planar Region Logging";

   private final PlanarRegionsListLogger prlLogger;
   private final PlanarRegionsListBuffer prlRealtimeBuffer;
   private PlanarRegionsListBuffer prlLogBuffer;

   private final GDXPlanarRegionsGraphic realtimeGraphic;
   private final GDXPlanarRegionsGraphic logGraphic;

   private final ImBoolean logPlanarRegions = new ImBoolean(false);
   private final ImBoolean showRealtime = new ImBoolean(false);
   private final ImBoolean showLog = new ImBoolean(false);
   private final ImInt timeRealtime = new ImInt(0);
   private final ImInt timeLog = new ImInt(0);

   private boolean firstRun = true;
   private boolean mustUpdateFiles = false;

   private final ArrayList<File> files = new ArrayList<>();

   public ImGuiGDXPlanarRegionLoggingPanel() {
      prlLogger = new PlanarRegionsListLogger(this.getClass().getSimpleName(), Integer.MAX_VALUE);
      prlRealtimeBuffer = new PlanarRegionsListBuffer();

      realtimeGraphic = new GDXPlanarRegionsGraphic();
      logGraphic = new GDXPlanarRegionsGraphic();
   }

   public void create(GDXImGuiBasedUI baseUI) {
      updateAvailableLogFiles();
   }

   public void update(long time, PlanarRegionsList prl) {
      if (logPlanarRegions.get())
      {
         prlLogger.update(time, prl);
         if (mustUpdateFiles)
         {
            mustUpdateFiles = false;
            updateAvailableLogFiles();
         }
      }

      prlRealtimeBuffer.putAndTick(time, prl);
   }

   private void updateAvailableLogFiles() {
      File logDirectory = Paths.get(PlanarRegionsListLogger.getLogDirectory()).toFile();
      files.clear();

      try
      {
         files.addAll(Arrays.asList(Objects.requireNonNull(logDirectory.listFiles(pathname -> pathname.toString().toLowerCase().endsWith(".prllog")))));
      } catch (NullPointerException ex) {
         LogTools.error("Could not open log directory - does folder exist?");
         LogTools.error(ex.getStackTrace());
      }

      files.sort((o1, o2) -> //Sorts most recent first
      {
         BasicFileAttributes left = null;
         BasicFileAttributes right = null;
         try
         {
            left = Files.readAttributes(o1.toPath(), BasicFileAttributes.class);
            right = Files.readAttributes(o2.toPath(), BasicFileAttributes.class);
         }
         catch (IOException ex)
         {
            LogTools.error("Could not sort files");
            LogTools.error(ex.getStackTrace());

            return 0;
         }

         return right.creationTime().compareTo(left.creationTime());
      });
   }

   public void renderImGuiWidgets() {
      if (ImGui.checkbox("Log Planar Regions", logPlanarRegions)) {
         if (logPlanarRegions.get())
         {
            prlLogger.start();
            mustUpdateFiles = true;
         }
      }

      ImGui.text("Max: " + (prlRealtimeBuffer.getEndTime() - prlRealtimeBuffer.getStartTime()));
      if (ImGui.inputInt("Position (Realtime)", timeRealtime) || firstRun) {
         PlanarRegionsList list = prlRealtimeBuffer.getNearTime(prlRealtimeBuffer.getStartTime() + timeRealtime.get());
         if (list != null)
         {
            realtimeGraphic.generateMeshesAsync(list);
         }
      }
      ImGui.sameLine();
      ImGui.checkbox("Show realtime buffer", showRealtime);

      if (prlLogBuffer != null)
      {
         ImGui.text("Max: " + (prlLogBuffer.getEndTime() - prlLogBuffer.getStartTime()));
         if (ImGui.inputInt("Position (Log)", timeLog) || firstRun)
         {
            PlanarRegionsList list = prlLogBuffer.getNearTime(prlLogBuffer.getStartTime() + timeLog.get());
            if (list != null)
            {
               logGraphic.generateMeshesAsync(list);
            }
         }
      } else {
         ImGui.text("Max: N/A");
         ImGui.inputInt("Position (Log)", timeLog);
      }
      ImGui.sameLine();
      ImGui.checkbox("Show selected log", showLog);

      if (ImGui.beginListBox("Log files")) {
         if (files != null) {
            Iterator<File> it = files.iterator();
            while (it.hasNext()) { //Use iterator to prevent rare ConcurrentModificationError
               File f = it.next();
               if (ImGui.selectable(f.getName())) {
                  try
                  {
                     prlLogBuffer = new PlanarRegionsListBuffer(f);
                  } catch (IOException ex) {
                     LogTools.error("Could not load file " + f.getName());
                     LogTools.error(ex);
                  }
               }
            }
         }

         ImGui.endListBox();
      }

      if (ImGui.button("Refresh logs")) {
         updateAvailableLogFiles();
      }

      firstRun = false;
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (showRealtime.get()) {
         realtimeGraphic.update();
         realtimeGraphic.getRenderables(renderables, pool);
      }

      if (showLog.get()) {
         logGraphic.update();
         logGraphic.getRenderables(renderables, pool);
      }
   }

   public void destroy() {
      realtimeGraphic.destroy();
      logGraphic.destroy();
   }

   public String getWindowName()
   {
      return WINDOW_NAME;
   }
}
