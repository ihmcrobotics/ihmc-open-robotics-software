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

      //Realtime buffer
      ImGui.text("View current session's regions:");
      ImGui.checkbox("Show##1", showRealtime);
      boolean realtime = ImGui.sliderInt("Time:##realtimeSlider", timeRealtime.getData(), 0, (int)(prlRealtimeBuffer.getEndTime() - prlRealtimeBuffer.getStartTime()), "");
      ImGui.sameLine();
      realtime |= ImGui.inputInt("##realtimeInput", timeRealtime);

      if (ImGui.button("<<<##realtime")) {
         timeRealtime.set(Math.max(timeRealtime.get() - 3000, 0));
         realtime = true;
      }
      ImGui.sameLine();
      if (ImGui.button("<-##realtime")) {
         long start = prlRealtimeBuffer.getStartTime();
         timeRealtime.set(Math.max((int)(prlRealtimeBuffer.getPreviousTime(start + timeRealtime.get()) - start), 0));
         realtime = true;
      }
      ImGui.sameLine();
      if (ImGui.button("->##realtime")) {
         long start = prlRealtimeBuffer.getStartTime();
         timeRealtime.set(Math.min((int)(prlRealtimeBuffer.getNextTime(start + timeRealtime.get()) - start), (int)(prlRealtimeBuffer.getEndTime() - prlRealtimeBuffer.getStartTime())));
         realtime = true;
      }
      ImGui.sameLine();
      if (ImGui.button(">>>##realtime")) {
         timeRealtime.set(Math.min(timeRealtime.get() + 3000, (int)(prlRealtimeBuffer.getEndTime() - prlRealtimeBuffer.getStartTime())));
         realtime = true;
      }

      if (realtime || firstRun) {
         PlanarRegionsList list = prlRealtimeBuffer.getNearTime(prlRealtimeBuffer.getStartTime() + timeRealtime.get());
         if (list != null)
         {
            realtimeGraphic.generateMeshesAsync(list);
         }
      }

      //Log buffer
      ImGui.text("View logged regions:");
      ImGui.checkbox("Show##2", showLog);
      if (prlLogBuffer != null)
      {
         boolean log = ImGui.sliderInt("Time:##logSlider", timeLog.getData(), 0, (int)(prlLogBuffer.getEndTime() - prlLogBuffer.getStartTime()), "");
         ImGui.sameLine();
         log |= ImGui.inputInt("##logInput", timeLog);

         if (ImGui.button("<<<##log")) {
            timeLog.set(Math.max(timeLog.get() - 3000, 0));
            log = true;
         }
         ImGui.sameLine();
         if (ImGui.button("<-##log")) {
            long start = prlLogBuffer.getStartTime();
            timeLog.set(Math.max((int)(prlLogBuffer.getPreviousTime(start + timeLog.get()) - start), 0));
            log = true;
         }
         ImGui.sameLine();
         if (ImGui.button("->##log")) {
            long start = prlLogBuffer.getStartTime();
            timeLog.set(Math.min((int)(prlLogBuffer.getNextTime(start + timeLog.get()) - start), (int)(prlLogBuffer.getEndTime() - prlLogBuffer.getStartTime())));
            log = true;
         }
         ImGui.sameLine();
         if (ImGui.button(">>>##log")) {
            timeLog.set(Math.min(timeLog.get() + 3000, (int)(prlLogBuffer.getEndTime() - prlLogBuffer.getStartTime())));
            log = true;
         }

         if (log || firstRun) { //Use bitwise or to make sure both are called
            PlanarRegionsList list = prlLogBuffer.getNearTime(prlLogBuffer.getStartTime() + timeLog.get());
            if (list != null)
            {
               logGraphic.generateMeshesAsync(list);
            }
         }
      } else {
         ImGui.sliderInt("Time:##logSlider", timeLog.getData(), 0, 0, "");
         ImGui.sameLine();
         ImGui.inputInt("##logInput", timeLog);

         ImGui.button("<<<##log");
         ImGui.sameLine();
         ImGui.button("<-##log");
         ImGui.sameLine();
         ImGui.button("->##log");
         ImGui.sameLine();
         ImGui.button(">>>##log");
      }

      ImGui.text("Import log from file:");
      if (ImGui.beginListBox("##filesviewer")) {
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

         ImGui.endListBox();
      }
      ImGui.sameLine();

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
