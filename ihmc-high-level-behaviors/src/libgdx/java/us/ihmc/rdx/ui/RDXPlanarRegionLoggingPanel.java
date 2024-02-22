package us.ihmc.rdx.ui;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import us.ihmc.perception.logging.PlanarRegionsReplayBuffer;
import us.ihmc.perception.logging.PlanarRegionsListLogger;
import us.ihmc.commons.Conversions;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
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

public class RDXPlanarRegionLoggingPanel extends RDXPanel implements RenderableProvider
{
   private static final int REALTIME_BUFFER_LEN = 3000;

   public static final String WINDOW_NAME = "Planar Region Logging";

   private final PlanarRegionsListLogger logger;
   private PlanarRegionsReplayBuffer realtimeBuffer;
   private PlanarRegionsReplayBuffer logBuffer;

   private final RDXPlanarRegionsGraphic realtimeGraphic;
   private final RDXPlanarRegionsGraphic logGraphic;

   private final ImBoolean logPlanarRegions = new ImBoolean(false);
   private final ImBoolean showRealtime = new ImBoolean(false);
   private final ImBoolean enableRealtime = new ImBoolean(false);
   private final ImBoolean showLog = new ImBoolean(false);
   private final ImFloat timeRealtimeFloat = new ImFloat(0);
   private long timeRealtime = 0;
   private final ImFloat timeLogFloat = new ImFloat(0);
   private long timeLog = 0;

   private boolean firstRun = true;
   private boolean mustUpdateFiles = false;

   private long realtimeStartTime = Long.MAX_VALUE;

   private final ArrayList<File> files = new ArrayList<>();

   public RDXPlanarRegionLoggingPanel()
   {
      super(WINDOW_NAME);
      setRenderMethod(this::renderImGuiWidgets);
      logger = new PlanarRegionsListLogger(this.getClass().getSimpleName(), Integer.MAX_VALUE);
      realtimeBuffer = new PlanarRegionsReplayBuffer(REALTIME_BUFFER_LEN);

      realtimeGraphic = new RDXPlanarRegionsGraphic();
      logGraphic = new RDXPlanarRegionsGraphic();
   }

   public void create()
   {
      updateAvailableLogFiles();
   }

   public void update(long time, PlanarRegionsList planarRegionsList)
   {
      if (logPlanarRegions.get())
      {
         logger.update(time, planarRegionsList);
         if (mustUpdateFiles)
         {
            mustUpdateFiles = false;
            updateAvailableLogFiles();
         }
      }

      if (enableRealtime.get())
         realtimeBuffer.putAndTick(time, planarRegionsList);
   }

   private void updateAvailableLogFiles()
   {
      File logDirectory = Paths.get(PlanarRegionsListLogger.getLogDirectory()).toFile();
      files.clear();

      try
      {
         files.addAll(Arrays.asList(Objects.requireNonNull(logDirectory.listFiles(pathname -> pathname.toString().toLowerCase().endsWith(".prllog")))));
      }
      catch (NullPointerException nullPointerException)
      {
         LogTools.error("Error loading planar region log files (.prllog) in {}", logDirectory);
      }

      files.sort((o1, o2) -> //Sorts most recent first
                 {
                    BasicFileAttributes left;
                    BasicFileAttributes right;
                    try
                    {
                       left = Files.readAttributes(o1.toPath(), BasicFileAttributes.class);
                       right = Files.readAttributes(o2.toPath(), BasicFileAttributes.class);
                    }
                    catch (IOException ioException)
                    {
                       LogTools.error("Could not sort files");
                       LogTools.error(ioException.getMessage());
                       ioException.printStackTrace();

                       return 0;
                    }

                    return right.creationTime().compareTo(left.creationTime());
                 });
   }

   private float getFloatSecondsFromNanos(long nanos, long startTime)
   {
      return (float) Conversions.nanosecondsToSeconds(nanos - startTime);
   }

   private long getNanosFromFloatSeconds(float seconds, long startTime)
   {
      return Conversions.secondsToNanoseconds(seconds) + startTime;
   }

   public void renderImGuiWidgets()
   {
      if (realtimeBuffer.getFirstEverTime() < realtimeStartTime)
         realtimeStartTime = realtimeBuffer.getFirstEverTime();

      // Realtime buffer
      ImGui.text("Current session's regions buffer:");
      ImGui.checkbox("Show##1", showRealtime);
      ImGui.sameLine();
      if (ImGui.button("Clear"))
      {
         realtimeBuffer = new PlanarRegionsReplayBuffer(REALTIME_BUFFER_LEN);
      }
      ImGui.sameLine();
      ImGui.checkbox("Enable", enableRealtime);
      boolean realtime = ImGui.sliderFloat("Time:##realtimeSlider",
                                           timeRealtimeFloat.getData(),
                                           realtimeBuffer.getStartTime() == -1 ? 0 : getFloatSecondsFromNanos(realtimeBuffer.getStartTime(), realtimeStartTime),
                                           realtimeBuffer.getEndTime() == -1 ? 0 : getFloatSecondsFromNanos(realtimeBuffer.getEndTime(), realtimeStartTime),
                                           "");
      ImGui.sameLine();
      realtime |= ImGui.inputFloat("##realtimeInput", timeRealtimeFloat);

      if (realtime)
      {
         timeRealtime = getNanosFromFloatSeconds(timeRealtimeFloat.get(), realtimeStartTime);
      }

      if (ImGui.button("<<<##realtime"))
      {
         timeRealtime = Math.max(timeRealtime - 3000000000L, realtimeStartTime);
         realtime = true;
         timeRealtimeFloat.set(getFloatSecondsFromNanos(timeRealtime, realtimeStartTime));
      }
      ImGui.sameLine();
      if (ImGui.button("<-##realtime"))
      {
         long start = realtimeBuffer.getStartTime();
         timeRealtime = Math.max(realtimeBuffer.getPreviousTime(timeRealtime), realtimeStartTime);
         realtime = true;
         timeRealtimeFloat.set(getFloatSecondsFromNanos(timeRealtime, realtimeStartTime));
      }
      ImGui.sameLine();
      if (ImGui.button("->##realtime"))
      {
         timeRealtime = Math.min(realtimeBuffer.getNextTime(timeRealtime), realtimeBuffer.getEndTime());
         realtime = true;
         timeRealtimeFloat.set(getFloatSecondsFromNanos(timeRealtime, realtimeStartTime));
      }
      ImGui.sameLine();
      if (ImGui.button(">>>##realtime"))
      {
         timeRealtime = Math.min(timeRealtime + 3000000000L, realtimeBuffer.getEndTime());
         realtime = true;
         timeRealtimeFloat.set(getFloatSecondsFromNanos(timeRealtime, realtimeStartTime));
      }

      if (realtime || firstRun)
      {
         PlanarRegionsList regions = (PlanarRegionsList) realtimeBuffer.getNearTime(timeRealtime);
         if (regions != null)
         {
            realtimeGraphic.generateMeshesAsync(regions);
         }
      }

      ImGui.separator();

      // Log buffer
      if (ImGui.checkbox("Log Planar Regions", logPlanarRegions))
      {
         if (logPlanarRegions.get())
         {
            logger.start();
            mustUpdateFiles = true;
         }
      }

      ImGui.checkbox("Show regions from log##2", showLog);
      if (logBuffer != null)
      {
         boolean log = ImGui.sliderFloat("Time:##logSlider",
                                         timeLogFloat.getData(),
                                         0,
                                         logBuffer.getFirstEverTime() == Long.MAX_VALUE ?
                                               0 :
                                               getFloatSecondsFromNanos(logBuffer.getEndTime(), logBuffer.getFirstEverTime()),
                                         "");
         ImGui.sameLine();
         log |= ImGui.inputFloat("##logInput", timeLogFloat);

         if (log)
            timeLog = getNanosFromFloatSeconds(timeLogFloat.get(), logBuffer.getFirstEverTime());

         if (ImGui.button("<<<##log"))
         {
            timeLog = Math.max(timeLog - 3000000000L, logBuffer.getStartTime());
            log = true;
            timeLogFloat.set(getFloatSecondsFromNanos(timeLog, logBuffer.getFirstEverTime()));
         }
         ImGui.sameLine();
         if (ImGui.button("<-##log"))
         {
            timeLog = Math.max(logBuffer.getPreviousTime(timeLog), logBuffer.getStartTime());
            log = true;
            timeLogFloat.set(getFloatSecondsFromNanos(timeLog, logBuffer.getFirstEverTime()));
         }
         ImGui.sameLine();
         if (ImGui.button("->##log"))
         {
            timeLog = Math.min(logBuffer.getNextTime(timeLog), logBuffer.getEndTime());
            log = true;
            timeLogFloat.set(getFloatSecondsFromNanos(timeLog, logBuffer.getFirstEverTime()));
         }
         ImGui.sameLine();
         if (ImGui.button(">>>##log"))
         {
            timeLog = Math.min(timeLog + 3000000000L, logBuffer.getEndTime());
            log = true;
            timeLogFloat.set(getFloatSecondsFromNanos(timeLog, logBuffer.getStartTime()));
         }

         if (log || firstRun)
         {
            PlanarRegionsList list = (PlanarRegionsList) logBuffer.getNearTime(timeLog);
            if (list != null)
            {
               logGraphic.generateMeshesAsync(list);
            }
         }
      }
      else
      {
         ImGui.sliderFloat("Time:##logSlider", timeLogFloat.getData(), 0, 0, "");
         ImGui.sameLine();
         ImGui.inputFloat("##logInput", timeLogFloat);

         ImGui.button("<<<##log");
         ImGui.sameLine();
         ImGui.button("<-##log");
         ImGui.sameLine();
         ImGui.button("->##log");
         ImGui.sameLine();
         ImGui.button(">>>##log");
      }

      ImGui.text("Import log from file:");
      if (ImGui.beginListBox("##filesviewer"))
      {
         Iterator<File> iterator = files.iterator();
         while (iterator.hasNext())
         { //Use iterator to prevent rare ConcurrentModificationError
            File file = iterator.next();
            if (ImGui.selectable(file.getName()))
            {
               try
               {
                  logBuffer = new PlanarRegionsReplayBuffer(file, PlanarRegionsList.class);
               }
               catch (IOException ioException)
               {
                  LogTools.error("Could not load file " + file.getName());
                  LogTools.error(ioException.getMessage());
                  ioException.printStackTrace();
               }
            }
         }

         ImGui.endListBox();
      }
      ImGui.sameLine();

      if (ImGui.button("Refresh logs"))
      {
         updateAvailableLogFiles();
      }

      firstRun = false;
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (showRealtime.get())
      {
         realtimeGraphic.update();
         realtimeGraphic.getRenderables(renderables, pool);
      }

      if (showLog.get())
      {
         logGraphic.update();
         logGraphic.getRenderables(renderables, pool);
      }
   }

   public void destroy()
   {
      realtimeGraphic.destroy();
      logGraphic.destroy();
   }

   public String getWindowName()
   {
      return WINDOW_NAME;
   }
}
