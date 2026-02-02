package us.ihmc.rdx;

import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXRawImagePointCloudVisualizer;
import us.ihmc.sensors.realsense.RealSenseBagPlaybackSensor;
import us.ihmc.sensors.realsense.RealSenseConfiguration;
import us.ihmc.sensors.realsense.RealSenseImageSensor;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.sensors.zed.ZEDSVOPlaybackSensor;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import static us.ihmc.zed.global.zed.SL_DEPTH_MODE_PERFORMANCE;

public class RDXRealSenseBagZEDSVOSyncedDemo
{
   private static final String REALSENSE_BAG_PATH = "/opt/ihmc/LogData/UserFolders/DexFolder/FrameGrabber9000/FrameGrabber9000Demo/20260123_131617.bag";
   private static final String ZED_SVO_PATH = "/opt/ihmc/LogData/UserFolders/DexFolder/FrameGrabber9000/FrameGrabber9000Demo/ZED_Recording_58123737_20260123_131618.svo";
   private static final RealSenseConfiguration REALSENSE_CONFIG = RealSenseConfiguration.D455_COLOR_720P_DEPTH_720P_30HZ;
   private static final long NANOSECONDS_PER_SECOND = 1_000_000_000L;

   private final RealSenseBagPlaybackSensor realSenseSensor;
   private final ZEDSVOPlaybackSensor zedSensor;
   private RDXRawImagePointCloudVisualizer realSenseVisualizer;
   private RDXRawImagePointCloudVisualizer zedVisualizer;
   private final RepeatingTaskThread realSenseGrabThread;
   private final RepeatingTaskThread zedGrabThread;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final String syncFilePath;
   private final ImBoolean realSensePaused = new ImBoolean(true);
   private final ImBoolean zedPaused = new ImBoolean(true);
   private final ImInt realSenseFrameNumber = new ImInt(0);
   private final ImInt zedFrameNumber = new ImInt(0);
   private final ImFloat offsetSeconds = new ImFloat(0.0f);
   private boolean holdingOffsetScrubber = false;
   private int realSenseTotalFrames;
   private int zedTotalFrames;
   private int skipZedUpdateFrames = 0;
   private final Throttler requestThrottler = new Throttler().setFrequency(10.0);

   public RDXRealSenseBagZEDSVOSyncedDemo()
   {
      realSenseSensor = new RealSenseBagPlaybackSensor(REALSENSE_CONFIG, REALSENSE_BAG_PATH);
      zedSensor = new ZEDSVOPlaybackSensor(0, ZEDModelData.ZED_2, SL_DEPTH_MODE_PERFORMANCE, ZED_SVO_PATH);

      realSenseGrabThread = new RepeatingTaskThread("RealSenseGrabThread", this::grabRealSenseFrame);
      zedGrabThread = new RepeatingTaskThread("ZEDGrabThread", this::grabZEDFrame);

      Path bagPath = Paths.get(REALSENSE_BAG_PATH);
      this.syncFilePath = bagPath.getParent().resolve("sync_offset.txt").toString();

      realSenseTotalFrames = -1;
      zedTotalFrames = -1;

      loadOffset();

      RDXBaseUI baseUI = new RDXBaseUI("RealSense Bag + ZED SVO Synced Demo");
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimaryScene().addCoordinateFrame(0.3);

            realSenseVisualizer = new RDXRawImagePointCloudVisualizer("RealSense Point Cloud");
            realSenseVisualizer.setActive(true);

            zedVisualizer = new RDXRawImagePointCloudVisualizer("ZED Point Cloud");
            zedVisualizer.setActive(true);

            baseUI.getPrimaryScene().addRenderableProvider((renderables, pool, sceneLevels) ->
            {
               if (realSenseVisualizer.isActive())
                  realSenseVisualizer.getRenderables(renderables, pool, sceneLevels);
               if (zedVisualizer.isActive())
                  zedVisualizer.getRenderables(renderables, pool, sceneLevels);
            });

            baseUI.getPrimary3DPanel().addOverlayPanel("Synced Playback", RDXRealSenseBagZEDSVOSyncedDemo.this::renderSyncedPlaybackPanel);

            baseUI.getImGuiPanelManager().addPanel("RealSense Options", realSenseVisualizer::renderImGuiWidgets);
            baseUI.getImGuiPanelManager().addPanel("ZED Options", zedVisualizer::renderImGuiWidgets);

            realSenseSensor.run(true);
            zedSensor.run(true);

            realSenseGrabThread.startRepeating();
            zedGrabThread.startRepeating();
         }

         @Override
         public void render()
         {
            updateZedPositionBasedOnOffset();

            if (realSenseVisualizer.isActive())
               realSenseVisualizer.update();
            if (zedVisualizer.isActive())
               zedVisualizer.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            destroy();
            realSenseVisualizer.destroy();
            zedVisualizer.destroy();
            baseUI.dispose();
         }
      });
   }

   private void grabRealSenseFrame() throws InterruptedException
   {
      realSenseSensor.waitForGrab();
      RawImage colorImage = realSenseSensor.getImage(RealSenseImageSensor.COLOR_IMAGE_KEY);
      RawImage depthImage = realSenseSensor.getImage(RealSenseImageSensor.DEPTH_IMAGE_KEY);

      if (colorImage != null)
      {
         realSenseVisualizer.setColorImage(colorImage);
         colorImage.release();
      }
      if (depthImage != null)
      {
         realSenseVisualizer.setDepthImage(depthImage);
         depthImage.release();
      }
   }

   private void grabZEDFrame() throws InterruptedException
   {
      zedSensor.waitForGrab();
      RawImage colorImage = zedSensor.getImage(ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
      RawImage depthImage = zedSensor.getImage(ZEDImageSensor.DEPTH_IMAGE_KEY);

      if (colorImage != null)
      {
         zedVisualizer.setColorImage(colorImage);
         colorImage.release();
      }
      if (depthImage != null)
      {
         zedVisualizer.setDepthImage(depthImage);
         depthImage.release();
      }
   }

   private void destroy()
   {
      realSenseGrabThread.blockingKill();
      zedGrabThread.blockingKill();
      realSenseSensor.close();
      zedSensor.close();
   }

   private void updateZedPositionBasedOnOffset()
   {
      if (realSenseTotalFrames == -1 || zedTotalFrames == -1 || holdingOffsetScrubber)
         return;

      long realSensePositionNanos = realSenseSensor.getCurrentPosition();
      double realSenseTimeSeconds = realSensePositionNanos / (double) NANOSECONDS_PER_SECOND;
      double zedTimeSeconds = realSenseTimeSeconds + offsetSeconds.get();

      long realSenseDurationNanos = realSenseSensor.getDuration();
      double realSenseDurationSeconds = realSenseDurationNanos / (double) NANOSECONDS_PER_SECOND;
      double estimatedZedFPS = zedTotalFrames / realSenseDurationSeconds;

      int targetZedFrame = (int) (zedTimeSeconds * estimatedZedFPS);
      targetZedFrame = Math.max(0, Math.min(targetZedFrame, zedTotalFrames - 1));

      int currentZedFrame = zedSensor.getCurrentPosition();

      if (Math.abs(targetZedFrame - currentZedFrame) > 10)
      {
         zedSensor.setCurrentPosition(targetZedFrame);
      }
   }

   private void renderSyncedPlaybackPanel()
   {
      if (realSenseTotalFrames == -1 && realSenseSensor.isSensorRunning())
      {
         long durationNanoseconds = realSenseSensor.getDuration();
         double durationSeconds = durationNanoseconds / (double) NANOSECONDS_PER_SECOND;
         int fps = REALSENSE_CONFIG.getDepthFPS();
         realSenseTotalFrames = (int) (durationSeconds * fps);
      }

      if (zedTotalFrames == -1 && zedSensor.isSensorRunning())
      {
         zedTotalFrames = zedSensor.getLength();
      }

      ImGuiTools.textBold("Synced Playback Control");
      ImGui.separator();

      ImGui.text("RealSense Bag");

      if (realSenseTotalFrames == -1)
      {
         ImGui.text("Waiting for RealSense sensor to initialize...");
      }
      else
      {
         long currentPositionNanoseconds = realSenseSensor.getCurrentPosition();
         double currentPositionSeconds = currentPositionNanoseconds / (double) NANOSECONDS_PER_SECOND;
         int currentFrame = (int) (currentPositionSeconds * REALSENSE_CONFIG.getDepthFPS());
         realSenseFrameNumber.set(currentFrame);

         ImGui.pushItemWidth(ImGui.getWindowWidth() - 150);
         ImGui.beginDisabled();
         ImGuiTools.sliderInt(labels.get("##realSenseScrubber"), realSenseFrameNumber, 0, Math.max(realSenseTotalFrames - 1, 0));
         ImGui.endDisabled();
         ImGui.popItemWidth();
         ImGui.sameLine();
         ImGui.text(String.format("%d/%d", realSenseFrameNumber.get(), realSenseTotalFrames));
      }

      ImGui.spacing();

      ImGui.text("ZED SVO");

      if (zedTotalFrames == -1)
      {
         ImGui.text("Waiting for ZED sensor to initialize...");
      }
      else
      {
         int currentZedPosition = zedSensor.getCurrentPosition();

         if (!holdingOffsetScrubber && skipZedUpdateFrames == 0)
         {
            zedFrameNumber.set(currentZedPosition);
         }

         if (skipZedUpdateFrames > 0)
            skipZedUpdateFrames--;

         ImGui.pushItemWidth(ImGui.getWindowWidth() - 150);
         ImGui.beginDisabled();
         ImGuiTools.sliderInt(labels.get("##zedScrubber"), zedFrameNumber, 0, Math.max(zedTotalFrames - 1, 0));
         ImGui.endDisabled();
         ImGui.popItemWidth();
         ImGui.sameLine();
         ImGui.text(String.format("%d/%d", zedFrameNumber.get(), zedTotalFrames));
      }

      ImGui.spacing();
      ImGui.separator();

      ImGui.text("ZED Time Offset");
      ImGui.pushItemWidth(ImGui.getWindowWidth() - 100);
      float maxOffsetSeconds = 30.0f;
      if (ImGui.sliderFloat(labels.get("##offsetScrubber"), offsetSeconds.getData(), -maxOffsetSeconds, maxOffsetSeconds, "%.2f s"))
      {
         holdingOffsetScrubber = true;
         if (requestThrottler.run())
         {
            applyOffset();
         }
      }

      if (ImGui.isItemDeactivatedAfterEdit())
      {
         holdingOffsetScrubber = false;
         applyOffset();
      }

      ImGui.popItemWidth();

      ImGui.spacing();

      if (ImGui.button(labels.get("Save Offset")))
      {
         saveOffset();
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Load Offset")))
      {
         loadOffset();
      }

      ImGui.spacing();
      ImGui.separator();

      if (ImGui.button(labels.get("Play Both")))
      {
         realSenseSensor.play();
         zedSensor.play();
         realSensePaused.set(false);
         zedPaused.set(false);
      }

      ImGui.sameLine();
      if (ImGui.button(labels.get("Pause Both")))
      {
         realSenseSensor.pause();
         zedSensor.pause();
         realSensePaused.set(true);
         zedPaused.set(true);
      }

      ImGui.sameLine();
      if (ImGui.button(labels.get("Reset Both")))
      {
         realSenseFrameNumber.set(0);
         seekRealSenseToFrame();

         double zedTimeSeconds = 0.0 + offsetSeconds.get();
         long realSenseDurationNanos = realSenseSensor.getDuration();
         double realSenseDurationSeconds = realSenseDurationNanos / (double) NANOSECONDS_PER_SECOND;
         double estimatedZedFPS = zedTotalFrames / realSenseDurationSeconds;
         int zedFrame = (int) (zedTimeSeconds * estimatedZedFPS);
         zedFrame = Math.max(0, Math.min(zedFrame, zedTotalFrames - 1));
         zedFrameNumber.set(zedFrame);
         seekZedToFrame();

         realSenseSensor.pause();
         zedSensor.pause();
         realSensePaused.set(true);
         zedPaused.set(true);
      }

      ImGui.spacing();
      ImGui.text(String.format("Sync file: %s", new File(syncFilePath).exists() ? "Found" : "Not found"));

      ImGui.spacing();
      ImGui.separator();

      ImGui.text("Point Cloud Visualizers");
      boolean rsActive = realSenseVisualizer.isActive();
      if (ImGui.checkbox(labels.get("RealSense Point Cloud"), rsActive))
      {
         realSenseVisualizer.setActive(!rsActive);
      }

      boolean zedActive = zedVisualizer.isActive();
      if (ImGui.checkbox(labels.get("ZED Point Cloud"), zedActive))
      {
         zedVisualizer.setActive(!zedActive);
      }
   }

   private void seekRealSenseToFrame()
   {
      int fps = REALSENSE_CONFIG.getDepthFPS();
      double frameTimeSeconds = realSenseFrameNumber.get() / (double) fps;
      long positionNanoseconds = (long) (frameTimeSeconds * NANOSECONDS_PER_SECOND);
      realSenseSensor.setCurrentPosition(positionNanoseconds);
   }

   private void seekZedToFrame()
   {
      zedSensor.setCurrentPosition(zedFrameNumber.get());
   }

   private void applyOffset()
   {
      long realSensePositionNanos = realSenseSensor.getCurrentPosition();
      double realSenseTimeSeconds = realSensePositionNanos / (double) NANOSECONDS_PER_SECOND;
      double zedTimeSeconds = realSenseTimeSeconds + offsetSeconds.get();

      long realSenseDurationNanos = realSenseSensor.getDuration();
      double realSenseDurationSeconds = realSenseDurationNanos / (double) NANOSECONDS_PER_SECOND;
      double estimatedZedFPS = zedTotalFrames / realSenseDurationSeconds;

      int zedFrame = (int) (zedTimeSeconds * estimatedZedFPS);
      zedFrame = Math.max(0, Math.min(zedFrame, zedTotalFrames - 1));

      zedFrameNumber.set(zedFrame);
      seekZedToFrame();

      skipZedUpdateFrames = 10;
   }

   private void saveOffset()
   {
      try (FileWriter writer = new FileWriter(syncFilePath))
      {
         writer.write(String.valueOf(offsetSeconds.get()));
         LogTools.info("Saved offset {} seconds to {}", offsetSeconds.get(), syncFilePath);
      }
      catch (IOException e)
      {
         LogTools.error("Failed to save offset: {}", e.getMessage());
      }
   }

   private void loadOffset()
   {
      File syncFile = new File(syncFilePath);
      if (syncFile.exists())
      {
         try
         {
            String content = Files.readString(Paths.get(syncFilePath)).trim();
            float loadedOffset = Float.parseFloat(content);
            offsetSeconds.set(loadedOffset);
            LogTools.info("Loaded offset {} seconds from {}", loadedOffset, syncFilePath);
         }
         catch (IOException | NumberFormatException e)
         {
            LogTools.error("Failed to load offset: {}", e.getMessage());
         }
      }
   }

   public static void main(String[] args)
   {
      new RDXRealSenseBagZEDSVOSyncedDemo();
   }
}
