package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.sensors.realsense.RealSenseBagPlaybackSensor;
import us.ihmc.sensors.realsense.RealSenseConfiguration;

public class RDXRealSenseBagPlayerPanel
{
   private static final String PANEL_NAME = "RealSense Bag Player";
   private static final long NANOSECONDS_PER_SECOND = 1_000_000_000L;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final RealSenseBagPlaybackSensor bagPlaybackSensor;
   private final RealSenseConfiguration configuration;

   private final ImInt requestedFrameNumber = new ImInt();
   private boolean holdingOnToTheSlider;
   private boolean paused;

   private final Throttler requestThrottler = new Throttler().setFrequency(10.0);

   public RDXRealSenseBagPlayerPanel(RealSenseBagPlaybackSensor bagPlaybackSensor, RealSenseConfiguration configuration)
   {
      this.bagPlaybackSensor = bagPlaybackSensor;
      this.configuration = configuration;
   }

   public void render()
   {
      ImGuiTools.textBold("Current Bag:");
      ImGui.sameLine();
      ImGui.textWrapped(bagPlaybackSensor.getBagFileName());

      long durationNanoseconds = bagPlaybackSensor.getDuration();
      long currentPositionNanoseconds = bagPlaybackSensor.getCurrentPosition();

      // Calculate frame numbers based on FPS
      int fps = configuration.getDepthFPS();
      double durationSeconds = durationNanoseconds / (double) NANOSECONDS_PER_SECOND;
      double currentPositionSeconds = currentPositionNanoseconds / (double) NANOSECONDS_PER_SECOND;

      int totalFrames = (int) Math.round(durationSeconds * fps);
      int currentFrame = (int) Math.round(currentPositionSeconds * fps);

      if (!holdingOnToTheSlider)
         requestedFrameNumber.set(currentFrame);

      if (ImGuiTools.sliderInt(labels.get("##frame"), requestedFrameNumber, 0, Math.max(totalFrames, 0)))
      {
         holdingOnToTheSlider = true;

         if (requestThrottler.run())
         {
            seekToRequestedFrame();
            bagPlaybackSensor.grabAndNotify();
         }
      }

      // Called once you let go of the slider
      if (ImGui.isItemDeactivatedAfterEdit())
      {
         holdingOnToTheSlider = false;
         seekToRequestedFrame();
         bagPlaybackSensor.grabAndNotify();
      }

      ImGui.sameLine();

      if (ImGui.button(labels.get("<")))
      {
         requestedFrameNumber.set(Math.max(requestedFrameNumber.get() - 1, 0));
         seekToRequestedFrame();
         bagPlaybackSensor.grabAndNotify();
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get(">")))
      {
         requestedFrameNumber.set(Math.min(requestedFrameNumber.get() + 1, totalFrames));
         seekToRequestedFrame();
         bagPlaybackSensor.grabAndNotify();
      }
      ImGui.sameLine();

      if (ImGui.button(labels.get(paused ? "Play" : "Pause")))
      {
         if (paused)
            bagPlaybackSensor.play();
         else
            bagPlaybackSensor.pause();
         paused = !paused;
      }

      ImGui.text(String.format("(%02d:%02d / %02d:%02d)",
                               (int) currentPositionSeconds / 60,
                               (int) currentPositionSeconds % 60,
                               (int) durationSeconds / 60,
                               (int) durationSeconds % 60));
   }

   private void seekToRequestedFrame()
   {
      int fps = configuration.getDepthFPS();
      double frameTimeSeconds = requestedFrameNumber.get() / (double) fps;
      // Adding a small epsilon to ensure we land inside the intended frame and not right on the boundary or slightly before it
      long positionNanoseconds = (long) (frameTimeSeconds * NANOSECONDS_PER_SECOND) + 1000;
      bagPlaybackSensor.setCurrentPosition(positionNanoseconds);
   }

   public String getPanelName()
   {
      return PANEL_NAME;
   }
}
