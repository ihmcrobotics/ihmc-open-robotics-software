package us.ihmc.gdx.ui.graphics.live;

import org.bytedeco.opencv.global.opencv_videoio;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_videoio.VideoCapture;
import org.bytedeco.opencv.opencv_videoio.VideoWriter;
import us.ihmc.log.LogTools;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class GDXLogVideoVisualizer extends GDXOpenCVVideoVisualizer
{
   private final String filename;
   private Mat image;
   private VideoCapture cap;
   private final ScheduledExecutorService executorService = Executors.newScheduledThreadPool(1);
   public GDXLogVideoVisualizer(String file)
   {
      super(file, file, false);
      this.filename = file;

      this.cap = new VideoCapture(file);

      this.cap.set(opencv_videoio.CAP_PROP_FOURCC, VideoWriter.fourcc((byte) 'C', (byte) 'Y', (byte) 'U', (byte) 'V'));

      executorService.scheduleAtFixedRate(this::loadNextFrame, 0L, 10L, TimeUnit.MILLISECONDS);
   }

   public void loadNextFrame()
   {
      boolean success = cap.read(this.image);

      if(success)
      {
         LogTools.info("Success");
      }
      else
      {
         LogTools.info("Failure");
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
//      super.renderImGuiWidgets();
//      ImGui.text(this.filename);
//      if (getHasReceivedOne())
//         getFrequencyPlot().renderImGuiWidgets();
   }
}
