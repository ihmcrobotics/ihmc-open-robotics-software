package us.ihmc.rdx;

import imgui.ImGui;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.perception.RawImage;
import us.ihmc.rdx.perception.RDXRealSenseBagPlayerPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXImageVisualizer;
import us.ihmc.rdx.ui.graphics.RDXRawImagePointCloudVisualizer;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.sensors.realsense.RealSenseBagPlaybackSensor;
import us.ihmc.sensors.realsense.RealSenseConfiguration;
import us.ihmc.sensors.realsense.RealSenseImageSensor;

import java.time.Instant;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public class RDXRealSenseBagPlaybackDemo
{
   private static final String BAG_FILE = "/home/rgriffin/Documents/SeaHunterData/20260224-150047-realsense.bag";
   private static final RealSenseConfiguration CONFIGURATION = RealSenseConfiguration.D455_COLOR_720P_DEPTH_720P_30HZ;

   private final RDXImageVisualizer colorVisualizer;
   private final RDXImageVisualizer depthVisualizer;
   private final List<RDXRawImagePointCloudVisualizer> pointCloudVisualizers = new ArrayList<>();
   private final ImInt ringBufferCapacity = new ImInt(10);

   private final RealSenseBagPlaybackSensor realsenseSensor;
   private final RepeatingTaskThread grabThread;

   private final Queue<RawImage> depthImageQueue = new LinkedList<>();
   private final ImFloat depthImageDelay = new ImFloat(0.0f);
   private final Queue<RawImage> colorImageQueue = new LinkedList<>();
   private final ImFloat colorImageDelay = new ImFloat(0.0f);

   private RDXRealSenseBagPlayerPanel bagPlayerPanel;

   private RDXRealSenseBagPlaybackDemo()
   {
      colorVisualizer = new RDXImageVisualizer("RealSense Color", "Color Panel", false);
      depthVisualizer = new RDXImageVisualizer("RealSense Depth", "Depth Panel", false);

      realsenseSensor = new RealSenseBagPlaybackSensor(CONFIGURATION, BAG_FILE);
      grabThread = new RepeatingTaskThread("RealSenseGrabThread", this::grabThread);

      RDXBaseUI baseUI = new RDXBaseUI();
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimaryScene().addCoordinateFrame(0.3);

            colorVisualizer.create();
            colorVisualizer.setActive(true);
            baseUI.getImGuiPanelManager().addPanel(colorVisualizer.getPanel());

            depthVisualizer.create();
            depthVisualizer.setActive(true);
            baseUI.getImGuiPanelManager().addPanel(depthVisualizer.getPanel());

            updatePointCloudVisualizers(baseUI);

            bagPlayerPanel = new RDXRealSenseBagPlayerPanel(realsenseSensor, CONFIGURATION);
            baseUI.getPrimary3DPanel().addOverlayPanel(bagPlayerPanel.getPanelName(), bagPlayerPanel::render);

            baseUI.getImGuiPanelManager().addPanel("Options", this::renderOptions);

            realsenseSensor.run(true);
            realsenseSensor.play(); // Start playback
            grabThread.startRepeating();
         }

         @Override
         public void render()
         {
            colorVisualizer.update();
            depthVisualizer.update();
            synchronized (pointCloudVisualizers)
            {
               for (RDXRawImagePointCloudVisualizer pointCloudVisualizer : pointCloudVisualizers)
                  pointCloudVisualizer.update();
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderOptions()
         {
            if (ImGui.sliderInt("Ring Buffer Capacity", ringBufferCapacity.getData(), 1, 100))
            {
               updatePointCloudVisualizers(baseUI);
            }

            synchronized (pointCloudVisualizers)
            {
               for (int i = 0; i < pointCloudVisualizers.size(); i++)
               {
                  if (ImGui.collapsingHeader("Point Cloud " + i))
                     pointCloudVisualizers.get(i).renderImGuiWidgets();
               }
            }

            ImGui.separator();

            ImGui.sliderFloat("Depth Delay", depthImageDelay.getData(), 0.0f, 10.0f);
            ImGui.sliderFloat("Color Delay", colorImageDelay.getData(), 0.0f, 10.0f);
         }

         @Override
         public void dispose()
         {
            destroy();
            baseUI.dispose();
            colorVisualizer.destroy();
            depthVisualizer.destroy();
            synchronized (pointCloudVisualizers)
            {
               for (RDXRawImagePointCloudVisualizer pointCloudVisualizer : pointCloudVisualizers)
                  pointCloudVisualizer.destroy();
            }
            colorImageQueue.forEach(RawImage::release);
            depthImageQueue.forEach(RawImage::release);
         }
      });
   }

   private void updatePointCloudVisualizers(RDXBaseUI baseUI)
   {
      synchronized (pointCloudVisualizers)
      {
         while (pointCloudVisualizers.size() < ringBufferCapacity.get())
         {
            RDXRawImagePointCloudVisualizer visualizer = new RDXRawImagePointCloudVisualizer("RealSense Point Cloud " + pointCloudVisualizers.size());
            pointCloudVisualizers.add(visualizer);
            baseUI.getPrimaryScene().addRenderableProvider(visualizer, visualizer);
         }
         while (pointCloudVisualizers.size() > ringBufferCapacity.get())
         {
            RDXRawImagePointCloudVisualizer visualizer = pointCloudVisualizers.remove(pointCloudVisualizers.size() - 1);
            baseUI.getPrimaryScene().removeRenderable(visualizer);
            visualizer.destroy();
         }
      }
   }

   private void grabThread() throws InterruptedException
   {
      realsenseSensor.waitForGrab();
      RawImage colorImage = realsenseSensor.getImage(RealSenseImageSensor.COLOR_IMAGE_KEY);
      RawImage depthImage = realsenseSensor.getImage(RealSenseImageSensor.DEPTH_IMAGE_KEY);

      if (colorImage != null)
         colorVisualizer.setImage(colorImage);
      if (depthImage != null)
         depthVisualizer.setImage(depthImage);

      if (colorImage != null)
         colorImageQueue.add(colorImage);
      if (depthImage != null)
         depthImageQueue.add(depthImage);

      Instant now = Instant.now();

      synchronized (pointCloudVisualizers)
      {
         RawImage oldestColorImage = colorImageQueue.peek();
         while (oldestColorImage != null && TimeTools.secondsBetween(oldestColorImage.getAcquisitionTime(), now) > colorImageDelay.get())
         {
            if (!pointCloudVisualizers.isEmpty())
            {
               int visualizerIndex = (int) (oldestColorImage.getSequenceNumber() % pointCloudVisualizers.size());
               pointCloudVisualizers.get(visualizerIndex).setColorImage(oldestColorImage);
            }
            colorImageQueue.remove().release();
            oldestColorImage = colorImageQueue.peek();
         }

         RawImage oldestDepthImage = depthImageQueue.peek();
         while (oldestDepthImage != null && TimeTools.secondsBetween(oldestDepthImage.getAcquisitionTime(), now) > depthImageDelay.get())
         {
            if (!pointCloudVisualizers.isEmpty())
            {
               int visualizerIndex = (int) (oldestDepthImage.getSequenceNumber() % pointCloudVisualizers.size());
               pointCloudVisualizers.get(visualizerIndex).setDepthImage(oldestDepthImage);
            }
            depthImageQueue.remove().release();
            oldestDepthImage = depthImageQueue.peek();
         }
      }
   }

   private void destroy()
   {
      grabThread.blockingKill();
      realsenseSensor.close();
   }

   public static void main(String[] args)
   {
      new RDXRealSenseBagPlaybackDemo();
   }
}
