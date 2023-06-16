package us.ihmc.rdx.perception;

import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXImagePanelTexture;

/**
 * Renders the CozyPose results.
 */
public class RDXWebcamCozyPoseDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI("Webcam CozyPose Demo");
   private RDXOpenCVWebcamReader webcamReader;
   private volatile boolean running = true;
   private final Mat bgrSourceCopy = new Mat();

   public RDXWebcamCozyPoseDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            webcamReader = new RDXOpenCVWebcamReader();
            webcamReader.setMonitorPanelUIThreadPreprocessor(this::monitorPanelUpdateOnUIThread);
            baseUI.getImGuiPanelManager().addPanel(webcamReader.getStatisticsPanel());

            webcamReader.create();
            baseUI.getImGuiPanelManager().addPanel(webcamReader.getSwapCVPanel().getImagePanel());

            ThreadTools.startAsDaemon(() ->
            {
               while (running)
               {
                  webcamReader.readWebcamImage();
                  webcamReader.getBGRImage().copyTo(bgrSourceCopy);
               }
            }, "CameraRead");
         }

         @Override
         public void render()
         {
            webcamReader.updateOnUIThread();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void monitorPanelUpdateOnUIThread(RDXImagePanelTexture texture)
         {
            Mat displayImageToDrawTo = texture.getRGBA8Mat();

            synchronized (this)
            {
               if (bgrSourceCopy.rows() > 0)
               {

                  opencv_imgproc.cvtColor(bgrSourceCopy, displayImageToDrawTo, opencv_imgproc.COLOR_BGR2RGBA, 0);
               }
            }
         }

         @Override
         public void dispose()
         {
            running = false;
            webcamReader.dispose();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXWebcamCozyPoseDemo();
   }
}
