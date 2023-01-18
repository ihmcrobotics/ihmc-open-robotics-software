package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import org.bytedeco.opencv.global.opencv_calib3d;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Size;
import org.bytedeco.opencv.opencv_features2d.SimpleBlobDetector;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.tools.thread.Activator;

public class WebcamCalibrationPatternDemo
{
   private final Activator nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(),
                                                  "ihmc-open-robotics-software",
                                                  "ihmc-high-level-behaviors/src/test/resources",
                                                  "Webcam Calibration Pattern Demo");
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private RDXOpenCVWebcamReader webcamReader;
   private Mat bgrWebcamCopy;
   private Mat grayscaleImage;
   private SimpleBlobDetector simpleBlobDetector;
   private volatile boolean running = true;
   private final ImInt patternWidth = new ImInt(8);
   private final ImInt patternHeight = new ImInt(11);
   private boolean patternFound = false;
   private Size patternSize;
   private Mat centers;

   public WebcamCalibrationPatternDemo()
   {
      baseUI.getImGuiPanelManager().addPanel(new ImGuiPanel("Calibration Pattern", this::renderImGuiWidgets));
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            webcamReader = new RDXOpenCVWebcamReader(nativesLoadedActivator);
            baseUI.getImGuiPanelManager().addPanel(webcamReader.getStatisticsPanel());
         }

         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  webcamReader.create();
                  baseUI.getImGuiPanelManager().addPanel(webcamReader.getSwapCVPanel().getVideoPanel());
                  baseUI.getPerspectiveManager().reloadPerspective();

                  bgrWebcamCopy = new Mat();
                  grayscaleImage = new Mat();
                  patternSize = new Size(patternWidth.get(), patternHeight.get());
                  centers = new Mat();
                  simpleBlobDetector = SimpleBlobDetector.create();

                  ThreadTools.startAsDaemon(() ->
                  {
                     while (running)
                     {
                        webcamReader.readWebcamImage();
                        webcamReader.getBGRImage().copyTo(bgrWebcamCopy);
                     }
                  }, "CameraRead");
               }

               webcamReader.getSwapCVPanel().getDataSwapReferenceManager().accessOnLowPriorityThread(data ->
               {
                  if (data.getRGBA8Image() != null && bgrWebcamCopy.rows() > 0)
                  {
                     opencv_imgproc.cvtColor(bgrWebcamCopy, grayscaleImage, opencv_imgproc.COLOR_BGR2GRAY);
                     patternFound = opencv_calib3d.findCirclesGrid(grayscaleImage,
                                                                   patternSize,
                                                                   centers,
                                                                   opencv_calib3d.CALIB_CB_SYMMETRIC_GRID,
                                                                   simpleBlobDetector);
                     opencv_calib3d.drawChessboardCorners(data.getRGBA8Mat(), patternSize, centers, patternFound);
                  }
               });

               webcamReader.update();
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
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

   private void renderImGuiWidgets()
   {
      if (ImGuiTools.volatileInputInt(labels.get("Pattern width"), patternWidth))
      {
         patternSize = new Size(patternWidth.get(), patternHeight.get());
      }
      if (ImGuiTools.volatileInputInt(labels.get("Pattern height"), patternHeight))
      {
         patternSize = new Size(patternWidth.get(), patternHeight.get());
      }
      ImGui.text("Pattern found: " + patternFound);
   }

   public static void main(String[] args)
   {
      new WebcamCalibrationPatternDemo();
   }
}
