package us.ihmc.rdx.perception;

import imgui.ImGui;
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
import us.ihmc.rdx.ui.graphics.ImGuiOpenCVSwapVideoPanelData;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
import us.ihmc.tools.thread.ZeroCopySwapReference;

import java.util.function.Consumer;

public class WebcamCalibrationPatternDemo
{
   private final Activator nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(),
                                                  "ihmc-open-robotics-software",
                                                  "ihmc-high-level-behaviors/src/test/resources",
                                                  "Webcam Calibration Pattern Demo");
   private final ResettableExceptionHandlingExecutorService patternDetectionThreadQueue
         = MissingThreadTools.newSingleThreadExecutor("PatternDetection", true, 1);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private RDXOpenCVWebcamReader webcamReader;
   private Mat bgrWebcamCopy;
   private Mat grayscaleImage;
   private SimpleBlobDetector simpleBlobDetector;
   private volatile boolean running = true;
   private final ImInt patternWidth = new ImInt(4);
   private final ImInt patternHeight = new ImInt(3);
   private boolean patternFound = false;
   private Size patternSize;
   private ZeroCopySwapReference<Mat> cornersOrCenters;
   private final Consumer<ImGuiOpenCVSwapVideoPanelData> accessOnHighPriorityThread = this::accessOnHighPriorityThread;
   private final Consumer<Mat> accessOnLowPriorityThread = this::accessOnLowPriorityThread;
   private final Runnable doPatternDetection = this::doPatternDetection;
   private enum Pattern { CHESSBOARD, CIRCLES }
   private Pattern pattern = Pattern.CHESSBOARD;
   private final Object avoidCopiedImageTearing = new Object();

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
                  cornersOrCenters = new ZeroCopySwapReference<>(Mat::new);
                  simpleBlobDetector = SimpleBlobDetector.create();

                  ThreadTools.startAsDaemon(() ->
                  {
                     while (running)
                     {
                        webcamReader.readWebcamImage();
                        synchronized (avoidCopiedImageTearing)
                        {
                           webcamReader.getBGRImage().copyTo(bgrWebcamCopy);
                        }
                     }
                  }, "CameraRead");
               }

               if (bgrWebcamCopy.rows() > 0)
               {
                  patternDetectionThreadQueue.clearQueueAndExecute(doPatternDetection);
               }

               webcamReader.getSwapCVPanel().getDataSwapReferenceManager().accessOnHighPriorityThread(accessOnHighPriorityThread);
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

   private void doPatternDetection()
   {
      synchronized (avoidCopiedImageTearing)
      {
         opencv_imgproc.cvtColor(bgrWebcamCopy, grayscaleImage, opencv_imgproc.COLOR_BGR2GRAY);
      }

      cornersOrCenters.accessOnLowPriorityThread(accessOnLowPriorityThread);
   }

   private void accessOnLowPriorityThread(Mat cornersOrCenters)
   {
      if (pattern == Pattern.CHESSBOARD)
      {
         patternFound = opencv_calib3d.findChessboardCorners(grayscaleImage,
                                                             patternSize,
                                                             cornersOrCenters,
                                                             opencv_calib3d.CALIB_CB_ADAPTIVE_THRESH | opencv_calib3d.CALIB_CB_NORMALIZE_IMAGE);
      }
      else
      {
         patternFound = opencv_calib3d.findCirclesGrid(grayscaleImage,
                                                       patternSize,
                                                       cornersOrCenters,
                                                       opencv_calib3d.CALIB_CB_SYMMETRIC_GRID,
                                                       simpleBlobDetector);
      }
   }

   private void accessOnHighPriorityThread(ImGuiOpenCVSwapVideoPanelData data)
   {
      if (webcamReader.getImageWasRead())
      {
         cornersOrCenters.accessOnHighPriorityThread(cornersOrCenters ->
         {
            opencv_calib3d.drawChessboardCorners(data.getRGBA8Mat(), patternSize, cornersOrCenters, patternFound);
         });
      }

      webcamReader.accessOnHighPriorityThread(data);
   }

   private void renderImGuiWidgets()
   {
      ImGui.text("Pattern:");
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Chessboard"), pattern == Pattern.CHESSBOARD))
      {
         pattern = Pattern.CHESSBOARD;
      }
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Circles"), pattern == Pattern.CIRCLES))
      {
         pattern = Pattern.CIRCLES;
      }
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
