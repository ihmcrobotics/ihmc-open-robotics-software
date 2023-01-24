package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.type.ImInt;
import org.bytedeco.opencv.global.opencv_calib3d;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point3fVectorVector;
import org.bytedeco.opencv.opencv_core.Size;
import org.bytedeco.opencv.opencv_features2d.SimpleBlobDetector;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.logging.HDF5ImageBrowser;
import us.ihmc.rdx.logging.HDF5ImageLogging;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.ImGuiOpenCVSwapVideoPanelData;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.function.Consumer;

public class BlackflyCalibrationSuite
{
   private static final String BLACKFLY_SERIAL_NUMBER = System.getProperty("blackfly.serial.number", "00000000");

   private final Activator nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(),
                                                  "ihmc-open-robotics-software",
                                                  "ihmc-high-level-behaviors/src/libgdx/resources",
                                                  "Blackfly Calibration Suite");
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private RDXBlackflyReader blackflyReader;
   private CalibrationPatternDetectionUI calibrationPatternDetectionUI;
   private HDF5ImageLogging hdf5ImageLogging;
   private HDF5ImageBrowser hdf5ImageBrowser;
   private RDXCVImagePanel calibrationSourceImagesPanel;
   private final RecyclingArrayList<Mat> calibrationSourceImages = new RecyclingArrayList<>(Mat::new);
   private final ImInt calibrationSourceImageIndex = new ImInt();
   private volatile boolean running = true;
   private final Consumer<ImGuiOpenCVSwapVideoPanelData> accessOnHighPriorityThread = this::accessOnHighPriorityThread;
   private Point3fVectorVector objectPoints;
   private Mat grayscaleImage;
   private Mat calibrationPatternOutput;
   private final Notification calibrationOutputNotification = new Notification();
   private final Object grayscaleImageInputSync = new Object();
   private final Object calibrationPatternOutputSync = new Object();
   private final ResettableExceptionHandlingExecutorService patternDetectionThreadQueue
         = MissingThreadTools.newSingleThreadExecutor("PatternDetection", true, 1);
   private boolean patternFound = false;
   private Mat cornersOrCenters;
   private SimpleBlobDetector simpleBlobDetector;

   public BlackflyCalibrationSuite()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            blackflyReader = new RDXBlackflyReader(nativesLoadedActivator, BLACKFLY_SERIAL_NUMBER);
            baseUI.getImGuiPanelManager().addPanel(blackflyReader.getStatisticsPanel());

            baseUI.getImGuiPanelManager().addPanel("Calibration", BlackflyCalibrationSuite.this::renderImGuiWidgets);
         }

         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  blackflyReader.create();
                  baseUI.getImGuiPanelManager().addPanel(blackflyReader.getSwapCVPanel().getVideoPanel());

                  calibrationPatternDetectionUI = new CalibrationPatternDetectionUI();
                  baseUI.getImGuiPanelManager().addPanel(calibrationPatternDetectionUI.getPanel());

                  hdf5ImageBrowser = new HDF5ImageBrowser();
                  baseUI.getImGuiPanelManager().addPanel(hdf5ImageBrowser.getControlPanel());
                  baseUI.getImGuiPanelManager().addPanel(hdf5ImageBrowser.getImagePanel().getVideoPanel());

                  calibrationSourceImagesPanel = new RDXCVImagePanel("Calibration Source Image", 100, 100);
                  baseUI.getImGuiPanelManager().addPanel(calibrationSourceImagesPanel.getVideoPanel());

                  baseUI.getLayoutManager().reloadLayout();

                  grayscaleImage = new Mat();
                  calibrationPatternOutput = new Mat();
                  cornersOrCenters = new Mat();
                  simpleBlobDetector = SimpleBlobDetector.create();

                  ThreadTools.startAsDaemon(() ->
                  {
                     while (running)
                     {
                        blackflyReader.readBlackflyImage();
                        calibrationPatternDetectionUI.copyRGBImage(blackflyReader.getRGBImage());

                        if (hdf5ImageLogging != null)
                           hdf5ImageLogging.copyRGBImage(blackflyReader.getRGBImage());
                     }
                  }, "CameraRead");
               }

               calibrationPatternDetectionUI.update();
               blackflyReader.getSwapCVPanel().getDataSwapReferenceManager().accessOnHighPriorityThread(accessOnHighPriorityThread);
               hdf5ImageBrowser.update();

               if (calibrationOutputNotification.poll())
               {
                  synchronized (calibrationPatternOutputSync)
                  {
                     calibrationSourceImagesPanel.resize(calibrationPatternOutput.cols(), calibrationPatternOutput.rows(), null);
                     calibrationPatternOutput.copyTo(calibrationSourceImagesPanel.getBytedecoImage().getBytedecoOpenCVMat());
                  }
               }

               calibrationSourceImagesPanel.draw();
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            running = false;
            blackflyReader.dispose();
            baseUI.dispose();
         }
      });
   }

   private void accessOnHighPriorityThread(ImGuiOpenCVSwapVideoPanelData data)
   {
      if (data.getRGBA8Image() != null)
      {
         if (blackflyReader.getImageWasRead())
         {
            if (hdf5ImageLogging == null)
            {
               hdf5ImageLogging = new HDF5ImageLogging(nativesLoadedActivator, (int) blackflyReader.getImageWidth(), (int) blackflyReader.getImageHeight());
               baseUI.getImGuiPanelManager().addPanel(hdf5ImageLogging.getPanel());
               baseUI.getLayoutManager().reloadLayout();
            }

            calibrationPatternDetectionUI.drawCornersOrCenters(data.getRGBA8Mat());
         }

         blackflyReader.accessOnHighPriorityThread(data);
      }
   }

   private void renderImGuiWidgets()
   {
      if (hdf5ImageBrowser.getDataSetIsOpen() && ImGui.button("Load sources from open data set"))
      {
         calibrationSourceImages.clear();
         calibrationSourceImageIndex.set(0);
         for (int i = 0; i < hdf5ImageBrowser.getNumberOfImages(); i++)
         {
            hdf5ImageBrowser.loadDataSetImage(i, calibrationSourceImages.add());
         }
         loadCalibrationSourceImage();
      }
      if (!calibrationSourceImages.isEmpty())
      {
         if (ImGui.sliderInt(labels.get("Index"), calibrationSourceImageIndex.getData(), 0, calibrationSourceImages.size() - 1))
         {
            loadCalibrationSourceImage();
         }
      }

      if (ImGui.button(labels.get("Start recording data set")))
      {

      }

      if (ImGui.button(labels.get("Capture calibration image")))
      {

      }

      if (ImGui.button(labels.get("Calibrate")))
      {
         ThreadTools.startAsDaemon(this::calibrate, "Calibration");
      }
   }

   private void loadCalibrationSourceImage()
   {
      Mat selectedImage = calibrationSourceImages.get(calibrationSourceImageIndex.get());
      CalibrationPatternType pattern = calibrationPatternDetectionUI.getPatternType();
      int patternWidth = calibrationPatternDetectionUI.getPatternWidth();
      int patternHeight = calibrationPatternDetectionUI.getPatternHeight();

      synchronized (grayscaleImageInputSync)
      {
         opencv_imgproc.cvtColor(selectedImage, grayscaleImage, opencv_imgproc.COLOR_BGR2GRAY);

         synchronized (calibrationPatternOutputSync)
         {
            selectedImage.copyTo(calibrationPatternOutput);
         }
      }

      patternDetectionThreadQueue.clearQueueAndExecute(() ->
      {
         Size patternSize = new Size(patternWidth, patternHeight);
         if (pattern == CalibrationPatternType.CHESSBOARD)
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

         synchronized (calibrationPatternOutputSync)
         {
            opencv_calib3d.drawChessboardCorners(calibrationPatternOutput, patternSize, cornersOrCenters, patternFound);
         }

         calibrationOutputNotification.set();
      });
   }

   private void calibrate()
   {
      objectPoints = new Point3fVectorVector();


//      opencv_calib3d.calibrate();
   }

   public static void main(String[] args)
   {
      new BlackflyCalibrationSuite();
   }
}
