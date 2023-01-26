package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.type.ImDouble;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import org.bytedeco.javacv.JavaCV;
import org.bytedeco.opencv.global.opencv_calib3d;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.*;
import org.bytedeco.opencv.opencv_features2d.SimpleBlobDetector;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.logging.HDF5ImageBrowser;
import us.ihmc.rdx.logging.HDF5ImageLogging;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.ImGuiOpenCVSwapVideoPanelData;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.function.Consumer;

public class BlackflyCalibrationSuite
{
   private static final String BLACKFLY_SERIAL_NUMBER = System.getProperty("blackfly.serial.number", "00000000");

   // https://www.fujifilm.com/us/en/business/optical-devices/machine-vision-lens/fe185-series
   private static final double FE185C086HA_1_FOCAL_LENGTH = 0.0027;
   // https://www.flir.com/products/blackfly-usb3/?model=BFLY-U3-23S6C-C&vertical=machine+vision&segment=iis
   private static final double BFLY_U3_23S6C_CMOS_SENSOR_FORMAT = UnitConversions.inchesToMeters(1.0 / 1.2);
   private static final double BFLY_U3_23S6C_CMOS_SENSOR_WIDTH = 0.01067;
   private static final double BFLY_U3_23S6C_CMOS_SENSOR_HEIGHT = 0.00800;
   private static final double BFLY_U3_23S6C_WIDTH_PIXELS = 1920.0;
   private static final double BFLY_U3_23S6C_HEIGHT_PIXELS = 1200.0;
   private static final double FE185C086HA_1_FOCAL_LENGTH_IN_BFLY_U3_23S6C_PIXELS
         = FE185C086HA_1_FOCAL_LENGTH * BFLY_U3_23S6C_WIDTH_PIXELS / BFLY_U3_23S6C_CMOS_SENSOR_WIDTH;

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
   private Point2fVectorVector imagePoints;
   private Mat grayscaleImage;
   private Mat calibrationPatternOutput;
   private final Notification calibrationOutputNotification = new Notification();
   private final Object calibrationPatternOutputSync = new Object();
   private final ResettableExceptionHandlingExecutorService patternDetectionThreadQueue = MissingThreadTools.newSingleThreadExecutor("PatternDetection", true);
   private boolean patternFound = false;
   private MatVector cornersOrCentersMatVector;
   private SimpleBlobDetector simpleBlobDetector;
   private final ImFloat patternDistanceBetweenPoints = new ImFloat();
   private final ImDouble fxGuess = new ImDouble(FE185C086HA_1_FOCAL_LENGTH_IN_BFLY_U3_23S6C_PIXELS);
   private final ImDouble fyGuess = new ImDouble(FE185C086HA_1_FOCAL_LENGTH_IN_BFLY_U3_23S6C_PIXELS);
   private final ImDouble cxGuess = new ImDouble(BFLY_U3_23S6C_WIDTH_PIXELS / 2.0);
   private final ImDouble cyGuess = new ImDouble(BFLY_U3_23S6C_HEIGHT_PIXELS / 2.0);
   private final MatVector estimatedRotationVectors = new MatVector();
   private final MatVector estimatedTranslationVectors = new MatVector();

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
                  cornersOrCentersMatVector = new MatVector();
                  imagePoints = new Point2fVectorVector();
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
         drawPatternOnCurrentImage();
         findCornersOrCenters();
      }
      if (ImGui.button("Find corners or centers again"))
      {
         findCornersOrCenters();
      }
      if (!calibrationSourceImages.isEmpty())
      {
         if (ImGui.sliderInt(labels.get("Index"), calibrationSourceImageIndex.getData(), 0, calibrationSourceImages.size() - 1))
         {
            drawPatternOnCurrentImage();
         }
      }

      ImGui.inputFloat(labels.get("Pattern distance between points"), patternDistanceBetweenPoints);
      ImGui.inputDouble(labels.get("Fx Guess (px)"), fxGuess);
      ImGui.inputDouble(labels.get("Fy Guess (px)"), fyGuess);
      ImGui.inputDouble(labels.get("Cx Guess (px)"), cxGuess);
      ImGui.inputDouble(labels.get("Cy Guess (px)"), cyGuess);

      if (ImGui.button(labels.get("Calibrate")))
      {
         ThreadTools.startAsDaemon(this::calibrate, "Calibration");
      }
   }

   private void findCornersOrCenters()
   {
      patternDetectionThreadQueue.execute(() ->
      {
         cornersOrCentersMatVector.clear();
         imagePoints.clear();

         for (int i = 0; i < calibrationSourceImages.size(); i++)
         {
            CalibrationPatternType pattern = calibrationPatternDetectionUI.getPatternType();
            int patternWidth = calibrationPatternDetectionUI.getPatternWidth();
            int patternHeight = calibrationPatternDetectionUI.getPatternHeight();
            Size patternSize = new Size(patternWidth, patternHeight);

            opencv_imgproc.cvtColor(calibrationSourceImages.get(i), grayscaleImage, opencv_imgproc.COLOR_BGR2GRAY);

            Mat cornersOrCentersMat = new Mat();
            if (pattern == CalibrationPatternType.CHESSBOARD)
            {
               patternFound = opencv_calib3d.findChessboardCorners(grayscaleImage,
                                                                   patternSize,
                                                                   cornersOrCentersMat,
                                                                   opencv_calib3d.CALIB_CB_ADAPTIVE_THRESH | opencv_calib3d.CALIB_CB_NORMALIZE_IMAGE);
            }
            else
            {
               patternFound = opencv_calib3d.findCirclesGrid(grayscaleImage,
                                                             patternSize,
                                                             cornersOrCentersMat,
                                                             opencv_calib3d.CALIB_CB_SYMMETRIC_GRID,
                                                             simpleBlobDetector);
            }
            cornersOrCentersMatVector.push_back(cornersOrCentersMat);

            Point2fVector cornersOrCenters = new Point2fVector();
            for (int x = 0; x < cornersOrCentersMat.cols(); x++)
            {
               for (int y = 0; y < cornersOrCentersMat.rows(); y++)
               {
                  cornersOrCenters.push_back(new Point2f(cornersOrCentersMat.ptr(y, x)));
               }
            }
            imagePoints.push_back(cornersOrCenters);
         }
      });
   }

   // TODO: Potentially put this on another thread; not sure how long it takes
   private void drawPatternOnCurrentImage()
   {
      int patternWidth = calibrationPatternDetectionUI.getPatternWidth();
      int patternHeight = calibrationPatternDetectionUI.getPatternHeight();
      Size patternSize = new Size(patternWidth, patternHeight);
      int sourceImageIndex = calibrationSourceImageIndex.get();
      Mat selectedImage = calibrationSourceImages.get(sourceImageIndex);
      selectedImage.copyTo(calibrationPatternOutput);
      opencv_calib3d.drawChessboardCorners(calibrationPatternOutput, patternSize, cornersOrCentersMatVector.get(sourceImageIndex), patternFound);
      calibrationOutputNotification.set();
   }

   private void calibrate()
   {
      int patternWidth = calibrationPatternDetectionUI.getPatternWidth();
      int patternHeight = calibrationPatternDetectionUI.getPatternHeight();

      Point3fVectorVector objectPoints = new Point3fVectorVector();
      MatVector objectPointsMatVector = new MatVector();
      for (int i = 0; i < calibrationSourceImages.size(); i++)
      {
         Point3fVector pointsOnPattern = new Point3fVector();
         for (int y = 0; y < patternHeight; y++)
         {
            for (int x = 0; x < patternWidth; x++) // The same pattern is used for all the images, but we still have to make all the copies
            {
               pointsOnPattern.push_back(new Point3f(x * patternDistanceBetweenPoints.get(), y * patternDistanceBetweenPoints.get(), 0.0f));
            }
         }
         objectPoints.push_back(pointsOnPattern);
         objectPointsMatVector.push_back(new Mat(pointsOnPattern)); // TODO: Does this work?
      }

      MatVector imagePointsMatVector = new MatVector();
      for (int i = 0; i < imagePoints.size(); i++)
      {
         imagePointsMatVector.push_back(new Mat(imagePoints.get(i))); // TODO: Does this work?
      }

      Size imageSize = new Size(calibrationSourceImages.get(0).cols(), calibrationSourceImages.get(0).rows());

      Mat cameraMatrix = Mat.eye(3, 3, opencv_core.CV_64F).asMat();
      // Using fisheye::CALIB_USE_INTRINSIC_GUESS
      cameraMatrix.ptr(0, 0).putDouble(fxGuess.get());
      cameraMatrix.ptr(1, 1).putDouble(fyGuess.get());
      cameraMatrix.ptr(0, 2).putDouble(cxGuess.get());
      cameraMatrix.ptr(1, 2).putDouble(cyGuess.get());

      Mat distortionCoefficients = Mat.zeros(4, 1, opencv_core.CV_64F).asMat();

      estimatedRotationVectors.clear();
      estimatedTranslationVectors.clear();

      // TODO: We may want to fix the principal point and focal length
      int flags = opencv_calib3d.FISHEYE_CALIB_USE_INTRINSIC_GUESS;

      TermCriteria terminationCriteria = new TermCriteria(TermCriteria.COUNT + TermCriteria.EPS, 100, JavaCV.DBL_EPSILON);

      // Here we use the cv::fisheye version
      // https://docs.opencv.org/4.6.0/db/d58/group__calib3d__fisheye.html#gad626a78de2b1dae7489e152a5a5a89e1
      opencv_calib3d.calibrate(objectPointsMatVector,
                               imagePointsMatVector,
                               imageSize,
                               cameraMatrix,
                               distortionCoefficients,
                               estimatedRotationVectors,
                               estimatedTranslationVectors,
                               flags,
                               terminationCriteria);

      LogTools.info("Calibration complete!");
      LogTools.info("# estimated rotation vectors: {}", estimatedRotationVectors.size());
      LogTools.info("# estimated translation vectors: {}", estimatedTranslationVectors.size());
   }

   public static void main(String[] args)
   {
      new BlackflyCalibrationSuite();
   }
}
