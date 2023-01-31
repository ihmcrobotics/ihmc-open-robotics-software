package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.flag.ImGuiInputTextFlags;
import imgui.type.*;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacv.JavaCV;
import org.bytedeco.opencv.global.opencv_calib3d;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.*;
import org.bytedeco.opencv.opencv_features2d.SimpleBlobDetector;
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
import us.ihmc.rdx.ui.graphics.ImGuiOpenCVSwapVideoPanel;
import us.ihmc.rdx.ui.graphics.ImGuiOpenCVSwapVideoPanelData;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

/**
 * Official OpenCV camera calibration tutorial:
 * https://docs.opencv.org/4.x/d4/d94/tutorial_camera_calibration.html
 */
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
   private ImGuiOpenCVSwapVideoPanel undistortedFisheyePanel;
   private RDXCVImagePanel calibrationSourceImagesPanel;
   private final RecyclingArrayList<Mat> calibrationSourceImages = new RecyclingArrayList<>(Mat::new);
   private final ImInt calibrationSourceImageIndex = new ImInt();
   private volatile boolean running = true;
   private Point2fVectorVector imagePoints;
   private MatVector imagePointsMatVector;
   private Mat grayscaleImage;
   private Mat calibrationPatternOutput;
   private final ResettableExceptionHandlingExecutorService patternDetectionThreadQueue
         = MissingThreadTools.newSingleThreadExecutor("PatternDetection", true, 1);
   private boolean patternFound = false;
   private final Notification calibrationSourceImagePatternDrawRequest = new Notification();
   private final Notification calibrationSourceImageDrawRequest = new Notification();
   private MatVector cornersOrCentersMatVector;
   private SimpleBlobDetector simpleBlobDetector;
   private final ImFloat patternDistanceBetweenPoints = new ImFloat(0.0189f);
   private final ImDouble fxGuess = new ImDouble(FE185C086HA_1_FOCAL_LENGTH_IN_BFLY_U3_23S6C_PIXELS);
   private final ImDouble fyGuess = new ImDouble(FE185C086HA_1_FOCAL_LENGTH_IN_BFLY_U3_23S6C_PIXELS);
   private final ImDouble cxGuess = new ImDouble(BFLY_U3_23S6C_WIDTH_PIXELS / 2.0);
   private final ImDouble cyGuess = new ImDouble(BFLY_U3_23S6C_HEIGHT_PIXELS / 2.0);
   private final MatVector estimatedRotationVectors = new MatVector();
   private final MatVector estimatedTranslationVectors = new MatVector();
   private double averageReprojectionError = Double.NaN;
   private final ImBoolean useIntrinsicsGuess = new ImBoolean(true);
   private final ImBoolean recomputeExtrinsic = new ImBoolean(false);
   private final ImBoolean checkValidityOfConditionNumber = new ImBoolean(false);
   private final ImBoolean fixSkew = new ImBoolean(false);
   private final ImBoolean fixPrincipalPoint = new ImBoolean(false);
   private final ImBoolean fixFocalLength = new ImBoolean(false);
   private final ImDouble calibratedFx = new ImDouble(fxGuess.get());
   private final ImDouble calibratedFy = new ImDouble(fyGuess.get());
   private final ImDouble calibratedCx = new ImDouble(cxGuess.get());
   private final ImDouble calibratedCy = new ImDouble(cyGuess.get());
   private final ImString cameraMatrixAsText = new ImString(512);
   private final ImDouble distortionCoefficientK1 = new ImDouble(0.01758);
   private final ImDouble distortionCoefficientK2 = new ImDouble(0.00455);
   private final ImDouble distortionCoefficientK3 = new ImDouble(-0.00399);
   private final ImDouble distortionCoefficientK4 = new ImDouble(0.00051);
   private final ImInt undistortedImageWidth = new ImInt((int) BFLY_U3_23S6C_WIDTH_PIXELS);
   private final ImInt undistortedImageHeight = new ImInt((int) BFLY_U3_23S6C_HEIGHT_PIXELS);
   private Mat distortionCoefficients;
   private Mat distortionCoefficientsCopy;
   private Mat cameraMatrixForMonitor;
   private Mat cameraMatrixForMonitorShifting;
   private Mat bgrImageForUndistortion;
   private final Notification calibrationFinished = new Notification();
   private Mat distortedImageCopy;
   private Mat undistortedLiveImage;
   private Mat cameraMatrix;
   private Size undistortedImageSize;
   private Mat identityCameraMatrix;

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
                  blackflyReader.setMonitorPanelUIThreadPreprocessor(this::blackflyReaderUIThreadPreprocessor);
                  baseUI.getImGuiPanelManager().addPanel(blackflyReader.getSwapCVPanel().getVideoPanel());

                  calibrationPatternDetectionUI = new CalibrationPatternDetectionUI();
                  baseUI.getImGuiPanelManager().addPanel(calibrationPatternDetectionUI.getPanel());

                  hdf5ImageBrowser = new HDF5ImageBrowser();
                  baseUI.getImGuiPanelManager().addPanel(hdf5ImageBrowser.getControlPanel());
                  baseUI.getImGuiPanelManager().addPanel(hdf5ImageBrowser.getImagePanel().getVideoPanel());

                  calibrationSourceImagesPanel = new RDXCVImagePanel("Calibration Source Image", 100, 100);
                  baseUI.getImGuiPanelManager().addPanel(calibrationSourceImagesPanel.getVideoPanel());

                  undistortedFisheyePanel = new ImGuiOpenCVSwapVideoPanel("Undistorted Fisheye Monitor",
                                                                          this::undistortedImageUpdateOnAsynchronousThread,
                                                                          this::undistoredImageUIThread);
                  baseUI.getImGuiPanelManager().addPanel(undistortedFisheyePanel.getVideoPanel());

                  baseUI.getLayoutManager().reloadLayout();

                  grayscaleImage = new Mat();
                  calibrationPatternOutput = new Mat();
                  cornersOrCentersMatVector = new MatVector();
                  imagePoints = new Point2fVectorVector();
                  imagePointsMatVector = new MatVector();
                  simpleBlobDetector = SimpleBlobDetector.create();
//                  distortionCoefficients = new Mat(distortionCoefficientK1.get(),
//                                                   distortionCoefficientK2.get(),
//                                                   distortionCoefficientK3.get(),
//                                                   distortionCoefficientK4.get());
                  distortionCoefficients = Mat.zeros(4, 1, opencv_core.CV_64F).asMat();
                  distortionCoefficients.ptr(0, 0).putDouble(distortionCoefficientK1.get());
                  distortionCoefficients.ptr(1, 0).putDouble(distortionCoefficientK2.get());
                  distortionCoefficients.ptr(2, 0).putDouble(distortionCoefficientK3.get());
                  distortionCoefficients.ptr(3, 0).putDouble(distortionCoefficientK4.get());
                  distortionCoefficientsCopy = new Mat();
                  distortionCoefficients.copyTo(distortionCoefficientsCopy);
                  MatExpr eyeExpression = Mat.eye(3, 3, opencv_core.CV_64F);
                  identityCameraMatrix = eyeExpression.asMat();
                  eyeExpression.close();
                  cameraMatrix = new Mat();
                  identityCameraMatrix.copyTo(cameraMatrix);
                  cameraMatrix.ptr(0, 0).putDouble(calibratedFx.get());
                  cameraMatrix.ptr(1, 1).putDouble(calibratedFy.get());
                  cameraMatrix.ptr(0, 2).putDouble(calibratedCx.get());
                  cameraMatrix.ptr(1, 2).putDouble(calibratedCy.get());
                  cameraMatrixForMonitor = new Mat();
                  cameraMatrix.copyTo(cameraMatrixForMonitor);
//                  cameraMatrixForMonitorShifting = Mat.eye(3, 3, opencv_core.CV_64F).asMat();
                  cameraMatrixForMonitorShifting = opencv_core.noArray();
                  undistortedImageSize = new Size(undistortedImageWidth.get(), undistortedImageHeight.get());
                  distortedImageCopy = new Mat();
                  undistortedLiveImage = new Mat();
                  bgrImageForUndistortion = new Mat();

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

               if (calibrationFinished.poll())
               {
                  distortionCoefficients.ptr(0, 0).putDouble(distortionCoefficientK1.get());
                  distortionCoefficients.ptr(1, 0).putDouble(distortionCoefficientK2.get());
                  distortionCoefficients.ptr(2, 0).putDouble(distortionCoefficientK3.get());
                  distortionCoefficients.ptr(3, 0).putDouble(distortionCoefficientK4.get());
                  distortionCoefficients.copyTo(distortionCoefficientsCopy);
                  cameraMatrix.copyTo(cameraMatrixForMonitor);
                  cameraMatrixForMonitor.ptr(0, 0).putDouble(calibratedFx.get());
                  cameraMatrixForMonitor.ptr(1, 1).putDouble(calibratedFy.get());
                  cameraMatrixForMonitor.ptr(0, 2).putDouble(calibratedCx.get());
                  cameraMatrixForMonitor.ptr(1, 2).putDouble(calibratedCy.get());

//                  opencv_calib3d.initUndistortRectifyMap(cameraMatrixForMonitor,
//                                                         distortionCoefficientsCopy,
//                                                         );
               }

               StringBuilder stringBuilder = new StringBuilder();
               stringBuilder.append("Camera matrix:\n");
               for (int row = 0; row < 3; row++)
               {
                  for (int col = 0; col < 3; col++)
                  {
                     stringBuilder.append("%.5f".formatted(cameraMatrixForMonitor.ptr(row, col).getDouble()) + " ");
                  }
                  stringBuilder.append("\n");
               }
               cameraMatrixAsText.set(stringBuilder.toString());

               calibrationPatternDetectionUI.update();
               blackflyReader.updateOnUIThread();
               hdf5ImageBrowser.update();

               if (calibrationSourceImageDrawRequest.poll())
               {
                  calibrationSourceImagesPanel.drawResizeAndCopy(calibrationSourceImages.get(calibrationSourceImageIndex.get()));
               }
               if (calibrationSourceImagePatternDrawRequest.poll())
               {
                  drawPatternOnCurrentImage();
                  calibrationSourceImagesPanel.drawResizeAndCopy(calibrationPatternOutput);
               }
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void blackflyReaderUIThreadPreprocessor(ImGuiOpenCVSwapVideoPanelData data)
         {
            if (hdf5ImageLogging == null)
            {
               hdf5ImageLogging = new HDF5ImageLogging(nativesLoadedActivator,
                                                       (int) blackflyReader.getImageWidth(),
                                                       (int) blackflyReader.getImageHeight());
               baseUI.getImGuiPanelManager().addPanel(hdf5ImageLogging.getPanel());
               baseUI.getLayoutManager().reloadLayout();
            }

            // Access the image before it gets drawn on; copy for the other thread
            data.getRGBA8Mat().copyTo(distortedImageCopy);

            calibrationPatternDetectionUI.drawCornersOrCenters(data.getRGBA8Mat());
         }

         private void undistortedImageUpdateOnAsynchronousThread(ImGuiOpenCVSwapVideoPanelData data)
         {
            // Fisheye undistortion
            // https://docs.opencv.org/4.6.0/db/d58/group__calib3d__fisheye.html#ga167df4b00a6fd55287ba829fbf9913b9
            undistortedImageSize.width(undistortedImageWidth.get());
            undistortedImageSize.height(undistortedImageHeight.get());
      //      opencv_imgproc.cvtColor(distortedImageCopy, bgrImageForUndistortion, opencv_imgproc.COLOR_RGBA2BGR);
            opencv_calib3d.undistortImage(distortedImageCopy,
                                          undistortedLiveImage,
                                          cameraMatrixForMonitor,
                                          distortionCoefficientsCopy,
                                          cameraMatrixForMonitorShifting,
                                          undistortedImageSize);

      //      data.getRGBA8Image()

      //            opencv_calib3d.undistortImage(bgrImageForUndistortion,
      //                                          undistortedLiveImage,
      //                                          cameraMatrixForMonitor,
      //                                          distortionCoefficientsCopy);
         }

         private void undistoredImageUIThread(ImGuiOpenCVSwapVideoPanelData data)
         {
      //      if
      //      undistortedFisheyePanel.resize(undistortedLiveImage.cols(), undistortedLiveImage.rows(), null);
      //      opencv_imgproc.cvtColor(undistortedLiveImage, undistortedFisheyePanel.getBytedecoImage().getBytedecoOpenCVMat(), opencv_imgproc.COLOR_BGR2RGBA);
      //      undistortedFisheyePanel.draw();
         }

         @Override
         public void dispose()
         {
            running = false;
            blackflyReader.dispose();
            hdf5ImageBrowser.destroy();
            hdf5ImageLogging.destroy();
            baseUI.dispose();
         }
      });
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
         calibrationSourceImageDrawRequest.set();
      }
      if (ImGui.button("Find corners or centers"))
      {
         findCornersOrCentersAsync();
      }
      if (!calibrationSourceImages.isEmpty())
      {
         if (ImGui.sliderInt(labels.get("Index"), calibrationSourceImageIndex.getData(), 0, calibrationSourceImages.size() - 1))
         {
            if (cornersOrCentersMatVector.size() > 0)
            {
               calibrationSourceImagePatternDrawRequest.set();
            }
            else
            {
               calibrationSourceImageDrawRequest.set();
            }
         }
      }

      ImGui.inputFloat(labels.get("Pattern distance between points"), patternDistanceBetweenPoints, 0.001f, 0.01f, "%.5f");
      ImGui.inputDouble(labels.get("Fx Guess (px)"), fxGuess);
      ImGui.inputDouble(labels.get("Fy Guess (px)"), fyGuess);
      ImGui.inputDouble(labels.get("Cx Guess (px)"), cxGuess);
      ImGui.inputDouble(labels.get("Cy Guess (px)"), cyGuess);

      ImGui.checkbox(labels.get("Use intrinsic guess"), useIntrinsicsGuess);
      ImGui.checkbox(labels.get("Recompute extrinsic"), recomputeExtrinsic);
      ImGuiTools.previousWidgetTooltip("Extrinsic will be recomputed after each iteration of intrinsic optimization.");
      ImGui.checkbox(labels.get("Check validity of condition number"), checkValidityOfConditionNumber);
      ImGui.checkbox(labels.get("Fix skew (alpha) to 0"), fixSkew);
      ImGui.checkbox(labels.get("Fix principal point to guess"), fixPrincipalPoint);
      ImGui.checkbox(labels.get("Fix focal length to guess"), fixFocalLength);

      ImGui.beginDisabled(patternDetectionThreadQueue.isExecuting());
      if (ImGui.button(labels.get("Calibrate")))
      {
         ThreadTools.startAsDaemon(this::calibrate, "Calibration");
      }
      ImGui.endDisabled();

      if (!Double.isNaN(averageReprojectionError))
      {
         ImGui.text("Average reprojection error: %.5f".formatted(averageReprojectionError));
         ImGuiTools.previousWidgetTooltip("This number (rms) should be as close to zero as possible.");
      }

      // TODO: Make widgets for adjusting the matrix and coeffs
      //   These should affect the live undistorted preview

      boolean userChangedUndistortParameters = false;
      userChangedUndistortParameters |= ImGuiTools.volatileInputDouble(labels.get("Calibrate Fx (px)"), calibratedFx, 100.0, 500.0, "%.5f");
      userChangedUndistortParameters |= ImGuiTools.volatileInputDouble(labels.get("Calibrate Fy (px)"), calibratedFy, 100.0, 500.0, "%.5f");
      userChangedUndistortParameters |= ImGuiTools.volatileInputDouble(labels.get("Calibrate Cx (px)"), calibratedCx, 100.0, 500.0, "%.5f");
      userChangedUndistortParameters |= ImGuiTools.volatileInputDouble(labels.get("Calibrate Cy (px)"), calibratedCy, 100.0, 500.0, "%.5f");

      userChangedUndistortParameters |= ImGuiTools.volatileInputDouble(labels.get("Distortion K1"), distortionCoefficientK1, 0.0001, 0.001, "%.7f");
      userChangedUndistortParameters |= ImGuiTools.volatileInputDouble(labels.get("Distortion K2"), distortionCoefficientK2, 0.0001, 0.001, "%.7f");
      userChangedUndistortParameters |= ImGuiTools.volatileInputDouble(labels.get("Distortion K3"), distortionCoefficientK3, 0.0001, 0.001, "%.7f");
      userChangedUndistortParameters |= ImGuiTools.volatileInputDouble(labels.get("Distortion K4"), distortionCoefficientK4, 0.0001, 0.001, "%.7f");

      ImGuiTools.volatileInputInt(labels.get("Undistorted image width"), undistortedImageWidth);
      ImGuiTools.volatileInputInt(labels.get("Undistorted image height"), undistortedImageHeight);

      ImGui.text("Camera matrix:");
      ImGui.inputTextMultiline(labels.getHidden("cameraMatrix"), cameraMatrixAsText, 0, 60, ImGuiInputTextFlags.ReadOnly);

      if (userChangedUndistortParameters)
         calibrationFinished.set();
   }

   private void findCornersOrCentersAsync()
   {
      patternDetectionThreadQueue.clearQueueAndExecute(() ->
      {
         cornersOrCentersMatVector.clear();
         imagePoints.clear();
         imagePointsMatVector.clear();

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
            LogTools.info("Found corners for image {}: {}", i, patternFound);
            cornersOrCentersMatVector.push_back(cornersOrCentersMat);

            Point2fVector cornersOrCenters = new Point2fVector();
            for (int x = 0; x < cornersOrCentersMat.cols(); x++)
            {
               for (int y = 0; y < cornersOrCentersMat.rows(); y++)
               {
                  BytePointer ptr = cornersOrCentersMat.ptr(y, x);
                  float xValue = ptr.getFloat();
                  float yValue = ptr.getFloat(Float.BYTES);
                  cornersOrCenters.push_back(new Point2f(xValue, yValue));
                  LogTools.debug("image point: {}, {}", xValue, yValue);
               }
            }
            imagePoints.push_back(cornersOrCenters);
            imagePointsMatVector.push_back(cornersOrCentersMat);
         }

         calibrationSourceImagePatternDrawRequest.set();
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
   }

   // TODO: Put on thread
   private void calibrate()
   {
      int patternWidth = calibrationPatternDetectionUI.getPatternWidth();
      int patternHeight = calibrationPatternDetectionUI.getPatternHeight();

      Point3fVectorVector objectPoints = new Point3fVectorVector();
      MatVector objectPointsMatVector = new MatVector();

      for (int i = 0; i < calibrationSourceImages.size(); i++)
      {
         Point3fVector pointsOnPattern = new Point3fVector();
         // We have to pack the Mat version for now until this is fixed:
         // https://github.com/bytedeco/javacpp-presets/issues/1185
         Mat pointsOnPatternMat = new Mat(patternHeight * patternWidth, 1, opencv_core.CV_32FC3);

         for (int y = 0; y < patternHeight; y++)
         {
            for (int x = 0; x < patternWidth; x++) // The same pattern is used for all the images, but we still have to make all the copies
            {
               float xValue = x * patternDistanceBetweenPoints.get();
               float yValue = y * patternDistanceBetweenPoints.get();
               float zValue = 0.0f;
               pointsOnPattern.push_back(new Point3f(xValue, yValue, 0.0f));

               BytePointer pointOnPatternPointer = pointsOnPatternMat.ptr(y * patternWidth + x, 0);
               pointOnPatternPointer.putFloat(xValue);
               pointOnPatternPointer.putFloat(Float.BYTES, yValue);
               pointOnPatternPointer.putFloat(2 * Float.BYTES, zValue);
               LogTools.debug("object point: {}, {}, {}", xValue, yValue, zValue);
            }
         }
         objectPoints.push_back(pointsOnPattern);
         objectPointsMatVector.push_back(pointsOnPatternMat);
      }

      Size imageSize = new Size(calibrationSourceImages.get(0).cols(), calibrationSourceImages.get(0).rows());

      identityCameraMatrix.copyTo(cameraMatrix);
      // Using fisheye::CALIB_USE_INTRINSIC_GUESS
      cameraMatrix.ptr(0, 0).putDouble(fxGuess.get());
      cameraMatrix.ptr(1, 1).putDouble(fyGuess.get());
      cameraMatrix.ptr(0, 2).putDouble(cxGuess.get());
      cameraMatrix.ptr(1, 2).putDouble(cyGuess.get());

      estimatedRotationVectors.clear();
      estimatedTranslationVectors.clear();

      // TODO: We may want to fix the principal point and focal length
      int flags = 0;
      if (useIntrinsicsGuess.get())
         flags |= opencv_calib3d.FISHEYE_CALIB_USE_INTRINSIC_GUESS;
      if (recomputeExtrinsic.get())
         flags |= opencv_calib3d.FISHEYE_CALIB_RECOMPUTE_EXTRINSIC;
      if (checkValidityOfConditionNumber.get())
         flags |= opencv_calib3d.FISHEYE_CALIB_CHECK_COND;
      if (fixSkew.get())
         flags |= opencv_calib3d.FISHEYE_CALIB_FIX_SKEW;
      if (fixPrincipalPoint.get())
         flags |= opencv_calib3d.FISHEYE_CALIB_FIX_PRINCIPAL_POINT;
      if (fixFocalLength.get())
         flags |= opencv_calib3d.FISHEYE_CALIB_FIX_FOCAL_LENGTH;

      TermCriteria terminationCriteria = new TermCriteria(TermCriteria.COUNT + TermCriteria.EPS, 100, JavaCV.DBL_EPSILON);

      // Here we use the cv::fisheye version
      // https://docs.opencv.org/4.6.0/db/d58/group__calib3d__fisheye.html#gad626a78de2b1dae7489e152a5a5a89e1
      averageReprojectionError = opencv_calib3d.calibrate(objectPointsMatVector,
                                                          imagePointsMatVector,
                                                          imageSize,
                                                          cameraMatrix,
                                                          distortionCoefficients,
                                                          estimatedRotationVectors,
                                                          estimatedTranslationVectors,
                                                          flags,
                                                          terminationCriteria);

      LogTools.info("Calibration complete!");
      LogTools.info("Number of estimated rotation vectors: {}", estimatedRotationVectors.size());
      LogTools.info("Number of estimated translation vectors: {}", estimatedTranslationVectors.size());

      StringBuilder stringBuilder = new StringBuilder();
      stringBuilder.append("Camera matrix:\n");
      for (int row = 0; row < 3; row++)
      {
         for (int col = 0; col < 3; col++)
         {
            stringBuilder.append("%.5f".formatted(cameraMatrix.ptr(row, col).getDouble()) + " ");
         }
         stringBuilder.append("\n");
      }
//      cameraMatrixAsText.set(stringBuilder.toString());

      calibratedFx.set(cameraMatrix.ptr(0, 0).getDouble());
      calibratedFy.set(cameraMatrix.ptr(1, 1).getDouble());
      calibratedCx.set(cameraMatrix.ptr(0, 2).getDouble());
      calibratedCy.set(cameraMatrix.ptr(1, 2).getDouble());

      distortionCoefficientK1.set(distortionCoefficients.ptr(0, 0).getDouble());
      distortionCoefficientK2.set(distortionCoefficients.ptr(1, 0).getDouble());
      distortionCoefficientK3.set(distortionCoefficients.ptr(2, 0).getDouble());
      distortionCoefficientK4.set(distortionCoefficients.ptr(3, 0).getDouble());

      calibrationFinished.set();
   }

   public static void main(String[] args)
   {
      new BlackflyCalibrationSuite();
   }
}
