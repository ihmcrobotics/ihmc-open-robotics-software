package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.flag.ImGuiDataType;
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
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.OpenCVArUcoMarker;
import us.ihmc.perception.OpenCVArUcoMarkerDetection;
import us.ihmc.perception.sensorHead.SensorHeadParameters;
import us.ihmc.perception.spinnaker.SpinnakerBlackfly;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.logging.RDXHDF5ImageBrowser;
import us.ihmc.rdx.logging.RDXHDF5ImageLoggingUI;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXOpenCVSwapVideoPanel;
import us.ihmc.rdx.ui.graphics.RDXImagePanelTexture;
import us.ihmc.rdx.ui.interactable.RDXInteractableBlackflyFujinon;
import us.ihmc.tools.thread.*;

import java.util.ArrayList;

/**
 * This application allows to create calibration datasets, load datasets,
 * calibrate the fisheye camera setup, visualize the undistorted result,
 * and tune all involved parameters.
 *
 * Official OpenCV camera calibration tutorial:
 * https://docs.opencv.org/4.x/d4/d94/tutorial_camera_calibration.html
 */
public class RDXBlackflyCalibrationSuite
{
   private static final String BLACKFLY_SERIAL_NUMBER = System.getProperty("blackfly.serial.number", "00000000");

   private final Activator nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();
   private final RDXBaseUI baseUI = new RDXBaseUI("Blackfly Calibration Suite");
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private RDXBlackflyReader blackflyReader;
   private RDXCalibrationPatternDetectionUI calibrationPatternDetectionUI;
   private RDXHDF5ImageLoggingUI hdf5ImageLoggingUI;
   private RDXHDF5ImageBrowser hdf5ImageBrowser;
   private RDXOpenCVSwapVideoPanel undistortedFisheyePanel;
   private RDXBytedecoImagePanel calibrationSourceImagesPanel;
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
   private final ImDouble fxGuess = new ImDouble(SensorHeadParameters.FE185C086HA_1_FOCAL_LENGTH_IN_BFLY_U3_23S6C_PIXELS);
   private final ImDouble fyGuess = new ImDouble(SensorHeadParameters.FE185C086HA_1_FOCAL_LENGTH_IN_BFLY_U3_23S6C_PIXELS);
   private final ImDouble cxGuess = new ImDouble(SpinnakerBlackfly.BFLY_U3_23S6C_WIDTH_PIXELS / 2.0);
   private final ImDouble cyGuess = new ImDouble(SpinnakerBlackfly.BFLY_U3_23S6C_HEIGHT_PIXELS / 2.0);
   private final MatVector estimatedRotationVectors = new MatVector();
   private final MatVector estimatedTranslationVectors = new MatVector();
   private double averageReprojectionError = Double.NaN;
   private final ImBoolean useIntrinsicsGuess = new ImBoolean(true);
   private final ImBoolean recomputeExtrinsic = new ImBoolean(false);
   private final ImBoolean checkValidityOfConditionNumber = new ImBoolean(false);
   private final ImBoolean fixSkew = new ImBoolean(false);
   private final ImBoolean fixPrincipalPoint = new ImBoolean(false);
   private final ImBoolean fixFocalLength = new ImBoolean(false);
   private final ImDouble calibratedFxForUndistortion = new ImDouble(fxGuess.get());
   private final ImDouble calibratedFyForUndistortion = new ImDouble(fyGuess.get());
   private final ImDouble calibratedCxForUndistortion = new ImDouble(cxGuess.get());
   private final ImDouble calibratedCyForUndistortion = new ImDouble(cyGuess.get());
   private final ImDouble manuallyTunedFxForColoring = new ImDouble(SensorHeadParameters.FOCAL_LENGTH_X_FOR_COLORING);
   private final ImDouble manuallyTunedFyForColoring = new ImDouble(SensorHeadParameters.FOCAL_LENGTH_Y_FOR_COLORING);
   private final ImDouble manuallyTunedCxForColoring = new ImDouble(SensorHeadParameters.PRINCIPAL_POINT_X_FOR_COLORING);
   private final ImDouble manuallyTunedCyForColoring = new ImDouble(SensorHeadParameters.PRINCIPAL_POINT_Y_FOR_COLORING);
   private final ImString coloringCameraMatrixAsText = new ImString(512);
   private final ImString cameraMatrixAsText = new ImString(512);
   private final ImString newCameraMatrixAsText = new ImString(512);
   private final ImDouble distortionCoefficientK1 = new ImDouble(0.01758);
   private final ImDouble distortionCoefficientK2 = new ImDouble(0.00455);
   private final ImDouble distortionCoefficientK3 = new ImDouble(-0.00399);
   private final ImDouble distortionCoefficientK4 = new ImDouble(0.00051);
   private final ImInt undistortedImageWidth = new ImInt((int) SpinnakerBlackfly.BFLY_U3_23S6C_WIDTH_PIXELS);
   private final ImInt undistortedImageHeight = new ImInt((int) SpinnakerBlackfly.BFLY_U3_23S6C_HEIGHT_PIXELS);
   private final ImDouble balanceNewFocalLength = new ImDouble(0.0);
   private final ImDouble fovScaleFocalLengthDivisor = new ImDouble(1.0);
   private Mat distortionCoefficients;
   private Mat cameraMatrix;
   private Mat coloringCameraMatrix;
   private SwapReference<Mat> imageForUndistortion;
   private SwapReference<Mat> cameraMatrixForUndistortion;
   private SwapReference<Mat> distortionCoefficientsForUndistortion;
   private Mat cameraMatrixForMonitorShifting;
   private Size sourceImageSize;
   private Size undistortedImageSize;
   private Mat rectificationTransformation;
   private Mat newCameraMatrixEstimate;
   private final Throttler undistortionThrottler = new Throttler().setFrequency(60.0);
   private OpenCVArUcoMarkerDetection openCVArUcoMarkerDetection;
   private Mat spareRGBMatForArUcoDrawing;
   private RDXOpenCVArUcoMarkerDetectionUI arUcoMarkerDetectionUI;
   private final RDXNettyOusterUI nettyOusterUI = new RDXNettyOusterUI();
   private RDXInteractableBlackflyFujinon interactableBlackflyFujinon;
   private ReferenceFrame blackflySensorFrame;

   public RDXBlackflyCalibrationSuite()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            blackflyReader = new RDXBlackflyReader(nativesLoadedActivator, BLACKFLY_SERIAL_NUMBER);
            baseUI.getImGuiPanelManager().addPanel(blackflyReader.getStatisticsPanel());

            baseUI.getImGuiPanelManager().addPanel("Calibration", RDXBlackflyCalibrationSuite.this::renderImGuiWidgets);

            nettyOusterUI.create(baseUI);
            ImGuiPanel panel = new ImGuiPanel("Ouster", nettyOusterUI::renderImGuiWidgets);
            baseUI.getImGuiPanelManager().addPanel(panel);
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
                  baseUI.getImGuiPanelManager().addPanel(blackflyReader.getSwapImagePanel().getImagePanel());

                  calibrationPatternDetectionUI = new RDXCalibrationPatternDetectionUI();
                  baseUI.getImGuiPanelManager().addPanel(calibrationPatternDetectionUI.getPanel());

                  hdf5ImageBrowser = new RDXHDF5ImageBrowser();
                  baseUI.getImGuiPanelManager().addPanel(hdf5ImageBrowser.getControlPanel());
                  baseUI.getImGuiPanelManager().addPanel(hdf5ImageBrowser.getImagePanel().getImagePanel());

                  calibrationSourceImagesPanel = new RDXBytedecoImagePanel("Calibration Source Image", 100, 100);
                  baseUI.getImGuiPanelManager().addPanel(calibrationSourceImagesPanel.getImagePanel());

                  undistortedFisheyePanel = new RDXOpenCVSwapVideoPanel("Undistorted Fisheye Monitor");
                  baseUI.getImGuiPanelManager().addPanel(undistortedFisheyePanel.getImagePanel());

                  interactableBlackflyFujinon = new RDXInteractableBlackflyFujinon(baseUI.getPrimary3DPanel());

                  openCVArUcoMarkerDetection = new OpenCVArUcoMarkerDetection();
                  blackflySensorFrame = interactableBlackflyFujinon.getInteractableFrameModel().getReferenceFrame();
                  openCVArUcoMarkerDetection.create(blackflySensorFrame);
                  arUcoMarkerDetectionUI = new RDXOpenCVArUcoMarkerDetectionUI();
                  ArrayList<OpenCVArUcoMarker> markersToTrack = new ArrayList<>();
                  double sideLength = 0.1982;
                  markersToTrack.add(new OpenCVArUcoMarker(0, sideLength));
                  // TODO: Use frame from openCVArUcoMarkerDetection? i.e. remove redudant parameter
                  arUcoMarkerDetectionUI.create(openCVArUcoMarkerDetection, markersToTrack, blackflySensorFrame);
                  baseUI.getImGuiPanelManager().addPanel(arUcoMarkerDetectionUI.getMainPanel());
                  baseUI.getPrimaryScene().addRenderableProvider(arUcoMarkerDetectionUI::getRenderables, RDXSceneLevel.VIRTUAL);

                  nettyOusterUI.createAfterNativesLoaded();
                  nettyOusterUI.getSensorFrame().update(transformToBlackfly -> transformToBlackfly.set(SensorHeadParameters.OUSTER_TO_FISHEYE_TRANSFORM));

                  baseUI.getLayoutManager().reloadLayout();

                  grayscaleImage = new Mat();
                  calibrationPatternOutput = new Mat();
                  cornersOrCentersMatVector = new MatVector();
                  imagePoints = new Point2fVectorVector();
                  imagePointsMatVector = new MatVector();
                  simpleBlobDetector = SimpleBlobDetector.create();
                  distortionCoefficients = new Mat(distortionCoefficientK1.get(),
                                                   distortionCoefficientK2.get(),
                                                   distortionCoefficientK3.get(),
                                                   distortionCoefficientK4.get());
                  distortionCoefficientsForUndistortion = new SwapReference<>(() -> new Mat(4, 1, opencv_core.CV_64F));
                  distortionCoefficientsForUndistortion.initializeBoth(distortionCoefficients::copyTo);
                  cameraMatrix = new Mat(3, 3, opencv_core.CV_64F);
                  opencv_core.setIdentity(cameraMatrix);
                  cameraMatrix.ptr(0, 0).putDouble(calibratedFxForUndistortion.get());
                  cameraMatrix.ptr(1, 1).putDouble(calibratedFyForUndistortion.get());
                  cameraMatrix.ptr(0, 2).putDouble(calibratedCxForUndistortion.get());
                  cameraMatrix.ptr(1, 2).putDouble(calibratedCyForUndistortion.get());
                  coloringCameraMatrix = new Mat(3, 3, opencv_core.CV_64F);
                  opencv_core.setIdentity(coloringCameraMatrix);
                  coloringCameraMatrix.ptr(0, 0).putDouble(manuallyTunedFxForColoring.get());
                  coloringCameraMatrix.ptr(1, 1).putDouble(manuallyTunedFyForColoring.get());
                  coloringCameraMatrix.ptr(0, 2).putDouble(manuallyTunedCxForColoring.get());
                  coloringCameraMatrix.ptr(1, 2).putDouble(manuallyTunedCyForColoring.get());
                  cameraMatrixForUndistortion = new SwapReference<>(Mat::new);
                  cameraMatrixForUndistortion.initializeBoth(cameraMatrix::copyTo);
                  rectificationTransformation = new Mat(3, 3, opencv_core.CV_64F);
                  opencv_core.setIdentity(rectificationTransformation);
                  newCameraMatrixEstimate = new Mat(3, 3, opencv_core.CV_64F);
                  opencv_core.setIdentity(newCameraMatrixEstimate);
                  cameraMatrixForMonitorShifting = opencv_core.noArray();
                  sourceImageSize = new Size((int) SpinnakerBlackfly.BFLY_U3_23S6C_WIDTH_PIXELS, (int) SpinnakerBlackfly.BFLY_U3_23S6C_HEIGHT_PIXELS);
                  undistortedImageSize = new Size(undistortedImageWidth.get(), undistortedImageHeight.get());
                  imageForUndistortion = new SwapReference<>(Mat::new);
                  spareRGBMatForArUcoDrawing = new Mat(100, 100, opencv_core.CV_8UC3);

                  ThreadTools.startAsDaemon(() ->
                  {
                     while (running)
                     {
                        blackflyReader.readBlackflyImage();
                        calibrationPatternDetectionUI.copyInSourceImage(blackflyReader.getBayerRGImage());

                        if (hdf5ImageLoggingUI != null)
                           hdf5ImageLoggingUI.copyRGBImage(blackflyReader.getBayerRGImage());
                     }
                  }, "CameraRead");
                  ThreadTools.startAsDaemon(() ->
                  {
                     while (running)
                     {
                        undistortionThrottler.waitAndRun();

                        if (imageForUndistortion.getForThreadTwo().rows() > 0 && imageForUndistortion.getForThreadTwo().cols() > 0)
                        {
                           undistortedImageUpdateOnAsynchronousThread(undistortedFisheyePanel.getAsynchronousThreadData());
                           undistortedFisheyePanel.swap();
                        }

                        imageForUndistortion.swap();
                     }
                  }, "Undistortion");
               }

               interactableBlackflyFujinon.update();

               synchronized (cameraMatrixForUndistortion)
               {
                  cameraMatrix.copyTo(cameraMatrixForUndistortion.getForThreadOne());
                  cameraMatrixForUndistortion.getForThreadOne().ptr(0, 0).putDouble(calibratedFxForUndistortion.get());
                  cameraMatrixForUndistortion.getForThreadOne().ptr(1, 1).putDouble(calibratedFyForUndistortion.get());
                  cameraMatrixForUndistortion.getForThreadOne().ptr(0, 2).putDouble(calibratedCxForUndistortion.get());
                  cameraMatrixForUndistortion.getForThreadOne().ptr(1, 2).putDouble(calibratedCyForUndistortion.get());

                  StringBuilder stringBuilder = new StringBuilder();
                  stringBuilder.append("Camera matrix:\n");
                  for (int row = 0; row < 3; row++)
                  {
                     for (int col = 0; col < 3; col++)
                     {
                        stringBuilder.append("%.5f".formatted(cameraMatrixForUndistortion.getForThreadOne().ptr(row, col).getDouble()) + " ");
                     }
                     stringBuilder.append("\n");
                  }
                  cameraMatrixAsText.set(stringBuilder.toString());
               }
               synchronized (distortionCoefficientsForUndistortion)
               {
                  distortionCoefficientsForUndistortion.getForThreadOne().ptr(0, 0).putDouble(distortionCoefficientK1.get());
                  distortionCoefficientsForUndistortion.getForThreadOne().ptr(1, 0).putDouble(distortionCoefficientK2.get());
                  distortionCoefficientsForUndistortion.getForThreadOne().ptr(2, 0).putDouble(distortionCoefficientK3.get());
                  distortionCoefficientsForUndistortion.getForThreadOne().ptr(3, 0).putDouble(distortionCoefficientK4.get());
               }

               calibrationPatternDetectionUI.update();
               blackflyReader.updateOnUIThread();
               hdf5ImageBrowser.update();
               undistortedFisheyePanel.updateTextureAndDrawOnUIThread();
               arUcoMarkerDetectionUI.update();

               if (calibrationSourceImageDrawRequest.poll())
               {
                  calibrationSourceImagesPanel.drawResizeAndCopy(calibrationSourceImages.get(calibrationSourceImageIndex.get()));
               }
               if (calibrationSourceImagePatternDrawRequest.poll())
               {
                  drawPatternOnCurrentImage();
                  calibrationSourceImagesPanel.drawResizeAndCopy(calibrationPatternOutput);
               }

               if (nettyOusterUI.isOusterInitialized())
               {
                  if (nettyOusterUI.getImagePanel() == null)
                  {
                     nettyOusterUI.createAfterOusterInitialized();

                     baseUI.getPrimaryScene().addRenderableProvider(nettyOusterUI::getRenderables);
                     baseUI.getImGuiPanelManager().addPanel(nettyOusterUI.getImagePanel().getImagePanel());
                     baseUI.getLayoutManager().reloadLayout();
                  }

                  nettyOusterUI.update();
               }
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void blackflyReaderUIThreadPreprocessor(RDXImagePanelTexture texture)
         {
            if (hdf5ImageLoggingUI == null)
            {
               hdf5ImageLoggingUI = new RDXHDF5ImageLoggingUI(nativesLoadedActivator,
                                                              blackflyReader.getImageWidth(),
                                                              blackflyReader.getImageHeight());
               baseUI.getImGuiPanelManager().addPanel(hdf5ImageLoggingUI.getPanel());
               baseUI.getLayoutManager().reloadLayout();
            }

            // Access the image before it gets drawn on; copy for the other thread
            synchronized (imageForUndistortion)
            {
               texture.getRGBA8Mat().copyTo(imageForUndistortion.getForThreadOne());
            }

            if (nettyOusterUI.getIsReady())
            {
               nettyOusterUI.setFisheyeImageToColorPoints(texture.getRGBA8Image(),
                                                          manuallyTunedFxForColoring.get(),
                                                          manuallyTunedFyForColoring.get(),
                                                          manuallyTunedCxForColoring.get(),
                                                          manuallyTunedCyForColoring.get());
               nettyOusterUI.getSensorFrame().getReferenceFrame()
                            .getTransformToDesiredFrame(nettyOusterUI.getOusterFisheyeKernel().getOusterToFisheyeTransformToPack(),
                                                        blackflySensorFrame);
            }

            calibrationPatternDetectionUI.drawCornersOrCenters(texture.getRGBA8Mat());
         }

         private void undistortedImageUpdateOnAsynchronousThread(RDXImagePanelTexture texture)
         {
            int width = undistortedImageWidth.get();
            int height = undistortedImageHeight.get();
            undistortedImageSize.width(width);
            undistortedImageSize.height(height);
            texture.ensureTextureDimensions(width, height);

            opencv_calib3d.fisheyeEstimateNewCameraMatrixForUndistortRectify(cameraMatrixForUndistortion.getForThreadTwo(),
                                                                             distortionCoefficientsForUndistortion.getForThreadTwo(),
                                                                             sourceImageSize,
                                                                             rectificationTransformation,
                                                                             newCameraMatrixEstimate,
                                                                             balanceNewFocalLength.get(),
                                                                             undistortedImageSize,
                                                                             fovScaleFocalLengthDivisor.get());

            StringBuilder stringBuilder = new StringBuilder();
            stringBuilder.append("Camera matrix for undistortion:\n");
            for (int row = 0; row < 3; row++)
            {
               for (int col = 0; col < 3; col++)
               {
                  stringBuilder.append("%.5f".formatted(newCameraMatrixEstimate.ptr(row, col).getDouble()) + " ");
               }
               stringBuilder.append("\n");
            }
            newCameraMatrixAsText.set(stringBuilder.toString());

            // Fisheye undistortion
            // https://docs.opencv.org/4.6.0/db/d58/group__calib3d__fisheye.html#ga167df4b00a6fd55287ba829fbf9913b9
            opencv_calib3d.fisheyeUndistortImage(imageForUndistortion.getForThreadTwo(),
                                                 texture.getRGBA8Mat(),
                                                 cameraMatrixForUndistortion.getForThreadTwo(),
                                                 distortionCoefficientsForUndistortion.getForThreadTwo(),
                                                 newCameraMatrixEstimate,
                                                 undistortedImageSize);

            // TODO: We need to switch to using these. You only need to do
            //   initUndistortRectifyMap once, then remap is fast
            //   opencv_calib3d.initUndistortRectifyMap();
            //   opencv_calib3d.remap();
            //   Also, we need to finish https://github.com/bytedeco/javacpp-presets/issues/1185
            //   in order to do this.

            newCameraMatrixEstimate.copyTo(openCVArUcoMarkerDetection.getCameraMatrix());
            openCVArUcoMarkerDetection.update(texture.getRGBA8Image());

            opencv_imgproc.cvtColor(texture.getRGBA8Mat(), spareRGBMatForArUcoDrawing, opencv_imgproc.COLOR_RGBA2RGB);
            openCVArUcoMarkerDetection.drawDetectedMarkers(spareRGBMatForArUcoDrawing);
            openCVArUcoMarkerDetection.drawRejectedPoints(spareRGBMatForArUcoDrawing);
            opencv_imgproc.cvtColor(spareRGBMatForArUcoDrawing, texture.getRGBA8Mat(), opencv_imgproc.COLOR_RGB2RGBA);

            cameraMatrixForUndistortion.swap();
            distortionCoefficientsForUndistortion.swap();
         }

         @Override
         public void dispose()
         {
            running = false;
            nettyOusterUI.destroy();
            blackflyReader.dispose();
            hdf5ImageBrowser.destroy();
            hdf5ImageLoggingUI.destroy();
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
      userChangedUndistortParameters |= ImGuiTools.volatileInputDouble(labels.get("Calibrate Fx (px)"), calibratedFxForUndistortion, 100.0, 500.0, "%.5f");
      userChangedUndistortParameters |= ImGuiTools.volatileInputDouble(labels.get("Calibrate Fy (px)"), calibratedFyForUndistortion, 100.0, 500.0, "%.5f");
      userChangedUndistortParameters |= ImGuiTools.volatileInputDouble(labels.get("Calibrate Cx (px)"), calibratedCxForUndistortion, 100.0, 500.0, "%.5f");
      userChangedUndistortParameters |= ImGuiTools.volatileInputDouble(labels.get("Calibrate Cy (px)"), calibratedCyForUndistortion, 100.0, 500.0, "%.5f");

      userChangedUndistortParameters |= ImGuiTools.volatileInputDouble(labels.get("Distortion K1"), distortionCoefficientK1, 0.0001, 0.001, "%.7f");
      userChangedUndistortParameters |= ImGuiTools.volatileInputDouble(labels.get("Distortion K2"), distortionCoefficientK2, 0.0001, 0.001, "%.7f");
      userChangedUndistortParameters |= ImGuiTools.volatileInputDouble(labels.get("Distortion K3"), distortionCoefficientK3, 0.0001, 0.001, "%.7f");
      userChangedUndistortParameters |= ImGuiTools.volatileInputDouble(labels.get("Distortion K4"), distortionCoefficientK4, 0.0001, 0.001, "%.7f");

      ImGui.sliderScalar(labels.get("New focal length (balance)"), ImGuiDataType.Double, balanceNewFocalLength, 0.0, 1.0, "%.5f");
      ImGuiTools.volatileInputDouble(labels.get("Focal length divisor (FOV scale)"), fovScaleFocalLengthDivisor, 0.01, 0.1, "%.5f");
      ImGuiTools.volatileInputInt(labels.get("Undistorted image width"), undistortedImageWidth);
      ImGuiTools.volatileInputInt(labels.get("Undistorted image height"), undistortedImageHeight);

      boolean userChangedColoringMatrixParameters = false;
      userChangedColoringMatrixParameters |= ImGuiTools.sliderDouble(labels.get("Coloring Fx (px)"), manuallyTunedFxForColoring, -100.0, 800.0 , "%.5f");
      userChangedColoringMatrixParameters |= ImGuiTools.sliderDouble(labels.get("Coloring Fy (px)"), manuallyTunedFyForColoring, -100.0, 800.0 , "%.5f");
      userChangedColoringMatrixParameters |= ImGuiTools.sliderDouble(labels.get("Coloring Cx (px)"), manuallyTunedCxForColoring, -100.0, 1200.0, "%.5f");
      userChangedColoringMatrixParameters |= ImGuiTools.sliderDouble(labels.get("Coloring Cy (px)"), manuallyTunedCyForColoring, -1000.0, 1200.0, "%.5f");
      if (userChangedColoringMatrixParameters)
      {
         coloringCameraMatrix.ptr(0, 0).putDouble(manuallyTunedFxForColoring.get());
         coloringCameraMatrix.ptr(1, 1).putDouble(manuallyTunedFyForColoring.get());
         coloringCameraMatrix.ptr(0, 2).putDouble(manuallyTunedCxForColoring.get());
         coloringCameraMatrix.ptr(1, 2).putDouble(manuallyTunedCyForColoring.get());
      }
      StringBuilder stringBuilder = new StringBuilder();
      stringBuilder.append("Fisheye camera matrix for point coloring:\n");
      for (int row = 0; row < 3; row++)
      {
         for (int col = 0; col < 3; col++)
         {
            stringBuilder.append("%.5f".formatted(coloringCameraMatrix.ptr(row, col).getDouble()) + " ");
         }
         stringBuilder.append("\n");
      }
      coloringCameraMatrixAsText.set(stringBuilder.toString());

      ImGui.text("Fisheye matrix:");
      ImGui.inputTextMultiline(labels.getHidden("fisheyeCameraMatrix"), coloringCameraMatrixAsText, 0, 60, ImGuiInputTextFlags.ReadOnly);
      ImGui.text("Camera matrix:");
      ImGui.inputTextMultiline(labels.getHidden("cameraMatrix"), cameraMatrixAsText, 0, 60, ImGuiInputTextFlags.ReadOnly);
      ImGui.text("New camera matrix:");
      ImGui.inputTextMultiline(labels.getHidden("newCameraMatrix"), newCameraMatrixAsText, 0, 60, ImGuiInputTextFlags.ReadOnly);
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
            RDXCalibrationPatternType pattern = calibrationPatternDetectionUI.getPatternType();
            int patternWidth = calibrationPatternDetectionUI.getPatternWidth();
            int patternHeight = calibrationPatternDetectionUI.getPatternHeight();
            Size patternSize = new Size(patternWidth, patternHeight);

            opencv_imgproc.cvtColor(calibrationSourceImages.get(i), grayscaleImage, opencv_imgproc.COLOR_BGR2GRAY);

            Mat cornersOrCentersMat = new Mat();
            if (pattern == RDXCalibrationPatternType.CHESSBOARD)
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

      opencv_core.setIdentity(cameraMatrix);
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
      averageReprojectionError = opencv_calib3d.fisheyeCalibrate(objectPointsMatVector,
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

      calibratedFxForUndistortion.set(cameraMatrix.ptr(0, 0).getDouble());
      calibratedFyForUndistortion.set(cameraMatrix.ptr(1, 1).getDouble());
      calibratedCxForUndistortion.set(cameraMatrix.ptr(0, 2).getDouble());
      calibratedCyForUndistortion.set(cameraMatrix.ptr(1, 2).getDouble());

      distortionCoefficientK1.set(distortionCoefficients.ptr(0, 0).getDouble());
      distortionCoefficientK2.set(distortionCoefficients.ptr(1, 0).getDouble());
      distortionCoefficientK3.set(distortionCoefficients.ptr(2, 0).getDouble());
      distortionCoefficientK4.set(distortionCoefficients.ptr(3, 0).getDouble());
   }

   public static void main(String[] args)
   {
      new RDXBlackflyCalibrationSuite();
   }
}
