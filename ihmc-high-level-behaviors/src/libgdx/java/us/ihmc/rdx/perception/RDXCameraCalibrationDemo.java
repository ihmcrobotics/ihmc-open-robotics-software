package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.type.ImBoolean;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.*;
import org.bytedeco.opencv.opencv_core.*;
import org.bytedeco.opencv.opencv_text.FloatVector;
import org.bytedeco.opencv.opencv_videoio.VideoCapture;
import org.bytedeco.opencv.opencv_videoio.VideoWriter;
import perception_msgs.msg.dds.BigVideoPacket;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXImagePanelTexture;
import us.ihmc.rdx.ui.graphics.RDXOpenCVGuidedSwapVideoPanel;
import us.ihmc.rdx.imgui.ImPlotFrequencyPlot;
import us.ihmc.rdx.imgui.ImPlotIntegerPlot;
import us.ihmc.rdx.imgui.ImPlotStopwatchPlot;
import us.ihmc.tools.thread.Throttler;

import java.io.File;
import java.util.function.Consumer;

/**
 * This is an old class that is around for reference only. {@link RDXBlackflyCalibrationSuite} is the new one.
 */
public class RDXCameraCalibrationDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final RDXPanel diagnosticPanel = new RDXPanel("Diagnostics", this::renderImGuiWidgets);
   private VideoCapture videoCapture;
   private int imageHeight = 1080;
   private int imageWidth = 1920;
   private double requestedFPS = 30.0;
   private double reportedFPS = 30.0;
   private String backendName = "";
   private Mat bgrImage;
   private BytePointer jpegImageBytePointer;
   private Mat yuv420Image;
   private RDXOpenCVGuidedSwapVideoPanel swapCVPanel;
   private final Consumer<RDXImagePanelTexture> accessOnHighPriorityThread = this::generateNewCameraMatrixOnUIThread;
   private final ImPlotStopwatchPlot readDurationPlot = new ImPlotStopwatchPlot("Read Duration");
   private final ImPlotStopwatchPlot encodeDurationPlot = new ImPlotStopwatchPlot("Encode Duration");
   private final ImPlotFrequencyPlot readFrequencyPlot = new ImPlotFrequencyPlot("Read Frequency");
   private final ImPlotFrequencyPlot encodeFrequencyPlot = new ImPlotFrequencyPlot("Encode Frequency");
   private final ImPlotIntegerPlot compressedBytesPlot = new ImPlotIntegerPlot("Compressed bytes");
   private final Stopwatch threadOneDuration = new Stopwatch();
   private final Stopwatch threadTwoDuration = new Stopwatch();
   private final BigVideoPacket videoPacket = new BigVideoPacket();
   private IntPointer compressionParameters;

   private final Object measurementSyncObject = new Object();

   private final Point2fVectorVector imagePoints = new Point2fVectorVector();
   private Mat cameraMatrix;
   private Mat distortionCoefficients = new Mat();
   private final MatVector rvecs = new MatVector();
   private final MatVector tvecs = new MatVector();
   private final FloatVector reprojectionErrors = new FloatVector();
   private double totalAvgError;
   private Mat newObjectPoints;
   private float gridWidth;
   private boolean releaseObject;
   private Size imageSize;
   private final Size boardSize = new Size(8, 11);
   private final int squareSize = 13;

   private enum CalibrationPattern
   {NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID}

   private final CalibrationPattern calibrationPattern = CalibrationPattern.CIRCLES_GRID;
   private Point3fVectorVector objectPoints;

   private final Size patternSize = new Size(9, 6); //width, height
   private final Mat matOfCorners = new Mat();
   private final Point2fVector tempPoint2fVector = new Point2fVector();
   private final Mat tempMat = new Mat();
   private RDXBytedecoImagePanel undistortedVideoPanel;
   private RDXBytedecoImagePanel testImagePanel;
   private Mat imageAtIndex = null;
   private final MatVector images = new MatVector();
   private Mat newCameraMatrix = new Mat();
   private final String calibrationPhotoDirectory = "ihmc-open-robotics-software/ihmc-high-level-behaviors/src/libgdx/resources/configurations/cameraCalibrationPhotos/";
   private File dir;
   private int currentNumberOfImagesInDirectory = 0;
   private final ImBoolean takingPhotosIsActive = new ImBoolean(false);
   private final Size winSize = new Size(11, 11); //Half of search window for cornerSubPix
   private final Size zeroZone = new Size(-1, -1);
   private final TermCriteria termCriteria = new TermCriteria(TermCriteria.EPS + TermCriteria.COUNT, 30, 0.0001);
   private final Throttler throttler = new Throttler();
   private final Notification generateCameraMatrixNotification = new Notification();

   public RDXCameraCalibrationDemo()
   {
      baseUI.getImGuiPanelManager().addPanel(diagnosticPanel);
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            videoCapture = new VideoCapture(0);

            int reportedImageWidth = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_WIDTH);
            int reportedImageHeight = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_HEIGHT);
            reportedFPS = videoCapture.get(opencv_videoio.CAP_PROP_FPS);

            LogTools.info("Default resolution: {} x {}", reportedImageWidth, reportedImageHeight);
            LogTools.info("Default fps: {}", reportedFPS);

            backendName = videoCapture.getBackendName().getString();

            videoCapture.set(opencv_videoio.CAP_PROP_FRAME_WIDTH, imageWidth);
            videoCapture.set(opencv_videoio.CAP_PROP_FRAME_HEIGHT, imageHeight);
            videoCapture.set(opencv_videoio.CAP_PROP_FOURCC, VideoWriter.fourcc((byte) 'M', (byte) 'J', (byte) 'P', (byte) 'G'));
            videoCapture.set(opencv_videoio.CAP_PROP_FPS, requestedFPS);

            imageWidth = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_WIDTH);
            imageHeight = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_HEIGHT);
            reportedFPS = videoCapture.get(opencv_videoio.CAP_PROP_FPS);
            LogTools.info("Format: {}", videoCapture.get(opencv_videoio.CAP_PROP_FORMAT));

            bgrImage = new Mat();
            yuv420Image = new Mat();
            jpegImageBytePointer = new BytePointer();

            swapCVPanel = new RDXOpenCVGuidedSwapVideoPanel("Video", this::videoUpdateOnAsynchronousThread, this::videoUpdateOnUIThread);
            undistortedVideoPanel = new RDXBytedecoImagePanel("Undistorted Video", imageWidth, imageHeight);
            testImagePanel = new RDXBytedecoImagePanel("Test Image 1", imageWidth, imageHeight);
            baseUI.getImGuiPanelManager().addPanel(swapCVPanel.getImagePanel());
            baseUI.getImGuiPanelManager().addPanel(undistortedVideoPanel.getImagePanel());
            baseUI.getImGuiPanelManager().addPanel(testImagePanel.getImagePanel());

            compressionParameters = new IntPointer(opencv_imgcodecs.IMWRITE_JPEG_QUALITY, 75);

            ThreadTools.startAsDaemon(() ->
            {
               while (true)
               {
                  // If encoding is running slower than reading, we want to wait a little so the read
                  // is freshest going into the encode and publish. We do this because we don't want
                  // to there to be more than one publish thread going
                  boolean shouldSleep;
                  double sleepTime;
                  synchronized (measurementSyncObject)
                  {
                     double averageThreadTwoDuration = threadTwoDuration.averageLap();
                     double averageThreadOneDuration = threadOneDuration.averageLap();
                     shouldSleep = !Double.isNaN(averageThreadTwoDuration) && !Double.isNaN(averageThreadOneDuration)
                                   && averageThreadTwoDuration > averageThreadOneDuration;
                     sleepTime = averageThreadTwoDuration - averageThreadOneDuration;

                     if (Double.isNaN(threadOneDuration.lap()))
                        threadOneDuration.reset();
                  }

                  if (shouldSleep)
                     ThreadTools.sleepSeconds(sleepTime);

                  readDurationPlot.start();
                  boolean imageWasRead = videoCapture.read(bgrImage);
                  readDurationPlot.stop();
                  readFrequencyPlot.ping();

                  if (!imageWasRead)
                  {
                     LogTools.error("Image was not read!");
                  }

                  // Convert colors are pretty fast. Encoding is slow, so let's do it in parallel.

                  swapCVPanel.updateOnAsynchronousThread();

                  opencv_imgproc.cvtColor(bgrImage, yuv420Image, opencv_imgproc.COLOR_BGR2YUV_I420);

                  synchronized (measurementSyncObject)
                  {
                     threadOneDuration.suspend();
                  }
               }
            }, "CameraRead");

            dir = new File(calibrationPhotoDirectory);
            File[] directoryListing = dir.listFiles();
            if (directoryListing != null)
            {
               for (File child : directoryListing)
               {
                  images.push_back(new Mat(opencv_imgcodecs.imread(child.getAbsolutePath())));
                  //LogTools.info("{}", child.getAbsolutePath());
               }
            }
            else
            {
               LogTools.info("Not a directory");
            }
            LogTools.info("There are {} many photos", images.size());
            currentNumberOfImagesInDirectory = (int) images.size();
         }

         @Override
         public void dispose()
         {
            videoCapture.release();
            baseUI.dispose();
         }

         @Override
         public void render()
         {
            swapCVPanel.updateOnUIThread();
            testImagePanel.draw();
            undistortedVideoPanel.draw();
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void videoUpdateOnAsynchronousThread(RDXImagePanelTexture texture)
         {
            texture.ensureTextureDimensions(imageWidth, imageHeight);
            opencv_imgproc.cvtColor(bgrImage, texture.getRGBA8Mat(), opencv_imgproc.COLOR_BGR2RGBA, 0);
         }

         private void videoUpdateOnUIThread(RDXImagePanelTexture texture)
         {
            if (generateCameraMatrixNotification.poll())
            {
               generateNewCameraMatrixOnUIThread(texture);
            }

            if (cameraMatrix != null)
            {
               newCameraMatrix = opencv_calib3d.getOptimalNewCameraMatrix(cameraMatrix, distortionCoefficients, imageSize, 1.0);

               texture.getRGBA8Image().getBytedecoOpenCVMat().copyTo(tempMat);
               opencv_calib3d.undistort(tempMat,
                                        undistortedVideoPanel.getBytedecoImage().getBytedecoOpenCVMat(),
                                        cameraMatrix,
                                        distortionCoefficients,
                                        newCameraMatrix);
            }

            if (takingPhotosIsActive.get() && throttler.run(0.5))
            {

               opencv_imgproc.cvtColor(texture.getRGBA8Image().getBytedecoOpenCVMat(), tempMat, opencv_imgproc.COLOR_BGR2RGB);
               opencv_imgcodecs.imwrite(calibrationPhotoDirectory + "CameraCalibrationPhoto" + (currentNumberOfImagesInDirectory /*+1*/) + ".jpg", tempMat);
               currentNumberOfImagesInDirectory++;
            }

            texture.updateTextureAndDraw(swapCVPanel.getImagePanel());
         }
      });
   }

   private void renderImGuiWidgets()
   {
      ImGui.text("Is open: " + videoCapture.isOpened());
      ImGui.text("Image dimensions: " + imageWidth + " x " + imageHeight);
      ImGui.text("Reported fps: " + reportedFPS);
      ImGui.text("Backend name: " + backendName);
      readFrequencyPlot.renderImGuiWidgets();
      encodeFrequencyPlot.renderImGuiWidgets();
      readDurationPlot.renderImGuiWidgets();
      encodeDurationPlot.renderImGuiWidgets();
      compressedBytesPlot.renderImGuiWidgets();
      ImGui.text(("There are currently {} photos" + currentNumberOfImagesInDirectory));
      if (ImGui.checkbox("Record photos for calibration", takingPhotosIsActive))
      {

      }
      if (ImGui.button("Generate camera matrix"))
      {
         generateCameraMatrixNotification.set();
      }
   }

   private void generateNewCameraMatrixOnUIThread(RDXImagePanelTexture texture)
   {
      LogTools.info("PATH {}", dir.getAbsolutePath());
      File[] directoryListing = dir.listFiles();
      if (directoryListing != null)
      {
         for (int childIndex = currentNumberOfImagesInDirectory; childIndex < directoryListing.length; childIndex++)
         {
            File child = directoryListing[childIndex];
            images.push_back(new Mat(opencv_imgcodecs.imread(child.getAbsolutePath())));
            //LogTools.info("{}", child.getAbsolutePath());
         }
      }
      else
      {
         LogTools.info("Not a directory");
      }
      LogTools.info("There are {} many photos", images.size());
      //numOfImagesInDirectory = (int) images.size();

      if (texture != null && texture.getRGBA8Image() != null && texture.getRGBA8Image().getBytedecoOpenCVMat() != null)
      {
         texture.getRGBA8Image().getBytedecoOpenCVMat().copyTo(undistortedVideoPanel.getBytedecoImage().getBytedecoOpenCVMat());
         undistortedVideoPanel.getBytedecoImage().getBytedecoOpenCVMat().copyTo(tempMat);
         gridWidth = (float) squareSize * (boardSize.width() - 1);

         for (int imageIndex = 0; imageIndex < images.size(); imageIndex++)
         {
            imageAtIndex = images.get(imageIndex);
            if (imageAtIndex.channels() == 3)
            {
               opencv_imgproc.cvtColor(imageAtIndex, imageAtIndex, opencv_imgproc.COLOR_BGR2GRAY);
            }
            imageSize = imageAtIndex.size(); //VIDEO OR IMAGE size???

            boolean found = opencv_calib3d.findChessboardCorners(imageAtIndex,
                                                                 patternSize,
                                                                 matOfCorners,
                                                                 opencv_calib3d.CALIB_CB_ADAPTIVE_THRESH | opencv_calib3d.CALIB_CB_NORMALIZE_IMAGE);

            if (found)
            {
               if (calibrationPattern == CalibrationPattern.CHESSBOARD)
               {
                  opencv_imgproc.cornerSubPix(imageAtIndex, matOfCorners, winSize, zeroZone, termCriteria);
               }

               tempPoint2fVector.clear();
               for (int x = 0; x < matOfCorners.cols(); x++)
               {
                  for (int y = 0; y < matOfCorners.rows(); y++)
                  {
                     tempPoint2fVector.push_back(new Point2f(matOfCorners.ptr(y, x)));
                  }
               }
               imagePoints.clear();
               imagePoints.push_back(tempPoint2fVector);
               runCalibrationAndSave();
            }
         }

         // For testing, show an image with the ChessboardCorners drawn
         if (cameraMatrix != null)
         {
            imageAtIndex = images.get(20);

            boolean found = opencv_calib3d.findChessboardCorners(imageAtIndex,
                                                                 patternSize,
                                                                 matOfCorners,
                                                                 opencv_calib3d.CALIB_CB_ADAPTIVE_THRESH | opencv_calib3d.CALIB_CB_NORMALIZE_IMAGE);
            if (found)
            {
               opencv_calib3d.drawChessboardCorners(imageAtIndex, patternSize, matOfCorners, found);
               opencv_imgproc.cvtColor(imageAtIndex, testImagePanel.getBytedecoImage().getBytedecoOpenCVMat(), opencv_imgproc.COLOR_GRAY2RGBA);
            }
         }
      }
   }

   /*
   private double computeReprojectionErrors()
   {
      Point2fVector imagePoints2 = new Point2fVector();
      Mat matImagePoints2 = new Mat(imagePoints2);
      int totalPoints = 0;
      double totalErr = 0, err;
      reprojectionErrors.resize(objectPoints.size());

      for(int i = 0; i < objectPoints.size(); ++i )
      {


         projectPoints(new Mat(objectPoints.get(i)), rvecs.get(i), tvecs.get(i), cameraMatrix, distortionCoefficients, matImagePoints2); //Currently trying to debug

         err = norm(new Mat(imagePoints.get(i)), NORM_L2, new Mat(imagePoints2));

         long n = objectPoints.get(i).size();
         reprojectionErrors.put(i, (float) sqrt(err * err / n) ); //Not sure if this is actually being put at i
         totalErr += err*err;
         totalPoints += n;
      }

      return sqrt(totalErr/totalPoints);
   }
*/

   private void calcBoardCornerPositions()
   {
      //cornersOfPattern.clear();
      objectPoints.get(0).clear();

      switch (calibrationPattern)
      {
         case CHESSBOARD:
         case CIRCLES_GRID:
            for (int y = 0; y < boardSize.height(); y++)
            {
               for (int x = 0; x < boardSize.width(); x++)
               {
                  objectPoints.get(0).push_back(new Point3f(x * squareSize, y * squareSize, 0));
               }
            }
            break;
         case ASYMMETRIC_CIRCLES_GRID:
            for (int y = 0; y < boardSize.height(); y++)
            {
               for (int x = 0; x < boardSize.width(); x++)
               {
                  objectPoints.get(0).push_back(new Point3f((2 * x + y % 2) * squareSize, y * squareSize, 0));
               }
            }
      }
   }

   private boolean runCalibration()
   {
      cameraMatrix = Mat.eye(3, 3, opencv_core.CV_64F).asMat();
      distortionCoefficients = Mat.zeros(8, 1, opencv_core.CV_64F).asMat();
      objectPoints = new Point3fVectorVector(1);

      calcBoardCornerPositions();
      objectPoints.get(0).get(boardSize.width() - 1).x(objectPoints.get(0).get(0).x() + gridWidth); // ???
      newObjectPoints = new Mat(objectPoints);

      objectPoints.resize(imagePoints.size()); //Docs resized it by (imagePoints.size(), objectPoints[0])

      opencv_calib3d.calibrateCamera(objectPoints,
                                     imagePoints,
                                     imageSize,
                                     cameraMatrix,
                                     distortionCoefficients,
                                     rvecs,
                                     tvecs,
                                     0,
                                     new TermCriteria(TermCriteria.EPS + TermCriteria.COUNT, 30, 0.0001));

      boolean inRange = opencv_core.checkRange(cameraMatrix) && opencv_core.checkRange(distortionCoefficients);

      //totalAvgError = computeReprojectionErrors();

      return inRange;
   }

   public boolean runCalibrationAndSave()
   {
      totalAvgError = 0;

      boolean isCalibrated = runCalibration();

      //LogTools.info("Calibration {}", isCalibrated? "succeeded": "failed");

      return isCalibrated;
   }

   public static void main(String[] args)
   {
      new RDXCameraCalibrationDemo();
   }
}