package us.ihmc.gdx.perception;

import controller_msgs.msg.dds.BigVideoPacket;
import imgui.ImGui;
import imgui.type.ImBoolean;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.*;
import org.bytedeco.opencv.opencv_core.*;
import org.bytedeco.opencv.opencv_text.FloatVector;
import org.bytedeco.opencv.opencv_videoio.VideoCapture;
import org.bytedeco.opencv.opencv_videoio.VideoWriter;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.graphics.ImGuiOpenCVSwapVideoPanel;
import us.ihmc.gdx.ui.tools.ImPlotFrequencyPlot;
import us.ihmc.gdx.ui.tools.ImPlotIntegerPlot;
import us.ihmc.gdx.ui.tools.ImPlotStopwatchPlot;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.Throttler;

import java.io.File;

import static org.bytedeco.opencv.global.opencv_calib3d.*;
import static org.bytedeco.opencv.global.opencv_core.*;

public class GDXCameraCalibrationDemo
{
   private final Activator nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources");
   private final ImGuiPanel diagnosticPanel = new ImGuiPanel("Diagnostics", this::renderImGuiWidgets);
   private VideoCapture videoCapture;
   private int imageHeight = 1080;
   private int imageWidth = 1920;
   private double requestedFPS = 30.0;
   private double reportedFPS = 30.0;
   private String backendName = "";
   private Mat bgrImage;
   private BytePointer jpegImageBytePointer;
   private Mat yuv420Image;
   private ImGuiOpenCVSwapVideoPanel swapCVPanel;
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





   Point2fVectorVector imagePoints = new Point2fVectorVector();
   Mat cameraMatrix;
   Mat distortionCoefficients = new Mat();
   MatVector rvecs = new MatVector();
   MatVector tvecs = new MatVector();
   FloatVector reprojectionErrors = new FloatVector();
   double totalAvgError;
   Mat newObjectPoints;
   float gridWidth;
   boolean releaseObject;
   Size imageSize;
   Size boardSize = new Size(9,6);
   int squareSize = 13;
   enum CalibrationPattern {NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID}
   CalibrationPattern calibrationPattern = CalibrationPattern.CHESSBOARD;
   Point3fVectorVector objectPoints;

   Size patternSize = new Size(9, 6); //width, height
   Mat matOfCorners = new Mat();
   Point2fVector tempPoint2fVector = new Point2fVector();
   Mat tempMat = new Mat();
   GDXCVImagePanel undistortedVideoPanel;
   GDXCVImagePanel testImagePanel;
   Mat imageAtIndex = null;
   MatVector images = new MatVector();
   boolean firstRun = true;
   Mat newCameraMatrix = new Mat();
   String calibrationPhotoDirectory= "ihmc-open-robotics-software/ihmc-high-level-behaviors/src/libgdx/resources/configurations/cameraCalibrationPhotos/";
   File dir;
   int currentNumberOfImagesInDirectory = 0;
   ImBoolean takingPhotosIsActive = new ImBoolean(false);
   Size winSize = new Size(11, 11); //Half of search window for cornerSubPix
   Size zeroZone = new Size(-1, -1);
   TermCriteria termCriteria = new TermCriteria(TermCriteria.EPS + TermCriteria.COUNT, 30, 0.0001);
   Throttler throttler = new Throttler();


   public GDXCameraCalibrationDemo()
   {

      baseUI.getImGuiPanelManager().addPanel(diagnosticPanel);
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
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
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  videoCapture = new VideoCapture(0);

                  int reportedImageWidth = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_WIDTH);
                  int reportedImageHeight = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_HEIGHT);
                  reportedFPS = videoCapture.get(opencv_videoio.CAP_PROP_FPS);

                  LogTools.info("Default resolution: {} x {}", reportedImageWidth, reportedImageHeight);
                  LogTools.info("Default fps: {}", reportedFPS);

                  backendName = BytedecoTools.stringFromByteBuffer(videoCapture.getBackendName());

                  videoCapture.set(opencv_videoio.CAP_PROP_FRAME_WIDTH, imageWidth);
                  videoCapture.set(opencv_videoio.CAP_PROP_FRAME_HEIGHT, imageHeight);
                  videoCapture.set(opencv_videoio.CAP_PROP_FOURCC, VideoWriter.fourcc((byte) 'M', (byte) 'J', (byte) 'P', (byte) 'G'));
                  videoCapture.set(opencv_videoio.CAP_PROP_FPS, requestedFPS);
                  //                  videoCapture.set(opencv_videoio.CAP_PROP_FRAME_WIDTH, 1280.0);
                  //                  videoCapture.set(opencv_videoio.CAP_PROP_FRAME_HEIGHT, 720.0);

                  imageWidth = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_WIDTH);
                  imageHeight = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_HEIGHT);
                  reportedFPS = videoCapture.get(opencv_videoio.CAP_PROP_FPS);
                  LogTools.info("Format: {}", videoCapture.get(opencv_videoio.CAP_PROP_FORMAT));

                  bgrImage = new Mat();
                  yuv420Image = new Mat();
                  jpegImageBytePointer = new BytePointer();

                  swapCVPanel = new ImGuiOpenCVSwapVideoPanel("Video", false);
                  undistortedVideoPanel = new GDXCVImagePanel("Undistorted Video", imageWidth, imageHeight);
                  testImagePanel = new GDXCVImagePanel("Test Image 1", imageWidth, imageHeight);
                  baseUI.getImGuiPanelManager().addPanel(swapCVPanel.getVideoPanel());
                  baseUI.getImGuiPanelManager().addPanel(undistortedVideoPanel.getVideoPanel());
                  baseUI.getImGuiPanelManager().addPanel(testImagePanel.getVideoPanel());
                  baseUI.getPerspectiveManager().reloadPerspective();

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

                                                  swapCVPanel.getDataSwapReferenceManager().accessOnLowPriorityThread(data ->
                                                                                                                      {
                                                                                                                         data.updateOnImageUpdateThread(imageWidth, imageHeight);
                                                                                                                         opencv_imgproc.cvtColor(bgrImage, data.getRGBA8Mat(), opencv_imgproc.COLOR_BGR2RGBA, 0);
                                                                                                                      });

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




               //Calibration Code
               swapCVPanel.getDataSwapReferenceManager().accessOnHighPriorityThread(data ->
               {
                  if (cameraMatrix != null)
                  {
                     newCameraMatrix = getOptimalNewCameraMatrix(cameraMatrix,
                                                                 distortionCoefficients,
                                                                 imageSize,
                                                                 1.0);

                     data.getBytedecoImage().getBytedecoOpenCVMat().copyTo(tempMat);
                     undistort(tempMat,
                               undistortedVideoPanel.getBytedecoImage().getBytedecoOpenCVMat(),
                               cameraMatrix,
                               distortionCoefficients,
                               newCameraMatrix);


                  }


                  if (takingPhotosIsActive.get() == true && throttler.run(0.5))
                  {

                     opencv_imgproc.cvtColor(data.getBytedecoImage().getBytedecoOpenCVMat(),
                                             tempMat,
                                             opencv_imgproc.COLOR_BGR2RGB);
                     opencv_imgcodecs.imwrite(
                           calibrationPhotoDirectory + "CameraCalibrationPhoto" + (currentNumberOfImagesInDirectory /*+1*/) + ".jpg", tempMat);
                     currentNumberOfImagesInDirectory++;
                  }
               });





               swapCVPanel.getDataSwapReferenceManager().accessOnHighPriorityThread(data ->
                                                                                    {
                                                                                       data.updateOnUIThread(swapCVPanel.getVideoPanel());
                                                                                    });

            }

            testImagePanel.draw();
            undistortedVideoPanel.draw();
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }
      });

   }

   private void renderImGuiWidgets()
   {
      if (nativesLoadedActivator.peek())
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
      }
      ImGui.text(("There are currently {} photos" + currentNumberOfImagesInDirectory));
      if(ImGui.checkbox("Record photos for calibration", takingPhotosIsActive))
      {

      }
      if (ImGui.button("Generate camera matrix"))
      {
         swapCVPanel.getDataSwapReferenceManager().accessOnHighPriorityThread(data ->
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


            if (data != null && data.getBytedecoImage() != null
                && data.getBytedecoImage().getBytedecoOpenCVMat() != null)
            {

               data.getBytedecoImage().getBytedecoOpenCVMat().copyTo(undistortedVideoPanel.getBytedecoImage().getBytedecoOpenCVMat());

               undistortedVideoPanel.getBytedecoImage().getBytedecoOpenCVMat().copyTo(tempMat);


               gridWidth = (float) squareSize * (boardSize.width() - 1);

               for (int imageIndex = 0; imageIndex < images.size(); imageIndex++)
               {

                  imageAtIndex = images.get(imageIndex);
                  if(imageAtIndex.channels()==3)
                  {
                     opencv_imgproc.cvtColor(imageAtIndex, imageAtIndex, opencv_imgproc.COLOR_BGR2GRAY);
                  }
                  imageSize = imageAtIndex.size(); //VIDEO OR IMAGE size???

                  boolean found = findChessboardCorners(imageAtIndex, patternSize, matOfCorners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

                  if (found == true)
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

               //For testing, show an image with the ChessboardCorners drawn
               if (cameraMatrix != null)
               {
                  imageAtIndex = images.get(20);

                  boolean found = findChessboardCorners(imageAtIndex, patternSize, matOfCorners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
                  if (found)
                  {
                     drawChessboardCorners(imageAtIndex, patternSize, matOfCorners, found);
                     opencv_imgproc.cvtColor(imageAtIndex, testImagePanel.getBytedecoImage().getBytedecoOpenCVMat(), opencv_imgproc.COLOR_GRAY2RGBA);
                  }
               }

            }
         });
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

      switch(calibrationPattern)
      {
         case CHESSBOARD:
         case CIRCLES_GRID:
            for(int y = 0; y<boardSize.height(); y++)
            {
               for(int x = 0; x<boardSize.width();x++)
               {
                  objectPoints.get(0).push_back(new Point3f(x*squareSize, y*squareSize, 0));
               }
            }
            break;
         case ASYMMETRIC_CIRCLES_GRID:
            for(int y = 0; y<boardSize.height(); y++)
            {
               for(int x = 0; x<boardSize.width();x++)
               {
                  objectPoints.get(0).push_back(new Point3f((2*x+y%2)*squareSize, y*squareSize, 0));
               }
            }
      }


   }

   private boolean runCalibration()
   {
      cameraMatrix = Mat.eye(3, 3, opencv_core.CV_64F).asMat();

      distortionCoefficients = Mat.zeros(8,1,opencv_core.CV_64F).asMat();

      objectPoints = new Point3fVectorVector(1) ;



      calcBoardCornerPositions();
      objectPoints.get(0).get(boardSize.width()-1).x(objectPoints.get(0).get(0).x() + gridWidth);
      newObjectPoints = new Mat(objectPoints);


      objectPoints.resize(imagePoints.size()); //Docs resized it by (imagePoints.size(), objectPoints[0])

      calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distortionCoefficients, rvecs, tvecs, 0,new TermCriteria( TermCriteria.EPS+TermCriteria.COUNT, 30, 0.0001 ) );

      boolean inRange = checkRange(cameraMatrix) && checkRange(distortionCoefficients);

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
      new GDXCameraCalibrationDemo();
   }

}
