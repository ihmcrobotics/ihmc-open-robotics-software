package us.ihmc.ihmcPerception.vision.shapes;

import boofcv.gui.image.ImagePanel;
import org.opencv.core.*;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.ihmcPerception.OpenCVTools;
import us.ihmc.ihmcPerception.vision.HSVValue;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

import javax.swing.*;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class OpenCVColoredCircularBlobDetector
{
   static
   {
      NativeLibraryLoader.loadLibrary("org.opencv", OpenCVTools.OPEN_CV_LIBRARY_NAME);
   }

   private final String videoFileName;
   private final int cameraIndex;
   private final CaptureSource captureSource;

   private final VideoCapture videoCapture;
   private final Map<HSVRange, Mat> rangeOutputMaterials = new HashMap<>();

   private final Mat tmpMat = new Mat();
   private final Mat thresholdMat = new Mat();
   private final Mat currentCameraFrameMatInBGR = new Mat();
   private final Mat currentCameraFrameMatInHSV = new Mat();
   private final Mat medianBlurredMat = new Mat();
   private final Mat houghCirclesOutputMat = new Mat();

   private final ArrayList<HoughCircleResult> circles = new ArrayList<>();
   private static Dimension imagePanelDimension;
   private static ImagePanel colorImagePanel;
   private static ImagePanel filterImagePanel;

   private static volatile boolean dirty = true;

   public enum CaptureSource
   {
      CAMERA, FILE, JAVA_BUFFERED_IMAGES
   }

   /* package private */ OpenCVColoredCircularBlobDetector(String videoFileName, int cameraIndex, CaptureSource captureSource)
   {
      this.videoFileName = videoFileName;
      this.cameraIndex = cameraIndex;
      this.captureSource = captureSource;

      switch (captureSource)
      {
      case CAMERA:
         videoCapture = new VideoCapture(cameraIndex);
         break;
      case FILE:
         videoCapture = new VideoCapture(videoFileName);
         break;
      default:
         videoCapture = null;
         break;
      }
   }

   public void resetRanges()
   {
      rangeOutputMaterials.clear();
   }

   public void addHSVRange(HSVRange range)
   {
      if(!rangeOutputMaterials.containsKey(range))
      {
         rangeOutputMaterials.put(range, new Mat());
      }
   }

   public ArrayList<HoughCircleResult> getCircles()
   {
      return circles;
   }

   public Mat getCurrentCameraFrameMatInBGR()
   {
      return currentCameraFrameMatInBGR;
   }

   public Mat getThresholdMat()
   {
      return thresholdMat;
   }

   public Mat getTmpMat()
   {
      return tmpMat;
   }

   public Mat getMedianBlurredMat()
   {
      return medianBlurredMat;
   }

   public void updateFromVideo()
   {
      switch (captureSource)
      {
      case CAMERA:
      case FILE:

         boolean updateSuccessful = updateCurrentFrameMaterialFromVideoCapture();

         if (!updateSuccessful)
         {
            break;
         }

         updateListOfCircles();

         break;
      default:
         throw new RuntimeException("Cannot process video stream when in CaptureSource.JAVA_BUFFERED_IMAGES mode!");
      }
   }

   public void updateFromBufferedImage(BufferedImage bufferedImage)
   {
      switch (captureSource)
      {
      case JAVA_BUFFERED_IMAGES:
         OpenCVTools.convertBufferedImageToMat(bufferedImage).copyTo(currentCameraFrameMatInBGR);

         updateListOfCircles();
         break;
      default:
         throw new RuntimeException("Cannot process BufferedImage when capturing from video stream or video file!");
      }
   }

   private boolean updateCurrentFrameMaterialFromVideoCapture()
   {
      if(videoCapture == null || !videoCapture.isOpened())
      {
         System.err.println("Video capture stream isn't open! Cannot update OpenCV Material for processing!");
         return false;
      }

      boolean readSuccessful = videoCapture.read(tmpMat);
      if(!readSuccessful)
      {
         System.err.println("Reading from video capture stream failed, cannot update OpenCV Material for processing!");
         return false;
      }

      tmpMat.copyTo(currentCameraFrameMatInBGR);
      return true;
   }

   private void updateListOfCircles()
   {
      Imgproc.cvtColor(currentCameraFrameMatInBGR, currentCameraFrameMatInHSV, Imgproc.COLOR_BGR2HSV);
      Imgproc.medianBlur(currentCameraFrameMatInHSV, medianBlurredMat, 5);

      if(rangeOutputMaterials.isEmpty())
      {
         System.err.println("No HSV ranges to check against. Set at least 1 range with addHSVRange()");
         return;
      }

      int count = 0;
      for (Map.Entry<HSVRange, Mat> hsvRangeMatEntry : rangeOutputMaterials.entrySet())
      {
         HSVRange hsvRange = hsvRangeMatEntry.getKey();
         Mat mat = hsvRangeMatEntry.getValue();

         Core.inRange(medianBlurredMat, hsvRange.getLowerBoundOpenCVScalar(), hsvRange.getUpperBoundOpenCVScalar(), mat);

         Imgproc.erode(mat, mat, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(20, 20)));
         Imgproc.dilate(mat, mat, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(20, 20)));

         Imgproc.dilate(mat, mat, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(20, 20)));
         Imgproc.erode(mat, mat, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(20, 20)));

         if(count == 0)
         {
            mat.copyTo(thresholdMat);
         }
         else
         {
            Core.addWeighted(thresholdMat, 1.0, mat, 1.0, 0.0, thresholdMat);
         }
         count++;
      }

      Imgproc.GaussianBlur(thresholdMat, thresholdMat, new Size(13, 13), 2, 2);

      Imgproc.HoughCircles(thresholdMat, houghCirclesOutputMat, Imgproc.CV_HOUGH_GRADIENT, 1, thresholdMat.rows() / 8, 100, 15, 0, 0);

      circles.clear();
      for (int i = 0; i < houghCirclesOutputMat.cols(); i++)
      {
         double[] circleData = houghCirclesOutputMat.get(0, i);
         Point2D circleCenter = new Point2D(circleData[0], circleData[1]);
         double circleRadius = circleData[2];

         circles.add(new HoughCircleResult(circleCenter, circleRadius));
      }
   }

   public static void main(String[] args)
   {
      OpenCVColoredCircularBlobDetectorFactory factory = new OpenCVColoredCircularBlobDetectorFactory();

      factory.setCameraIndex(0);
      factory.setCaptureSource(CaptureSource.CAMERA);
      OpenCVColoredCircularBlobDetector openCVColoredCircularBlobDetector = factory.buildBlobDetector();

      HSVRange greenRange = new HSVRange(new HSVValue(55, 80, 80), new HSVValue(139, 255, 255));
//      HSVRange brightRedRange = new HSVRange(new HSVValue(120, 80, 80), new HSVValue(179, 255, 255));
//      HSVRange dullRedRange = new HSVRange(new HSVValue(3, 80, 80), new HSVValue(10, 255, 255));
      HSVRange yellowRange = new HSVRange(new HSVValue(25, 100, 100), new HSVValue(40, 255, 255));

      openCVColoredCircularBlobDetector.addHSVRange(greenRange);
//      openCVColoredCircularBlobDetector.addHSVRange(brightRedRange);
//      openCVColoredCircularBlobDetector.addHSVRange(dullRedRange);
      openCVColoredCircularBlobDetector.addHSVRange(yellowRange);

      openCVColoredCircularBlobDetector.updateFromVideo();

      JFrame frame = new JFrame("OpenCV Colored Circular Blob Detector");
      frame.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);

      final Container rootPane = frame.getContentPane();
      rootPane.setLayout(new BoxLayout(rootPane, BoxLayout.PAGE_AXIS));

      setupImagePanels(openCVColoredCircularBlobDetector, rootPane);

      JPanel slidersPanel = new JPanel();
      slidersPanel.setLayout(new BoxLayout(slidersPanel, BoxLayout.LINE_AXIS));

      final HSVValue lowerBound = new HSVValue(0, 0, 0);
      final HSVValue upperBound = new HSVValue(0, 0, 0);

      final Dimension colorDimension = new Dimension(100, 100);

      final JPanel lowerBoundColorIndicationPanel = new JPanel();
      lowerBoundColorIndicationPanel.setBackground(Color.red);
      lowerBoundColorIndicationPanel.setPreferredSize(colorDimension);

      final JPanel upperBoundColorIndicationPanel = new JPanel();
      upperBoundColorIndicationPanel.setBackground(Color.red);
      upperBoundColorIndicationPanel.setPreferredSize(colorDimension);

      JPanel lowerBoundPanel = setupHSVPanel("HSV Lower Bound", lowerBound, lowerBoundColorIndicationPanel);
      JPanel upperBoundPanel = setupHSVPanel("HSV Upper Bound", upperBound, upperBoundColorIndicationPanel);

      slidersPanel.add(lowerBoundPanel);
      slidersPanel.add(upperBoundPanel);

      rootPane.add(slidersPanel);

      frame.pack();
      frame.setVisible(true);

      Scalar circleColor = new Scalar(160, 0, 0);

      while (true)
      {
         if(!frame.isVisible() && openCVColoredCircularBlobDetector.videoCapture != null)
         {
            openCVColoredCircularBlobDetector.videoCapture.release();
            break;
         }

         if(dirty)
         {
            openCVColoredCircularBlobDetector.rangeOutputMaterials.clear();
            openCVColoredCircularBlobDetector.rangeOutputMaterials.put(new HSVRange(lowerBound, upperBound), new Mat());
            dirty = false;
         }

         final Color lowerBoundBackgroundColor = OpenCVTools.convertHSVValueToColor(lowerBound);
         final Color upperBoundBackgroundColor = OpenCVTools.convertHSVValueToColor(upperBound);

         SwingUtilities.invokeLater(new Runnable()
         {
            @Override
            public void run()
            {
               lowerBoundColorIndicationPanel.setBackground(lowerBoundBackgroundColor);
               upperBoundColorIndicationPanel.setBackground(upperBoundBackgroundColor);
               rootPane.repaint();
            }
         });

         openCVColoredCircularBlobDetector.updateFromVideo();

         ArrayList<HoughCircleResult> circles = openCVColoredCircularBlobDetector.getCircles();

         for (HoughCircleResult circle : circles)
         {
            Point openCVPoint = new Point(circle.getCenter().getX(), circle.getCenter().getY());
            Imgproc.circle(openCVColoredCircularBlobDetector.getCurrentCameraFrameMatInBGR(), openCVPoint, (int) circle.getRadius(), circleColor, 1);
            Imgproc.circle(openCVColoredCircularBlobDetector.getThresholdMat(), openCVPoint, (int) circle.getRadius(), circleColor, 1);
         }

         colorImagePanel.setBufferedImageSafe(OpenCVTools.convertMatToBufferedImage(openCVColoredCircularBlobDetector.getCurrentCameraFrameMatInBGR()));
         filterImagePanel.setBufferedImageSafe(OpenCVTools.convertMatToBufferedImage(openCVColoredCircularBlobDetector.getThresholdMat()));
      }

      System.exit(0);
   }

   private static JPanel setupHSVPanel(String panelTitle, final HSVValue hsvValue, JPanel colorIndicationPanel)
   {
      JPanel boundPanel = new JPanel();
      boundPanel.setLayout(new FlowLayout());
      boundPanel.setBorder(BorderFactory.createTitledBorder(panelTitle));

      JPanel slidersPanel = new JPanel();
      slidersPanel.setLayout(new BoxLayout(slidersPanel, BoxLayout.PAGE_AXIS));

      final JSlider hueSlider = new JSlider(JSlider.HORIZONTAL, 0, 180, 0);
      final JLabel hueLabel = new JLabel("Hue: " + hueSlider.getValue());
      hueSlider.setMajorTickSpacing(85);
      hueSlider.setPaintTicks(true);
      hueSlider.setPaintLabels(true);
      hueSlider.addChangeListener(new ChangeListener()
      {
         @Override
         public void stateChanged(ChangeEvent e)
         {
            hueLabel.setText("Hue: " + hueSlider.getValue());
            hsvValue.setHue(hueSlider.getValue());
            dirty = true;
         }
      });

      final JSlider saturationSlider = new JSlider(JSlider.HORIZONTAL, 0, 255, 0);
      final JLabel saturationLabel = new JLabel("Sat: " + saturationSlider.getValue());
      saturationSlider.setMajorTickSpacing(85);
      saturationSlider.setPaintTicks(true);
      saturationSlider.setPaintLabels(true);
      saturationSlider.addChangeListener(new ChangeListener()
      {
         @Override
         public void stateChanged(ChangeEvent e)
         {
            saturationLabel.setText("Sat: " + saturationSlider.getValue());
            hsvValue.setSaturation(saturationSlider.getValue());
            dirty = true;
         }
      });

      final JSlider valueSlider = new JSlider(JSlider.HORIZONTAL, 0, 255, 0);
      final JLabel valueLabel = new JLabel("Val: " + valueSlider.getValue());
      valueSlider.setMajorTickSpacing(85);
      valueSlider.setPaintTicks(true);
      valueSlider.setPaintLabels(true);
      valueSlider.addChangeListener(new ChangeListener()
      {
         @Override
         public void stateChanged(ChangeEvent e)
         {
            valueLabel.setText("Val: " + valueSlider.getValue());
            hsvValue.setBrightnessValue(valueSlider.getValue());
            dirty = true;
         }
      });

      slidersPanel.add(hueLabel);
      slidersPanel.add(hueSlider);

      slidersPanel.add(saturationLabel);
      slidersPanel.add(saturationSlider);

      slidersPanel.add(valueLabel);
      slidersPanel.add(valueSlider);

      boundPanel.add(slidersPanel);
      boundPanel.add(colorIndicationPanel);

      return boundPanel;
   }

   private static void setupImagePanels(OpenCVColoredCircularBlobDetector openCVColoredCircularBlobDetector, Container rootPane)
   {
      JPanel wrapperPanel = new JPanel();
      wrapperPanel.setLayout(new BoxLayout(wrapperPanel, BoxLayout.LINE_AXIS));
      imagePanelDimension = new Dimension(openCVColoredCircularBlobDetector.currentCameraFrameMatInBGR.width() / 2, openCVColoredCircularBlobDetector.currentCameraFrameMatInBGR.height() / 2);
      colorImagePanel = new ImagePanel();
      filterImagePanel = new ImagePanel();

      colorImagePanel.setPreferredSize(imagePanelDimension);
      colorImagePanel.setMaximumSize(imagePanelDimension);
      colorImagePanel.setMinimumSize(imagePanelDimension);

      filterImagePanel.setPreferredSize(imagePanelDimension);
      filterImagePanel.setMaximumSize(imagePanelDimension);
      filterImagePanel.setMinimumSize(imagePanelDimension);

      wrapperPanel.add(colorImagePanel);
      wrapperPanel.add(filterImagePanel);

      rootPane.add(wrapperPanel);
   }
}
