package us.ihmc.ihmcPerception.blob;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import javax.imageio.ImageIO;
import javax.vecmath.Point2i;
import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import boofcv.gui.image.ImagePanel;
import boofcv.gui.image.ShowImages;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;
import us.ihmc.tools.time.Timer;

/**
 * Taken from https://github.com/Itseez/opencv/blob/master/samples/android/color-blob-detection/src/org/opencv/samples/colorblobdetect/ColorBlobDetector.java
 */
public class ColorBlobDetector
{
   static
   {
      NativeLibraryLoader.loadLibrary("org.opencv", "opencv_java2411");
   }

   public static final Point3d TIGA_ORANGE_PING_PONG_BALL = new Point3d(55, 71, 95);
   public static final HueSaturationValueRange TIGA_ORANGE_PING_PONG_BALL_HSV_RANGE = new HueSaturationValueRange(25, 50, 150, 255, 150, 255);
   public static final int TIGA_ORANGE_PING_PONG_BALL_SIZE = 5;
   
   // Lower and Upper bounds for range checking in HSV color space
   private Scalar mLowerBound = new Scalar(0);
   private Scalar mUpperBound = new Scalar(0);
   // Minimum contour area in percent for contours filtering
   private static double mMinContourArea = 0.1;
   // Color radius for range checking in HSV color space
   private Scalar mColorRadius = new Scalar(25, 50, 50, 0);
   private Mat mSpectrum = new Mat();
   private List<MatOfPoint> mContours = new ArrayList<MatOfPoint>();

   // Cache
   Mat mPyrDownMat = new Mat();
   Mat mHsvMat = new Mat();
   Mat mMask = new Mat();
   Mat mDilatedMask = new Mat();
   Mat mHierarchy = new Mat();
   
   public static Mat convertBufferedImageToHSV(BufferedImage bufferedImage)
   {
      Mat matImage;
      byte[] pixelBuffer;
      switch (bufferedImage.getType())
      {
      case BufferedImage.TYPE_3BYTE_BGR:
         matImage = new Mat(bufferedImage.getHeight(), bufferedImage.getWidth(), CvType.CV_8UC3);
         pixelBuffer = ((DataBufferByte) (bufferedImage.getRaster().getDataBuffer())).getData();
         matImage.put(0, 0, pixelBuffer);
         convertImageFromBGRToHSV(matImage, matImage);
         return matImage;
      case BufferedImage.TYPE_4BYTE_ABGR:
      case BufferedImage.TYPE_INT_ARGB:
         //need to double check endianess
         matImage = new Mat(bufferedImage.getHeight(), bufferedImage.getWidth(), CvType.CV_8UC4);
         pixelBuffer = new byte[bufferedImage.getHeight() * bufferedImage.getWidth() * 4];
         ByteBuffer pixelBuf = ByteBuffer.wrap(pixelBuffer);
         int[] intBuf = bufferedImage.getRGB(0, 0, bufferedImage.getWidth(), bufferedImage.getHeight(), null, 0, bufferedImage.getWidth());
         pixelBuf.asIntBuffer().put(intBuf);
         matImage.put(0, 0, pixelBuffer);
         convertImageFromRGBToHSV(matImage, matImage);
         return matImage;
      default:
         throw new RuntimeException("Unknown type: " + bufferedImage.getType());
      }
   }
   
   public static void convertImageFromBGRToHSV(Mat bgrImage, Mat hsvResult)
   {
      Imgproc.cvtColor(bgrImage, hsvResult, Imgproc.COLOR_BGR2HSV_FULL);
   }
   
   public static void convertImageFromRGBToHSV(Mat rgbImage, Mat hsvResult)
   {
      Imgproc.cvtColor(rgbImage, hsvResult, Imgproc.COLOR_RGB2HSV_FULL);
   }
   
   public static void thresholdImage(Mat hsvImage, Mat thresholdedImage, HueSaturationValueRange hsvRange)
   {
      Core.inRange(hsvImage, hsvRange.getMin(), hsvRange.getMax(), thresholdedImage);
   }
   
   public static void morphologicallyOpen(Mat hsvImage, int size)
   {
      Imgproc.erode(hsvImage, hsvImage, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(size, size)));
      Imgproc.dilate(hsvImage, hsvImage, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(size, size)));
   }
   
   public static void morphologicallyClose(Mat hsvImage, int size)
   {
      Imgproc.dilate(hsvImage, hsvImage, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(size, size)));
      Imgproc.erode(hsvImage, hsvImage, Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(size, size)));
   }
   
   public static Point2i findBlobFromThresholdImage(Mat thresholdedImage)
   {
      Moments moments = Imgproc.moments(thresholdedImage);
      
      double m01 = moments.get_m01();
      double m10 = moments.get_m10();
      double area = moments.get_m00();
      
      Point2i blobPosition = new Point2i((int) (m10 / area), (int) (m01 / area));
      
      return blobPosition;
   }
   
   /**
    * Taken from: http://opencv-srf.blogspot.com/2010/09/object-detection-using-color-seperation.html
    */
   public static Point2i findBlob(BufferedImage bufferedImage, HueSaturationValueRange hsvRange, int size)
   {
      Mat image = convertBufferedImageToHSV(bufferedImage);
      thresholdImage(image, image, hsvRange);
      morphologicallyOpen(image, size);
      morphologicallyClose(image, size);
      return findBlobFromThresholdImage(image);
   }
   
   public static Point2i findBlob(Mat image, HueSaturationValueRange hsvRange, int size)
   {
      convertImageFromRGBToHSV(image, image);
      thresholdImage(image, image, hsvRange);
      morphologicallyOpen(image, size);
      morphologicallyClose(image, size);
      return findBlobFromThresholdImage(image);
   }

   public void setColorRadius(Scalar radius)
   {
      mColorRadius = radius;
   }

   public void setHsvColor(Tuple3d hsvColor)
   {
      double minH = (hsvColor.getX() >= mColorRadius.val[0]) ? hsvColor.getX() - mColorRadius.val[0] : 0;
      double maxH = (hsvColor.getX() + mColorRadius.val[0] <= 255) ? hsvColor.getX() + mColorRadius.val[0] : 255;

      mLowerBound.val[0] = minH;
      mUpperBound.val[0] = maxH;

      mLowerBound.val[1] = hsvColor.getY() - mColorRadius.val[1];
      mUpperBound.val[1] = hsvColor.getY() + mColorRadius.val[1];

      mLowerBound.val[2] = hsvColor.getZ() - mColorRadius.val[2];
      mUpperBound.val[2] = hsvColor.getZ() + mColorRadius.val[2];

      mLowerBound.val[3] = 0;
      mUpperBound.val[3] = 255;

      Mat spectrumHsv = new Mat(1, (int) (maxH - minH), CvType.CV_8UC3);

      for (int j = 0; j < maxH - minH; j++)
      {
         byte[] tmp = { (byte) (minH + j), (byte) 255, (byte) 255 };
         spectrumHsv.put(0, j, tmp);
      }

      Imgproc.cvtColor(spectrumHsv, mSpectrum, Imgproc.COLOR_HSV2RGB_FULL, 4);
   }

   public Mat getSpectrum()
   {
      return mSpectrum;
   }

   public void setMinContourArea(double area)
   {
      mMinContourArea = area;
   }

   public List<MatOfPoint> process(Mat rgbaImage)
   {
      Imgproc.pyrDown(rgbaImage, mPyrDownMat);
      Imgproc.pyrDown(mPyrDownMat, mPyrDownMat);

      Imgproc.cvtColor(mPyrDownMat, mHsvMat, Imgproc.COLOR_RGB2HSV_FULL);

      Core.inRange(mHsvMat, mLowerBound, mUpperBound, mMask);
      Imgproc.dilate(mMask, mDilatedMask, new Mat());

      List<MatOfPoint> contours = new ArrayList<MatOfPoint>();

      Imgproc.findContours(mDilatedMask, contours, mHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

      // Find max contour area
      double maxArea = 0;
      Iterator<MatOfPoint> each = contours.iterator();
      while (each.hasNext())
      {
         MatOfPoint wrapper = each.next();
         double area = Imgproc.contourArea(wrapper);
         if (area > maxArea)
            maxArea = area;
      }

      // Filter contours by area and resize to fit the original image size
      mContours.clear();
      each = contours.iterator();
      while (each.hasNext())
      {
         MatOfPoint contour = each.next();
         if (Imgproc.contourArea(contour) > mMinContourArea * maxArea)
         {
            Core.multiply(contour, new Scalar(4, 4), contour);
            mContours.add(contour);
         }
      }
      
//      List<Point3d> pointList = new ArrayList<Point3d>();
//      for (MatOfPoint points : mContours)
//      {
//         for (int i = 0; i < points.height(); i++)
//         {
//            pointList.add(new Point3d(points.get(i, 0),(double) points.get(i, 1),(double) points.get(i, 2)));
//         }
//      }
      
      PrintTools.info("mContours.size(): " + mContours.size());
      
      return mContours;
   }

   public List<MatOfPoint> getContours()
   {
      return mContours;
   }
   
   public static void main(String[] args) throws IOException
   {
      VideoCapture videoCapture = new VideoCapture(0);
      ImagePanel imagePanel = null;
      Mat image = new Mat();
      MatOfByte matOfByte = new MatOfByte();
      Timer blobTimer = new Timer().start();
      HueSaturationValueRange hsvRange = ColorBlobDetector.TIGA_ORANGE_PING_PONG_BALL_HSV_RANGE;
      int size = ColorBlobDetector.TIGA_ORANGE_PING_PONG_BALL_SIZE;
      
      while (true)
      {
         videoCapture.read(image);

         
         Highgui.imencode(".bmp", image, matOfByte);
         BufferedImage bufferedImage = ImageIO.read(new ByteArrayInputStream(matOfByte.toArray()));
         
         blobTimer.lap();
         
         convertImageFromBGRToHSV(image, image);
         thresholdImage(image, image, hsvRange);
         morphologicallyOpen(image, size);
         morphologicallyClose(image, size);
         Point2i ballLocation = findBlobFromThresholdImage(image);
         
         PrintTools.info("Blob time: " + blobTimer.lapElapsed() + " Blob location: " + ballLocation);
         
         Graphics2D g2d = bufferedImage.createGraphics();
         g2d.setStroke(new BasicStroke(3));
         g2d.setColor(Color.BLUE);
         g2d.drawOval(ballLocation.x, ballLocation.y, 5, 5);
         g2d.dispose();
         
         if (imagePanel == null)
         {
            imagePanel = ShowImages.showWindow(bufferedImage, "faces");
         }
         else
         {
            imagePanel.setBufferedImageSafe(bufferedImage);
         }
      }
   }
}