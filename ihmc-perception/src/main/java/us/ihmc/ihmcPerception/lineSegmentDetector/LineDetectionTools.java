package us.ihmc.ihmcPerception.lineSegmentDetector;

import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.ximgproc.FastLineDetector;
import org.opencv.ximgproc.Ximgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class LineDetectionTools
{
   public static Mat getFLDLinesFromImage(Mat image)
   {
      Mat lines = new Mat();
      Mat gray = new Mat();
      Imgproc.cvtColor(image, gray, Imgproc.COLOR_BGR2GRAY);

      FastLineDetector fld = Ximgproc.createFastLineDetector(30, 1.41421356f, 50, 50, 3, true);
      fld.detect(gray, lines);
      return lines;
   }

   public static Mat getLinesFromImage(Mat image)
   {
      Mat frame = preProcessImage(image);
      Mat edges = getCannyEdges(frame);
      return getHoughLinesFromEdges(edges);
   }

   public static Mat preProcessImage(Mat frame)
   {
      Mat gray = new Mat();
      //        Imgproc.pyrDown(frame,frame);
      Imgproc.cvtColor(frame, gray, Imgproc.COLOR_BGR2GRAY);
      Imgproc.medianBlur(gray, gray, 5);
      Imgproc.GaussianBlur(gray, gray, new Size(3, 3), 20);
      return gray;
   }

   public static Mat getCannyEdges(Mat gray)
   {
      Mat edges = new Mat();
      Imgproc.Canny(gray, edges, 60, 60 * 3, 3, true);
      Mat cannyColor = new Mat();
      Imgproc.cvtColor(edges, cannyColor, Imgproc.COLOR_GRAY2BGR);
      return edges;
   }

   public static Mat getHoughLinesFromEdges(Mat edges)
   {
      Mat lines = new Mat();
      Imgproc.HoughLinesP(edges, lines, 1, Math.PI / 180, 70, 30, 10);
      return lines;
   }

   public static void displayLineMatches(Mat prevImg, Mat curImg, Mat dispImage, ArrayList<LineMatch> correspLines)
   {
      List<Mat> src = Arrays.asList(prevImg, curImg);
      Core.hconcat(src, dispImage);
      for (LineMatch lm : correspLines)
      {
         Imgproc.line(dispImage,
                      new Point(lm.prevLine[0], lm.prevLine[1]),
                      new Point(lm.curLine[0] + prevImg.cols(), lm.curLine[1]),
                      new Scalar(255, 50, 50),
                      2);
         Imgproc.line(dispImage,
                      new Point(lm.prevLine[2], lm.prevLine[3]),
                      new Point(lm.curLine[2] + prevImg.cols(), lm.curLine[3]),
                      new Scalar(255, 50, 50),
                      2);
      }
   }

   public static Mat getImage(String path)
   {
      return Imgcodecs.imread(path);
   }
}
