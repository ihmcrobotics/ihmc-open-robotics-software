package us.ihmc.ihmcPerception.lineSegmentDetector;

import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.*;
import org.bytedeco.opencv.opencv_imgproc.*;
import static org.bytedeco.opencv.global.opencv_core.*;
import static org.bytedeco.opencv.global.opencv_imgcodecs.imread;
import static org.bytedeco.opencv.global.opencv_imgproc.*;

import java.util.ArrayList;

public class LineDetectionTools
{
   public static Vec4iVector getCannyHoughLinesFromImage(Mat image)
   {
      Mat gray = new Mat(image); // start avoiding extra allocation here
      cvtColor(image, gray, COLOR_BGR2GRAY);
      Mat edges = new Mat(gray);
      Canny(gray, edges, 60, 60 * 3, 3, true);
      Vec4iVector lines = new Vec4iVector();
      HoughLinesP(edges, lines, 1, Math.PI / 180, 70, 30, 10);
      return lines;
   }

   public static Vec4iVector getLinesFromImage(Mat image)
   {
      Mat frame = preProcessImage(image);
      Mat edges = getCannyEdges(frame);
      return getHoughLinesFromEdges(edges);
   }

   public static Mat preProcessImage(Mat frame)
   {
      Mat gray = new Mat();
      //        Imgproc.pyrDown(frame,frame);
      cvtColor(frame, gray, COLOR_BGR2GRAY);
      medianBlur(gray, gray, 5);
      GaussianBlur(gray, gray, new Size(3, 3), 20);
      return gray;
   }

   public static Mat getCannyEdges(Mat gray)
   {
      Mat edges = gray.clone();
      Canny(gray, edges, 60, 60 * 3, 3, true);
      Mat cannyColor = edges.clone();
      cvtColor(edges, cannyColor, COLOR_GRAY2BGR);
      return edges;
   }

   public static Vec4iVector getHoughLinesFromEdges(Mat edges)
   {
      Vec4iVector lines = new Vec4iVector();
      HoughLinesP(edges, lines, 1, Math.PI / 180, 70, 30, 10);
      return lines;
   }

   public static void displayLineMatches(Mat prevImg, Mat curImg, Mat dispImage, ArrayList<LineSegmentMatch> correspLines)
   {
      MatVector src = new MatVector(prevImg, curImg);
      hconcat(src, dispImage);
      for (LineSegmentMatch lm : correspLines)
      {
//         line(dispImage,
//                      new Point((int)lm.previousLineSegment[0], (int)lm.previousLineSegment[1]),
//                      new Point((int)lm.currentLineSegment[0] + prevImg.cols(), (int)lm.currentLineSegment[1]),
//                      Scalar.YELLOW,
//                      2,LINE_8, 0);
//         line(dispImage,
//                      new Point((int)lm.previousLineSegment[2], (int)lm.previousLineSegment[3]),
//                      new Point((int)lm.currentLineSegment[2] + prevImg.cols(), (int)lm.currentLineSegment[3]),
//                      Scalar.YELLOW,
//                      2, LINE_8, 0);
      }
   }

   public static Mat getImage(String path)
   {
      return imread(path);
   }

   public static void main(String[] args)
   {
//      System.loadLibrary("opencv_java310");
//      FastLineDetector fld = Ximgproc.createFastLineDetector(20, 1.41421356f, 50, 50, 3, true);
   }
}
