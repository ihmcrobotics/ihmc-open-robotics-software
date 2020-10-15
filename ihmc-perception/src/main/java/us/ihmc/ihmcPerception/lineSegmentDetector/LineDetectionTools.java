package us.ihmc.ihmcPerception.lineSegmentDetector;

import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.javacv.*;
import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.indexer.*;
import org.bytedeco.opencv.opencv_core.*;
import org.bytedeco.opencv.opencv_imgproc.*;
import org.bytedeco.opencv.opencv_calib3d.*;
import org.bytedeco.opencv.opencv_objdetect.*;
import static org.bytedeco.opencv.global.opencv_core.*;
import static org.bytedeco.opencv.global.opencv_imgcodecs.imread;
import static org.bytedeco.opencv.global.opencv_imgproc.*;
import static org.bytedeco.opencv.global.opencv_calib3d.*;
import static org.bytedeco.opencv.global.opencv_objdetect.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class LineDetectionTools
{
   public static Vec4iVector getCannyHoughLinesFromImage(Mat image)
   {
      Vec4iVector lines = new Vec4iVector();
      Mat gray = new Mat();
      cvtColor(image, gray, COLOR_BGR2GRAY);

      Mat cannyEdges = new Mat();

      cannyEdges = getCannyEdges(gray);

      lines = getHoughLinesFromEdges(cannyEdges);


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
      Mat edges = new Mat();
      Canny(gray, edges, 60, 60 * 3, 3, true);
      Mat cannyColor = new Mat();
      cvtColor(edges, cannyColor, COLOR_GRAY2BGR);
      return edges;
   }

   public static Vec4iVector getHoughLinesFromEdges(Mat edges)
   {
      Vec4iVector lines = new Vec4iVector();
      HoughLinesP(edges, lines, 1, Math.PI / 180, 70, 30, 10);
      return lines;
   }

   public static void displayLineMatches(Mat prevImg, Mat curImg, Mat dispImage, ArrayList<LineMatch> correspLines)
   {
      MatVector src = new MatVector(prevImg, curImg);
      hconcat(src, dispImage);
      for (LineMatch lm : correspLines)
      {
         line(dispImage,
                      new Point((int)lm.prevLine[0], (int)lm.prevLine[1]),
                      new Point((int)lm.curLine[0] + prevImg.cols(), (int)lm.curLine[1]),
                      Scalar.YELLOW,
                      2,LINE_8, 0);
         line(dispImage,
                      new Point((int)lm.prevLine[2], (int)lm.prevLine[3]),
                      new Point((int)lm.curLine[2] + prevImg.cols(), (int)lm.curLine[3]),
                      Scalar.YELLOW,
                      2, LINE_8, 0);
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
