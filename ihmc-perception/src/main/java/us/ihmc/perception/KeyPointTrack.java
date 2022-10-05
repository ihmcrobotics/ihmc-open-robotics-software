package us.ihmc.perception;

import org.bytedeco.opencv.opencv_core.KeyPoint;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.tuple2D.Point2D;

import java.util.ArrayList;

public class KeyPointTrack
{
   private Mat headDescriptor;
   private ArrayList<KeyPoint> tail;

   private int totalKeypoints = 0;

   public KeyPointTrack(KeyPoint head, Mat headDescriptor)
   {
      tail.add(head);
      this.headDescriptor = headDescriptor;
   }

   public void pushHead(KeyPoint kp, Mat desc)
   {
      tail.add(kp);
      headDescriptor = desc;
   }

   public Point2D predict()
   {
      Point2D kp;
      if (tail.size() > 1)
      {

         kp = new Point2D(tail.get(1).pt().x() + 2 * (tail.get(0).pt().x() - tail.get(1).pt().x()),
                          tail.get(1).pt().y() + 2 * (tail.get(0).pt().y() - tail.get(1).pt().y()));
         return kp;
      }
      else
         return new Point2D();
   }
}
