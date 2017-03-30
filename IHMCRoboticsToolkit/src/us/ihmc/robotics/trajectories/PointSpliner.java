package us.ihmc.robotics.trajectories;

import java.util.ArrayList;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class PointSpliner
{
   private CubicSplineCurveGenerator[] cubicSplineCurveGenerators;
   private ReferenceFrame referenceFrame;

   public PointSpliner(ArrayList<FramePoint> listOfPoints)
   {
      this(listOfPoints, null, null);
   }

   @SuppressWarnings("unchecked")
   public PointSpliner(ArrayList<FramePoint> listOfPoints, FrameVector initialHeading, FrameVector finalHeading)
   {
      if (listOfPoints.size() < 3)
         throw new RuntimeException("List must have at least 3 elements");

      for (int i = 1; i < listOfPoints.size(); i++)
      {
         listOfPoints.get(i).checkReferenceFrameMatch(listOfPoints.get(i - 1));
      }

      referenceFrame = listOfPoints.get(0).getReferenceFrame();

      double headingScale = 6.0;

      if (initialHeading != null)
      {
         initialHeading.checkReferenceFrameMatch(referenceFrame);
         initialHeading.scale(headingScale);
      }

      if (finalHeading != null)
      {
         finalHeading.checkReferenceFrameMatch(referenceFrame);
         finalHeading.scale(headingScale);
      }

      ArrayList<Point2D>[] pointArrays = new ArrayList[3];

      for (int i = 0; i < pointArrays.length; i++)
      {
         pointArrays[i] = new ArrayList<Point2D>();
      }

      double maxIndex = (listOfPoints.size()) - 1.0;
      for (int i = 0; i < listOfPoints.size(); i++)
      {
         double alpha = (i) / maxIndex;
         Point2D point2d;

         point2d = new Point2D(alpha, listOfPoints.get(i).getX());
         pointArrays[0].add(point2d);

         point2d = new Point2D(alpha, listOfPoints.get(i).getY());
         pointArrays[1].add(point2d);

         point2d = new Point2D(alpha, listOfPoints.get(i).getZ());
         pointArrays[2].add(point2d);
      }

      cubicSplineCurveGenerators = new CubicSplineCurveGenerator[3];

      for (int i = 0; i < pointArrays.length; i++)
      {
         cubicSplineCurveGenerators[i] = new CubicSplineCurveGenerator(pointArrays[i]);

         double startDerivative, endDerivative;
         if (initialHeading != null)
         {
            switch (i)
            {
               case 0 :
               {
                  startDerivative = initialHeading.getX();

                  break;
               }

               case 1 :
               {
                  startDerivative = initialHeading.getY();

                  break;
               }

               case 2 :
               {
                  startDerivative = initialHeading.getZ();

                  break;
               }

               default :
                  throw new RuntimeException("It should not get here");
            }
         }
         else
            startDerivative = Double.NaN;


         if (finalHeading != null)
         {
            switch (i)
            {
               case 0 :
               {
                  endDerivative = finalHeading.getX();

                  break;
               }

               case 1 :
               {
                  endDerivative = finalHeading.getY();

                  break;
               }

               case 2 :
               {
                  endDerivative = finalHeading.getZ();

                  break;
               }

               default :
                  throw new RuntimeException("It should not get here");
            }
         }
         else
            endDerivative = Double.NaN;

         cubicSplineCurveGenerators[i].setStartAndEndDerivatives(startDerivative, endDerivative);
      }
   }

   public ArrayList<FramePoint> getSplinedPoints(int numberOfPoints)
   {
      Point2D[][] listOfPoints = new Point2D[3][numberOfPoints];

//    LinearInterpolatorCurveGenerator[] linearInterpolatorCurveGenerator = new LinearInterpolatorCurveGenerator[3];

      for (int i = 0; i < cubicSplineCurveGenerators.length; i++)
      {
         listOfPoints[i] = cubicSplineCurveGenerators[i].getArrayOfPoints(numberOfPoints);

//       linearInterpolatorCurveGenerator[i] = new LinearInterpolatorCurveGenerator(listOfPoints[i]);
      }


      ArrayList<FramePoint> ret = new ArrayList<FramePoint>();

      @SuppressWarnings("unused")
      double maxIndex = listOfPoints[0].length - 1;
      for (int i = 0; i < listOfPoints[0].length; i++)
      {
//       double alpha = ((double) i)/maxIndex;
//       point3d.x = linearInterpolatorCurveGenerator[0].getPointGivenX(alpha).y;
//       point3d.y = linearInterpolatorCurveGenerator[1].getPointGivenX(alpha).y;
//       point3d.z = linearInterpolatorCurveGenerator[2].getPointGivenX(alpha).y;

         // check to make sure S is the same
         if (listOfPoints[0][i].getX() != listOfPoints[1][i].getX())
            throw new RuntimeException(listOfPoints[0][i].getX() + " != " + listOfPoints[1][i].getX());

         if (listOfPoints[1][i].getX() != listOfPoints[2][i].getX())
            throw new RuntimeException(listOfPoints[1][i].getX() + " != " + listOfPoints[2][i].getX());

         FramePoint framePoint = new FramePoint(referenceFrame);

         framePoint.setX(listOfPoints[0][i].getY());
         framePoint.setY(listOfPoints[1][i].getY());
         framePoint.setZ(listOfPoints[2][i].getY());

         ret.add(framePoint);
      }

      return ret;
   }

   public static void main(String[] args)
   {
      ArrayList<FramePoint> listOfPoints = new ArrayList<FramePoint>();
      FramePoint framePoint;

//    framePoint = new FramePoint(ReferenceFrame.getWorldFrame(), 10.0, 4.0, 00.0);
//    listOfPoints.add(framePoint);

//    framePoint = new FramePoint(ReferenceFrame.getWorldFrame(), 12.0, 4.0, 00.0);
//    listOfPoints.add(framePoint);

      framePoint = new FramePoint(ReferenceFrame.getWorldFrame(), 20.0, 10.0, 00.0);
      listOfPoints.add(framePoint);

      framePoint = new FramePoint(ReferenceFrame.getWorldFrame(), 48.0, 4.0, 00.0);
      listOfPoints.add(framePoint);

      framePoint = new FramePoint(ReferenceFrame.getWorldFrame(), 30.0, 0.0, 00.0);
      listOfPoints.add(framePoint);

      FrameVector initialHeading = new FrameVector(ReferenceFrame.getWorldFrame(), 0.0, 1.0, 0.0);
      FrameVector finalHeading = new FrameVector(ReferenceFrame.getWorldFrame(), 0.0, -4.0, 0.0);

      PointSpliner pointSpliner = new PointSpliner(listOfPoints, initialHeading, finalHeading);

//    PointSpliner pointSpliner = new PointSpliner(listOfPoints);

      ArrayList<FramePoint> splinedList = pointSpliner.getSplinedPoints(100);


      Point2D[] checkPoints = new Point2D[splinedList.size()];

      for (int i = 0; i < checkPoints.length; i++)
      {
         Point2D point2d = new Point2D(splinedList.get(i).getX(), splinedList.get(i).getY());
         checkPoints[i] = point2d;
      }

//    PointAndLinePlotter pointAndLinePlotter = new PointAndLinePlotter();
//    pointAndLinePlotter.setAxisSquare(true);
//    pointAndLinePlotter.addPointsWithoutRepaint(checkPoints, Color.BLUE, 5);
//
//    Point2d[] originalPoints = new Point2d[listOfPoints.size()];
//    for(int i=0; i<originalPoints.length; i++)
//    {
//       Point2d point2d = new Point2d(listOfPoints.get(i).getX(), listOfPoints.get(i).getY());
//       originalPoints[i] = point2d;
//    }
//
//    pointAndLinePlotter.addPoints(originalPoints, Color.RED, 9);


   }
}
