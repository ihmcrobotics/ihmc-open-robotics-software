package us.ihmc.robotics.trajectories;

import java.util.ArrayList;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;

public class ListOfPointsTrajectory
{
   private ArrayList<FramePoint3D> listOfPoints;
   private double lengthOfPath;
   private LinearInterpolater linearInterpolater;
   private double[] alpha;

   public ListOfPointsTrajectory(final ArrayList<FramePoint3D> listOfPointsForSegment)
   {
      if ((listOfPointsForSegment == null) || (listOfPointsForSegment.size() < 2))
      {
         throw new RuntimeException("listOfPoints must have at least 2 elements.");
      }

      this.listOfPoints = new ArrayList<FramePoint3D>();

      // add the start and end point
      this.listOfPoints.add(new FramePoint3D(listOfPointsForSegment.get(0)));

      for (int i = 1; i < listOfPointsForSegment.size(); i++)
      {
         double distanceToPreviousPoint = listOfPointsForSegment.get(i).distance(listOfPointsForSegment.get(i - 1));
         if (distanceToPreviousPoint > 0.0)
            this.listOfPoints.add(new FramePoint3D(listOfPointsForSegment.get(i)));
      }

      // set all the Z's to zero and add to list
//    for()
//    {
//       framePoint.setZ(0.0);
//       this.listOfPoints.add(new FramePoint(framePoint));
//    }

      setupAlphaArrayAndLinearInterpolator();
   }

   public static ListOfPointsTrajectory createListOfPointsTrajectory(ArrayList<Point2D> points, ReferenceFrame referenceFrame)
   {
      ArrayList<FramePoint3D> framePoints = new ArrayList<FramePoint3D>(points.size());

      for (Point2D point : points)
      {
         FramePoint3D framePoint = new FramePoint3D(referenceFrame, point.getX(), point.getY(), 0.0);
         framePoints.add(framePoint);
      }

      return new ListOfPointsTrajectory(framePoints);
   }

   private void setupAlphaArrayAndLinearInterpolator()
   {
      int numberOfPoints = listOfPoints.size();

      alpha = new double[numberOfPoints];
      double[] index = new double[numberOfPoints];
      double[] distanceOnPath = calculateLengthOfPath(listOfPoints);

      this.lengthOfPath = distanceOnPath[distanceOnPath.length - 1];

      if ((numberOfPoints > 1) && (this.lengthOfPath == 0.0))
         throw new RuntimeException("If more than one point, the length of the path must be greater than zero");

      alpha[0] = 0.0;
      index[0] = 0;

      for (int i = 1; i < numberOfPoints; i++)
      {
         alpha[i] = distanceOnPath[i] / lengthOfPath;
         index[i] = i;
      }

      try
      {
         this.linearInterpolater = new LinearInterpolater(alpha, index);
      }
      catch (Exception ex)
      {
         this.linearInterpolater = null;

//       System.out.println(ex);
//       throw new RuntimeException("ListOfPointsTrajectory: error in generating alpha or index");
      }
   }

   public ListOfPointsTrajectory(ListOfPointsTrajectory listOfPointsTrajectory)
   {
      this(listOfPointsTrajectory.getOriginalList());
   }

   public double getPathLength()
   {
      return lengthOfPath;
   }


   public double getAlphaGivenIndexInOriginalList(int index)
   {
      if (index > (alpha.length - 1))
         index = alpha.length - 1;

      return alpha[index];
   }

   public int getSizeOfInternalList()
   {
      return listOfPoints.size();
   }

   public ArrayList<FramePoint3D> getOriginalList()
   {
      return new ArrayList<FramePoint3D>(listOfPoints);
   }


   public ArrayList<FramePointAndAlpha> getOriginalListAndAlphas()
   {
      ArrayList<FramePointAndAlpha> ret = new ArrayList<FramePointAndAlpha>();

      for (int i = 0; i < listOfPoints.size(); i++)
      {
         ret.add(new FramePointAndAlpha(listOfPoints.get(i), alpha[i]));
      }

      return ret;
   }

   public class FramePointAndAlpha
   {
      private final FramePoint3D framePoint;
      private final double alpha;

      FramePointAndAlpha(FramePoint3D framePoint, double alpha)
      {
         this.framePoint = framePoint;
         this.alpha = alpha;
      }

      public FramePoint3D getFramePoint()
      {
         return framePoint;
      }

      public double getAlpha()
      {
         return alpha;
      }
   }


   private double[] calculateLengthOfPath(ArrayList<FramePoint3D> pointsOnPath)
   {
      double[] distanceOnPath = new double[pointsOnPath.size()];
      distanceOnPath[0] = 0.0;

      for (int i = 1; i < pointsOnPath.size(); i++)
      {
         distanceOnPath[i] = distanceOnPath[i - 1] + pointsOnPath.get(i).distance(pointsOnPath.get(i - 1));
      }

      return distanceOnPath;
   }


   public double getAlpha(FramePoint3D pointToCheck)
   {
      double minimumDistance = Double.POSITIVE_INFINITY;
      double alphaToReturn = 1.0;

      double deltaAlpha = 0.01;

      // +++JEP 3/4/2007: Needed to add deltaAlpha/2.0 or it might never get to 1.0
      for (double alpha = 0.0; alpha <= 1.0 + deltaAlpha / 2.0; alpha = alpha + deltaAlpha)
      {
         if (alpha > 1.0)
            alpha = 1.0;

         FramePoint3D testPoint = this.getPointOnPath(alpha);
         double distance = testPoint.distance(pointToCheck);
         if (distance < minimumDistance)
         {
            minimumDistance = distance;
            alphaToReturn = alpha;
         }
      }

      return alphaToReturn;
   }

   public int getIndexOfClosestPoint(FramePoint3D pointToCheck)
   {
      double alpha = getAlpha(pointToCheck);

      return (int) Math.round(getIndexOfClosestAlpha(alpha));
   }


   public double getIndexOfClosestAlpha(double alpha)
   {
      double index;
      if (linearInterpolater != null)
         index = linearInterpolater.getPoint(alpha);
      else
         index = 0.0;

      if (index > listOfPoints.size() - 1)
         index = listOfPoints.size() - 1;
      if (index < 0.0)
         index = 0.0;

      return index;

   }

   public FramePoint3D getPointOnPath(double alpha)
   {
      return getPointOnPath(alpha, false);
   }

   public FramePoint3D getPointOnPathAndAddToInternalList(double alpha)
   {
      return getPointOnPath(alpha, true);
   }


   private FramePoint3D getPointOnPath(double alpha, boolean addToInternalRepresentation)
   {
      if (alpha < 0.0)
         alpha = 0.0;
      if (alpha > 1.0)
         alpha = 1.0;

      double indexDouble;
      if (linearInterpolater != null)
         indexDouble = this.linearInterpolater.getPoint(alpha);
      else
         indexDouble = 0.0;

      int indexBefore = (int) Math.floor(indexDouble);
      int indexAfter = (int) Math.ceil(indexDouble);

      double fractionBetweenPoints = indexDouble - indexBefore;

      FramePoint3D beforePoint = new FramePoint3D(listOfPoints.get(indexBefore));
      FramePoint3D afterPoint = new FramePoint3D(listOfPoints.get(indexAfter));

      FrameVector3D vectorBetweenPoints = new FrameVector3D(beforePoint.getReferenceFrame());
      vectorBetweenPoints.sub(afterPoint, beforePoint);

      if (fractionBetweenPoints == 0.0)
      {
         vectorBetweenPoints.set(0.0, 0.0, 0.0);
      }
      else
      {
         vectorBetweenPoints.scale(fractionBetweenPoints);
      }

      FramePoint3D returnPoint = new FramePoint3D(beforePoint);
      returnPoint.add(vectorBetweenPoints);

      if ((addToInternalRepresentation) && (beforePoint.distance(returnPoint) > 1e-7) && (afterPoint.distance(returnPoint) > 1e-7))
      {
         ArrayList<FramePoint3D> newListOfPoints = new ArrayList<FramePoint3D>();
         for (int i = 0; i <= indexBefore; i++)
         {
            newListOfPoints.add(listOfPoints.get(i));
         }

         newListOfPoints.add(returnPoint);

         for (int i = indexAfter; i < listOfPoints.size(); i++)
         {
            newListOfPoints.add(listOfPoints.get(i));
         }

         listOfPoints = newListOfPoints;
         setupAlphaArrayAndLinearInterpolator();
      }

      return returnPoint;
   }

   public FramePoint3D getPointOnPathDistanceFromStart(double distanceFromStart)
   {
      double pathLength = this.getPathLength();
      if (pathLength < 1e-12)
         return listOfPoints.get(0);

      double alpha = distanceFromStart / pathLength;

      return getPointOnPath(alpha);
   }


   public FramePoint3D getPointOnPathDistanceFromStartAndAddToInternalList(double distanceFromStart)
   {
      double pathLength = this.getPathLength();
      if (pathLength < 1e-12)
         return listOfPoints.get(0);

      double alpha = distanceFromStart / pathLength;

      return getPointOnPathAndAddToInternalList(alpha);
   }




   public static ArrayList<FramePoint3D> expandList(ArrayList<FramePoint3D> listOfPoints, int numberOfPoints)
   {
//    if (numberOfPoints <= listOfPoints.size())
//       return new ArrayList<FramePoint>(listOfPoints);

      ListOfPointsTrajectory listOfPointsTrajectory = new ListOfPointsTrajectory(listOfPoints);
      double percentStepSize = 1.0 / (numberOfPoints - 1.0);

      ArrayList<FramePoint3D> ret = new ArrayList<FramePoint3D>();
      for (double alpha = 0.0; alpha <= 1.0; alpha = alpha + percentStepSize)
      {
         ret.add(listOfPointsTrajectory.getPointOnPath(alpha));
      }

      return ret;
   }


// public void visualizeListOfPointsTrajectory(BagOfBalls bagOfBalls)
// {
//    bagOfBalls.reset();
//
//    for (double alpha = 0.0; alpha < 1.0; alpha = alpha + 0.01)
//    {
//       bagOfBalls.setBall(getPointOnPath(alpha));
//    }
// }

   public static void main(String[] args)
   {
      FramePoint3D framePoint;
      ArrayList<FramePoint3D> list = new ArrayList<FramePoint3D>();

      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();

      framePoint = new FramePoint3D(referenceFrame, 0.0, 0.0, 0.0);
      list.add(framePoint);

//    framePoint = new FramePoint(LittleDogFrames.getWorldFrame(), 1.0, 0.0, 0.0);
//    list.add(framePoint);
//
//    framePoint = new FramePoint(LittleDogFrames.getWorldFrame(), 1.0, 0.0, 0.0);
//    list.add(framePoint);

      ListOfPointsTrajectory listOfPointsTrajectory = null;
      try
      {
         listOfPointsTrajectory = new ListOfPointsTrajectory(list);
      }
      catch (Exception ex)
      {
      }

      for (double alpha = 0.0; alpha <= 1.0; alpha = alpha + 0.1)
      {
         FramePoint3D testPoint = listOfPointsTrajectory.getPointOnPath(alpha);
         System.out.println("testPoint=" + testPoint);
      }

   }


}
