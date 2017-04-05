package us.ihmc.robotics.trajectories;

import us.ihmc.euclid.tuple2D.Point2D;

public interface CurveGenerator
{
   public abstract Point2D getPointGivenX(double xValue);

   public abstract double getDerivative(double xValue);

   public abstract double getXmin();

   public abstract double getXmax();

   public abstract Point2D[] getArrayOfPoints(int numberOfPointsToReturn);

   public abstract void setArrayOfPoints(Point2D[] newArrayOfPoints);
}
