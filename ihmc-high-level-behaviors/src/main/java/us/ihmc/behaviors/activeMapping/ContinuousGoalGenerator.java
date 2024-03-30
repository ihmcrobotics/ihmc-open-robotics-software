package us.ihmc.behaviors.activeMapping;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;

import java.util.Random;

public class ContinuousGoalGenerator
{
   private double boundaryMinX = 0.0;
   private double boundaryMaxX = 5.0;
   private double boundaryMinY = 0.0;
   private double boundaryMaxY = 5.0;

   private Random random = new Random(System.nanoTime());

   private Point2D previousPosition = new Point2D();
   private Point2D currentPosition = new Point2D();
   private Point2D goalPosition = new Point2D();

   private Vector2D currentVelocity = new Vector2D();

   public ContinuousGoalGenerator(double boundaryMinX, double boundaryMaxX, double boundaryMinY, double boundaryMaxY)
   {
      this.boundaryMinX = boundaryMinX;
      this.boundaryMaxX = boundaryMaxX;
      this.boundaryMinY = boundaryMinY;
      this.boundaryMaxY = boundaryMaxY;
   }

   public void updateCurrentPosition(Point3D currentPosition)
   {
      previousPosition.set(this.currentPosition);
      this.currentPosition.set(currentPosition);
      computeVelocity();
   }

   public void computeVelocity()
   {
      currentVelocity.sub(currentPosition, previousPosition);
   }

   public Point2D getNextLocation(double margin)
   {
      goalPosition.set(currentPosition);
      goalPosition.add(currentVelocity);
      goalPosition.add(random.nextDouble(-margin, margin), random.nextDouble(-margin, margin));

      if (goalPosition.getX() < boundaryMinX)
      {
         goalPosition.setX(boundaryMinX);
      }
      else if (goalPosition.getX() > boundaryMaxX)
      {
         goalPosition.setX(boundaryMaxX);
      }

      if (goalPosition.getY() < boundaryMinY)
      {
         goalPosition.setY(boundaryMinY);
      }
      else if (goalPosition.getY() > boundaryMaxY)
      {
         goalPosition.setY(boundaryMaxY);
      }

      return goalPosition;
   }
}
