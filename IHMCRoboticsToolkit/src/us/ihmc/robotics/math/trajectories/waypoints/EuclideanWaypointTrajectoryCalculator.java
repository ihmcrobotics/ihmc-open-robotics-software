package us.ihmc.robotics.math.trajectories.waypoints;

import java.util.EnumMap;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class EuclideanWaypointTrajectoryCalculator
{
   private ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();

   private final RecyclingArrayList<FrameEuclideanWaypoint> waypoints = new RecyclingArrayList<>(20, FrameEuclideanWaypoint.class);
   private final EnumMap<Direction, YoPolynomial> polynomials = new EnumMap<>(Direction.class);

   public EuclideanWaypointTrajectoryCalculator()
   {
      clear();
      YoVariableRegistry dummyRegistry = new YoVariableRegistry("Dummy");
      for (Direction direction : Direction.values)
         polynomials.put(direction, new YoPolynomial(StringUtils.lowerCase(direction.toString()) + "Polynomial", 6, dummyRegistry));
   }

   public void clear()
   {
      waypoints.clear();
   }

   public void changeFrame(ReferenceFrame referenceFrame)
   {
      for (int i = 0; i < getNumberOfWaypoints(); i++)
         waypoints.get(i).changeFrame(referenceFrame);
      this.referenceFrame = referenceFrame;
   }

   public void appendWaypoint(EuclideanWaypointInterface<?> waypoint)
   {
      waypoints.add().set(waypoint);
   }

   public void appendWaypoint(Point3d position)
   {
      FrameEuclideanWaypoint newWaypoint = waypoints.add();
      newWaypoint.setToZero(referenceFrame);
      newWaypoint.setTimeToNaN();
      newWaypoint.setPosition(position);
      newWaypoint.setLinearVelocityToNaN();
   }

   public void appendWaypoint(double time, Point3d position)
   {
      FrameEuclideanWaypoint newWaypoint = waypoints.add();
      newWaypoint.setToZero(referenceFrame);
      newWaypoint.setTime(time);
      newWaypoint.setPosition(position);
      newWaypoint.setLinearVelocityToNaN();
   }

   public void appendWaypoint(double time, Point3d position, Vector3d linearVelocity)
   {
      waypoints.add().setIncludingFrame(referenceFrame, time, position, linearVelocity);
   }

   public void computeWaypointTimes(double firstWaypointTime, double trajectoryTime)
   {
      int numberOfWaypoints = getNumberOfWaypoints();
      if (numberOfWaypoints == 0)
         throw new RuntimeException("There is no waypoint.");

      if (numberOfWaypoints == 1)
      {
         waypoints.get(0).setTime(trajectoryTime);
         return;
      }

      double totalLength = 0.0;

      for (int i = 0; i < numberOfWaypoints - 1; i++)
      {
         FramePoint position = waypoints.get(i).getPosition();
         FramePoint nextPosition = waypoints.get(i + 1).getPosition();
         totalLength += position.distance(nextPosition);
      }

      waypoints.get(0).setTime(firstWaypointTime);
      waypoints.get(waypoints.size() - 1).setTime(firstWaypointTime + trajectoryTime);
      double time = firstWaypointTime;

      for (int i = 1; i < numberOfWaypoints - 1; i++)
      {
         FramePoint position = waypoints.get(i).getPosition();
         FramePoint previousPosition = waypoints.get(i - 1).getPosition();
         double subLength = position.distance(previousPosition);
         time += trajectoryTime * (subLength / totalLength);
         waypoints.get(i).setTime(time);
      }
   }

   private final FrameVector computedLinearVelocity = new FrameVector();

   public void computeWaypointVelocities(boolean startAndFinishWithZeroVelocity)
   {
      int numberOfWaypoints = getNumberOfWaypoints();
      if (numberOfWaypoints < 3)
         throw new RuntimeException("Need at least 3 waypoints.");

      if (startAndFinishWithZeroVelocity)
      {
         waypoints.get(0).setLinearVelocityToZero();
         waypoints.get(numberOfWaypoints - 1).setLinearVelocityToZero();

         if (numberOfWaypoints == 3)
         {
            computedLinearVelocity.setToZero(waypoints.get(1).getReferenceFrame());
            compute2ndWaypointVelocityWithVelocityConstraint(waypoints.get(0), waypoints.get(1), waypoints.get(2), computedLinearVelocity);
            waypoints.get(1).setLinearVelocity(computedLinearVelocity);
            return;
         }
      }
      else
      {
         computedLinearVelocity.setToZero(waypoints.get(0).getReferenceFrame());
         computeWaypointVelocity(waypoints.get(0), waypoints.get(1), waypoints.get(2), Waypoint.FIRST, computedLinearVelocity);
         waypoints.get(0).setLinearVelocity(computedLinearVelocity);

         computedLinearVelocity.setToZero(waypoints.get(numberOfWaypoints - 1).getReferenceFrame());
         computeWaypointVelocity(waypoints.get(numberOfWaypoints - 3), waypoints.get(numberOfWaypoints - 2), waypoints.get(numberOfWaypoints - 1), Waypoint.THIRD, computedLinearVelocity);
         waypoints.get(numberOfWaypoints - 1).setLinearVelocity(computedLinearVelocity);
      }

      for (int i = 1; i < numberOfWaypoints - 1; i++)
      {
         FrameEuclideanWaypoint firstWaypoint = waypoints.get(i - 1);
         FrameEuclideanWaypoint secondWaypoint = waypoints.get(i);
         FrameEuclideanWaypoint thirdWaypoint = waypoints.get(i + 1);
         computedLinearVelocity.setToZero(waypoints.get(i).getReferenceFrame());
         computeWaypointVelocity(firstWaypoint, secondWaypoint, thirdWaypoint, Waypoint.SECOND, computedLinearVelocity);
         waypoints.get(i).setLinearVelocity(computedLinearVelocity);
      }
   }

   private final FramePoint tempFramePoint = new FramePoint();
   private final FrameVector tempFrameVector = new FrameVector();

   private void compute2ndWaypointVelocityWithVelocityConstraint(EuclideanWaypointInterface<?> firstWaypoint, EuclideanWaypointInterface<?> secondWaypoint,
         EuclideanWaypointInterface<?> thirdWaypoint, FrameVector linearVelocityToPack)
   {
      for (Direction direction : Direction.values)
      {
         firstWaypoint.getPosition(tempFramePoint.getPoint());
         firstWaypoint.getLinearVelocity(tempFrameVector.getVector());
         double t0 = firstWaypoint.getTime();
         double z0 = tempFramePoint.get(direction);
         double zd0 = tempFrameVector.get(direction);

         secondWaypoint.getPosition(tempFramePoint.getPoint());
         double tIntermediate = secondWaypoint.getTime();
         double zIntermediate = tempFramePoint.get(direction);

         thirdWaypoint.getPosition(tempFramePoint.getPoint());
         thirdWaypoint.getLinearVelocity(tempFrameVector.getVector());
         double tf = thirdWaypoint.getTime();
         double zf = tempFramePoint.get(direction);
         double zdf = tempFrameVector.get(direction);

         YoPolynomial polynomial = polynomials.get(direction);
         polynomial.setQuarticUsingWayPoint(t0, tIntermediate, tf, z0, zd0, zIntermediate, zf, zdf);
         polynomial.compute(tIntermediate);
         linearVelocityToPack.set(direction, polynomial.getVelocity());
      }
   }

   private enum Waypoint
   {
      FIRST, SECOND, THIRD
   };

   private void computeWaypointVelocity(EuclideanWaypointInterface<?> firstWaypoint, EuclideanWaypointInterface<?> secondWaypoint, EuclideanWaypointInterface<?> thirdWaypoint,
         Waypoint waypointToComputeVelocityOf, FrameVector linearVelocityToPack)
   {
      for (Direction direction : Direction.values)
      {
         firstWaypoint.getPosition(tempFramePoint.getPoint());
         firstWaypoint.getLinearVelocity(tempFrameVector.getVector());
         double t0 = firstWaypoint.getTime();
         double z0 = tempFramePoint.get(direction);
         
         secondWaypoint.getPosition(tempFramePoint.getPoint());
         double tIntermediate = secondWaypoint.getTime();
         double zIntermediate = tempFramePoint.get(direction);
         
         thirdWaypoint.getPosition(tempFramePoint.getPoint());
         thirdWaypoint.getLinearVelocity(tempFrameVector.getVector());
         double tf = thirdWaypoint.getTime();
         double zf = tempFramePoint.get(direction);
         
         YoPolynomial polynomial = polynomials.get(direction);
         polynomial.setQuadraticUsingIntermediatePoint(t0, tIntermediate, tf, z0, zIntermediate, zf);
         switch (waypointToComputeVelocityOf)
         {
         case FIRST:
            polynomial.compute(t0);
            break;
         case SECOND:
            polynomial.compute(tIntermediate);
            break;
         case THIRD:
            polynomial.compute(tf);
         default:
            break;
         }

         linearVelocityToPack.set(direction, polynomial.getVelocity());
      }
   }

   public int getNumberOfWaypoints()
   {
      return waypoints.size();
   }

   public RecyclingArrayList<? extends EuclideanWaypointInterface<?>> getWaypoints()
   {
      return waypoints;
   }
}
