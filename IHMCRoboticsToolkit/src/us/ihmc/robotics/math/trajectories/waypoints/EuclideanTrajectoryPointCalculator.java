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

public class EuclideanTrajectoryPointCalculator
{
   private ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();

   private final RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = new RecyclingArrayList<>(20, FrameEuclideanTrajectoryPoint.class);
   private final EnumMap<Direction, YoPolynomial> polynomials = new EnumMap<>(Direction.class);

   public EuclideanTrajectoryPointCalculator()
   {
      clear();
      YoVariableRegistry dummyRegistry = new YoVariableRegistry("Dummy");
      for (Direction direction : Direction.values)
         polynomials.put(direction, new YoPolynomial(StringUtils.lowerCase(direction.toString()) + "Polynomial", 6, dummyRegistry));
   }

   public void clear()
   {
      trajectoryPoints.clear();
   }

   public void changeFrame(ReferenceFrame referenceFrame)
   {
      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
         trajectoryPoints.get(i).changeFrame(referenceFrame);
      this.referenceFrame = referenceFrame;
   }

   public void appendTrajectoryPoint(EuclideanTrajectoryPointInterface<?> trajectoryPoint)
   {
      trajectoryPoints.add().set(trajectoryPoint);
   }

   public void appendTrajectoryPoint(Point3d position)
   {
      FrameEuclideanTrajectoryPoint newTrajectoryPoint = trajectoryPoints.add();
      newTrajectoryPoint.setToZero(referenceFrame);
      newTrajectoryPoint.setTimeToNaN();
      newTrajectoryPoint.setPosition(position);
      newTrajectoryPoint.setLinearVelocityToNaN();
   }

   public void appendTrajectoryPoint(double time, Point3d position)
   {
      FrameEuclideanTrajectoryPoint newTrajectoryPoint = trajectoryPoints.add();
      newTrajectoryPoint.setToZero(referenceFrame);
      newTrajectoryPoint.setTime(time);
      newTrajectoryPoint.setPosition(position);
      newTrajectoryPoint.setLinearVelocityToNaN();
   }

   public void appendTrajectoryPoint(double time, Point3d position, Vector3d linearVelocity)
   {
      trajectoryPoints.add().setIncludingFrame(referenceFrame, time, position, linearVelocity);
   }

   public void computeTrajectoryPointTimes(double firstTrajectoryPointTime, double trajectoryTime)
   {
      int numberOfTrajectoryPoints = getNumberOfTrajectoryPoints();
      if (numberOfTrajectoryPoints == 0)
         throw new RuntimeException("There is no trajectory point.");

      if (numberOfTrajectoryPoints == 1)
      {
         trajectoryPoints.get(0).setTime(trajectoryTime);
         return;
      }

      double totalLength = 0.0;

      for (int i = 0; i < numberOfTrajectoryPoints - 1; i++)
      {
         FramePoint position = trajectoryPoints.get(i).getPosition();
         FramePoint nextPosition = trajectoryPoints.get(i + 1).getPosition();
         totalLength += position.distance(nextPosition);
      }

      trajectoryPoints.get(0).setTime(firstTrajectoryPointTime);
      trajectoryPoints.get(trajectoryPoints.size() - 1).setTime(firstTrajectoryPointTime + trajectoryTime);
      double time = firstTrajectoryPointTime;

      for (int i = 1; i < numberOfTrajectoryPoints - 1; i++)
      {
         FramePoint position = trajectoryPoints.get(i).getPosition();
         FramePoint previousPosition = trajectoryPoints.get(i - 1).getPosition();
         double subLength = position.distance(previousPosition);
         time += trajectoryTime * (subLength / totalLength);
         trajectoryPoints.get(i).setTime(time);
      }
   }

   private final FrameVector computedLinearVelocity = new FrameVector();

   public void computeTrajectoryPointVelocities(boolean startAndFinishWithZeroVelocity)
   {
      int numberOfTrajectoryPoints = getNumberOfTrajectoryPoints();
      if (numberOfTrajectoryPoints < 3)
         throw new RuntimeException("Need at least 3 trajectory points.");

      if (startAndFinishWithZeroVelocity)
      {
         trajectoryPoints.get(0).setLinearVelocityToZero();
         trajectoryPoints.get(numberOfTrajectoryPoints - 1).setLinearVelocityToZero();

         if (numberOfTrajectoryPoints == 3)
         {
            computedLinearVelocity.setToZero(trajectoryPoints.get(1).getReferenceFrame());
            compute2ndTrajectoryPointVelocityWithVelocityConstraint(trajectoryPoints.get(0), trajectoryPoints.get(1), trajectoryPoints.get(2),
                  computedLinearVelocity);
            trajectoryPoints.get(1).setLinearVelocity(computedLinearVelocity);
            return;
         }
      }
      else
      {
         computedLinearVelocity.setToZero(trajectoryPoints.get(0).getReferenceFrame());
         computeTrajectoryPointVelocity(trajectoryPoints.get(0), trajectoryPoints.get(1), trajectoryPoints.get(2), TrajectoryPoint.FIRST,
               computedLinearVelocity);
         trajectoryPoints.get(0).setLinearVelocity(computedLinearVelocity);

         computedLinearVelocity.setToZero(trajectoryPoints.get(numberOfTrajectoryPoints - 1).getReferenceFrame());
         computeTrajectoryPointVelocity(trajectoryPoints.get(numberOfTrajectoryPoints - 3), trajectoryPoints.get(numberOfTrajectoryPoints - 2),
               trajectoryPoints.get(numberOfTrajectoryPoints - 1), TrajectoryPoint.THIRD, computedLinearVelocity);
         trajectoryPoints.get(numberOfTrajectoryPoints - 1).setLinearVelocity(computedLinearVelocity);
      }

      for (int i = 1; i < numberOfTrajectoryPoints - 1; i++)
      {
         FrameEuclideanTrajectoryPoint firstTrajectoryPoint = trajectoryPoints.get(i - 1);
         FrameEuclideanTrajectoryPoint secondTrajectoryPoint = trajectoryPoints.get(i);
         FrameEuclideanTrajectoryPoint thirdTrajectoryPoint = trajectoryPoints.get(i + 1);
         computedLinearVelocity.setToZero(trajectoryPoints.get(i).getReferenceFrame());
         computeTrajectoryPointVelocity(firstTrajectoryPoint, secondTrajectoryPoint, thirdTrajectoryPoint, TrajectoryPoint.SECOND, computedLinearVelocity);
         trajectoryPoints.get(i).setLinearVelocity(computedLinearVelocity);
      }
   }

   private final FramePoint tempFramePoint = new FramePoint();
   private final FrameVector tempFrameVector = new FrameVector();

   private void compute2ndTrajectoryPointVelocityWithVelocityConstraint(EuclideanTrajectoryPointInterface<?> firstTrajectoryPoint,
         EuclideanTrajectoryPointInterface<?> secondTrajectoryPoint, EuclideanTrajectoryPointInterface<?> thirdTrajectoryPoint,
         FrameVector linearVelocityToPack)
   {
      for (Direction direction : Direction.values)
      {
         firstTrajectoryPoint.getPosition(tempFramePoint.getPoint());
         firstTrajectoryPoint.getLinearVelocity(tempFrameVector.getVector());
         double t0 = firstTrajectoryPoint.getTime();
         double z0 = tempFramePoint.get(direction);
         double zd0 = tempFrameVector.get(direction);

         secondTrajectoryPoint.getPosition(tempFramePoint.getPoint());
         double tIntermediate = secondTrajectoryPoint.getTime();
         double zIntermediate = tempFramePoint.get(direction);

         thirdTrajectoryPoint.getPosition(tempFramePoint.getPoint());
         thirdTrajectoryPoint.getLinearVelocity(tempFrameVector.getVector());
         double tf = thirdTrajectoryPoint.getTime();
         double zf = tempFramePoint.get(direction);
         double zdf = tempFrameVector.get(direction);

         YoPolynomial polynomial = polynomials.get(direction);
         polynomial.setQuarticUsingWayPoint(t0, tIntermediate, tf, z0, zd0, zIntermediate, zf, zdf);
         polynomial.compute(tIntermediate);
         linearVelocityToPack.set(direction, polynomial.getVelocity());
      }
   }

   private enum TrajectoryPoint
   {
      FIRST, SECOND, THIRD
   };

   private void computeTrajectoryPointVelocity(EuclideanTrajectoryPointInterface<?> firstTrajectoryPoint,
         EuclideanTrajectoryPointInterface<?> secondTrajectoryPoint, EuclideanTrajectoryPointInterface<?> thirdTrajectoryPoint,
         TrajectoryPoint trajectoryPointToComputeVelocityOf, FrameVector linearVelocityToPack)
   {
      for (Direction direction : Direction.values)
      {
         firstTrajectoryPoint.getPosition(tempFramePoint.getPoint());
         firstTrajectoryPoint.getLinearVelocity(tempFrameVector.getVector());
         double t0 = firstTrajectoryPoint.getTime();
         double z0 = tempFramePoint.get(direction);

         secondTrajectoryPoint.getPosition(tempFramePoint.getPoint());
         double tIntermediate = secondTrajectoryPoint.getTime();
         double zIntermediate = tempFramePoint.get(direction);

         thirdTrajectoryPoint.getPosition(tempFramePoint.getPoint());
         thirdTrajectoryPoint.getLinearVelocity(tempFrameVector.getVector());
         double tf = thirdTrajectoryPoint.getTime();
         double zf = tempFramePoint.get(direction);

         YoPolynomial polynomial = polynomials.get(direction);
         polynomial.setQuadraticUsingIntermediatePoint(t0, tIntermediate, tf, z0, zIntermediate, zf);
         switch (trajectoryPointToComputeVelocityOf)
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

   public int getNumberOfTrajectoryPoints()
   {
      return trajectoryPoints.size();
   }

   public RecyclingArrayList<? extends EuclideanTrajectoryPointInterface<?>> getTrajectoryPoints()
   {
      return trajectoryPoints;
   }
}
