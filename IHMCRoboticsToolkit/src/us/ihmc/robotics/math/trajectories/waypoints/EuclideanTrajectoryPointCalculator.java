package us.ihmc.robotics.math.trajectories.waypoints;

import java.util.EnumMap;

import org.apache.commons.lang3.StringUtils;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanTrajectoryPointInterface;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * This multi waypoint trajectory generator helper will generate velocities at intermediate way points of a trajectory which will result in a smooth path trough
 * those way points.It is based on the paper "Curvature-continuous 3D Path-planning Using QPMI Method" which can be found at
 * http://cdn.intechopen.com/pdfs-wm/48591.pdf
 *
 * The minimal number of way points is 3.
 */
public class EuclideanTrajectoryPointCalculator
{
   private static final boolean DEBUG = false;

   private ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();

   private final RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = new RecyclingArrayList<>(20, FrameEuclideanTrajectoryPoint.class);
   private final EnumMap<Direction, YoPolynomial> polynomials = new EnumMap<>(Direction.class);

   private boolean useWeightMethod = false;
   private double minWeight = Double.NaN;
   private double maxWeight = Double.NaN;
   private double weightMethodPow = 4.0;
   private final TDoubleArrayList subLengths = new TDoubleArrayList();
   private final TDoubleArrayList waypointDistanceFromStart = new TDoubleArrayList();
   private final TDoubleArrayList weightedSubLengths = new TDoubleArrayList();
   private final TDoubleArrayList weights = new TDoubleArrayList();

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

   public void appendTrajectoryPoint(Point3D position)
   {
      FrameEuclideanTrajectoryPoint newTrajectoryPoint = trajectoryPoints.add();
      newTrajectoryPoint.setToZero(referenceFrame);
      newTrajectoryPoint.setTimeToNaN();
      newTrajectoryPoint.setPosition(position);
      newTrajectoryPoint.setLinearVelocityToNaN();
   }

   public void appendTrajectoryPoint(double time, Point3D position)
   {
      FrameEuclideanTrajectoryPoint newTrajectoryPoint = trajectoryPoints.add();
      newTrajectoryPoint.setToZero(referenceFrame);
      newTrajectoryPoint.setTime(time);
      newTrajectoryPoint.setPosition(position);
      newTrajectoryPoint.setLinearVelocityToNaN();
   }

   public void appendTrajectoryPoint(double time, Point3D position, Vector3D linearVelocity)
   {
      trajectoryPoints.add().setIncludingFrame(referenceFrame, time, position, linearVelocity);
   }

   public void enableWeightMethod(double maxWeight, double minWeight)
   {
      MathTools.checkIfInRange(minWeight, 1.0-3, 100.0);
      MathTools.checkIfInRange(maxWeight, 1.0-3, 100.0);

      this.useWeightMethod = true;
      double weightRatio = maxWeight / minWeight;
      this.minWeight = 1.0;
      this.maxWeight = weightRatio;
   }

   public void setWeightMethodPow(double weightMethodPow)
   {
      this.weightMethodPow = weightMethodPow;
   }

   public void disableWeightMethod()
   {
      this.useWeightMethod = false;
      minWeight = Double.NaN;
      maxWeight = Double.NaN;
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
      subLengths.reset();
      waypointDistanceFromStart.reset();
      waypointDistanceFromStart.add(0.0);

      for (int i = 0; i < numberOfTrajectoryPoints - 1; i++)
      {
         double subLength = trajectoryPoints.get(i).positionDistance(trajectoryPoints.get(i + 1));
         subLengths.add(subLength);
         totalLength += subLength;
         waypointDistanceFromStart.add(totalLength);
         if (DEBUG)
            System.out.println("Sub length: " + i + ": " + subLength);
      }

      trajectoryPoints.get(0).setTime(firstTrajectoryPointTime);
      trajectoryPoints.get(trajectoryPoints.size() - 1).setTime(firstTrajectoryPointTime + trajectoryTime);


      if (useWeightMethod)
      {
         double weightedTotalLength = 0.0;
         double halfTotalLength = totalLength / 2.0;
         weightedSubLengths.reset();

         for (int i = 0; i < numberOfTrajectoryPoints; i++)
         {
            double distanceFromMiddle = Math.abs(waypointDistanceFromStart.get(i) - halfTotalLength);
            double distanceFromMiddleNormalized = Math.pow(distanceFromMiddle / halfTotalLength, weightMethodPow);
            double weight = distanceFromMiddleNormalized * maxWeight + (1.0 - distanceFromMiddleNormalized) * minWeight;
            weights.add(weight);
         }

         for (int i = 0; i < numberOfTrajectoryPoints - 1; i++)
         {
            double weightedSubLength = 0.5 * (weights.get(i) + weights.get(i + 1)) * subLengths.get(i);
            weightedSubLengths.add(weightedSubLength);
            weightedTotalLength += weightedSubLength;
            if (DEBUG)
               System.out.println("Weighted sub length: " + i + ": " + weightedSubLength);
         }

         if (DEBUG)
         {
            System.out.println(totalLength);
            System.out.println(weightedTotalLength);
         }
         
         double time = firstTrajectoryPointTime;

         for (int i = 1; i <= numberOfTrajectoryPoints - 1; i++)
         {
            double weightedSubLength = weightedSubLengths.get(i - 1);
            double relativeTime = trajectoryTime * (weightedSubLength / weightedTotalLength);
            if (DEBUG)
               System.out.println("Relative time: " + i + ": " + relativeTime);
            time += relativeTime;
            trajectoryPoints.get(i).setTime(time);
         }
      }
      else
      {
         double time = firstTrajectoryPointTime;

         for (int i = 1; i < numberOfTrajectoryPoints - 1; i++)
         {
            double subLength = subLengths.get(i);
            time += trajectoryTime * (subLength / totalLength);
            trajectoryPoints.get(i).setTime(time);
         }
      }
   }

   private final FrameVector computedLinearVelocity = new FrameVector();

   public void computeTrajectoryPointVelocities(boolean startAndFinishWithZeroVelocity)
   {
      int numberOfTrajectoryPoints = getNumberOfTrajectoryPoints();
      if (numberOfTrajectoryPoints < 3)
         throw new RuntimeException("Need at least 3 trajectory points.");

      FrameEuclideanTrajectoryPoint firstTrajectoryPoint;
      FrameEuclideanTrajectoryPoint secondTrajectoryPoint;
      FrameEuclideanTrajectoryPoint thirdTrajectoryPoint;

      if (startAndFinishWithZeroVelocity)
      {
         trajectoryPoints.get(0).setLinearVelocityToZero();
         trajectoryPoints.get(numberOfTrajectoryPoints - 1).setLinearVelocityToZero();

         if (numberOfTrajectoryPoints == 3)
         {
            firstTrajectoryPoint = trajectoryPoints.get(0);
            secondTrajectoryPoint = trajectoryPoints.get(1);
            thirdTrajectoryPoint = trajectoryPoints.get(2);

            computedLinearVelocity.setToZero(secondTrajectoryPoint.getReferenceFrame());
            compute2ndTrajectoryPointVelocityWithVelocityConstraint(firstTrajectoryPoint, secondTrajectoryPoint, thirdTrajectoryPoint, computedLinearVelocity);
            secondTrajectoryPoint.setLinearVelocity(computedLinearVelocity);
            return;
         }
      }
      else
      {
         firstTrajectoryPoint = trajectoryPoints.get(0);
         secondTrajectoryPoint = trajectoryPoints.get(1);
         thirdTrajectoryPoint = trajectoryPoints.get(2);

         computedLinearVelocity.setToZero(firstTrajectoryPoint.getReferenceFrame());
         computeTrajectoryPointVelocity(firstTrajectoryPoint, secondTrajectoryPoint, thirdTrajectoryPoint, TrajectoryPoint.FIRST, computedLinearVelocity);
         firstTrajectoryPoint.setLinearVelocity(computedLinearVelocity);

         firstTrajectoryPoint = trajectoryPoints.get(numberOfTrajectoryPoints - 3);
         secondTrajectoryPoint = trajectoryPoints.get(numberOfTrajectoryPoints - 2);
         thirdTrajectoryPoint = trajectoryPoints.get(numberOfTrajectoryPoints - 1);

         computedLinearVelocity.setToZero(thirdTrajectoryPoint.getReferenceFrame());
         computeTrajectoryPointVelocity(firstTrajectoryPoint, secondTrajectoryPoint, thirdTrajectoryPoint, TrajectoryPoint.THIRD, computedLinearVelocity);
         thirdTrajectoryPoint.setLinearVelocity(computedLinearVelocity);
      }

      for (int i = 1; i < numberOfTrajectoryPoints - 1; i++)
      {
         firstTrajectoryPoint = trajectoryPoints.get(i - 1);
         secondTrajectoryPoint = trajectoryPoints.get(i);
         thirdTrajectoryPoint = trajectoryPoints.get(i + 1);
         computedLinearVelocity.setToZero(trajectoryPoints.get(i).getReferenceFrame());
         computeTrajectoryPointVelocity(firstTrajectoryPoint, secondTrajectoryPoint, thirdTrajectoryPoint, TrajectoryPoint.SECOND, computedLinearVelocity);
         secondTrajectoryPoint.setLinearVelocity(computedLinearVelocity);
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

   public RecyclingArrayList<FrameEuclideanTrajectoryPoint> getTrajectoryPoints()
   {
      return trajectoryPoints;
   }
}
