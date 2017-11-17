package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.OneDoFTrajectoryPointInterface;

public class TrajectoryPoint1DCalculator
{
   private static final double EPSILON = 1.0e-7;
   private final SimpleTrajectoryPoint1DList trajectory = new SimpleTrajectoryPoint1DList();
   private final YoPolynomial polynomial = new YoPolynomial("polynomial", 6, new YoVariableRegistry("Dummy"));

   public TrajectoryPoint1DCalculator()
   {
      clear();
   }

   public void clear()
   {
      trajectory.clear();
   }

   public void appendTrajectoryPoint(OneDoFTrajectoryPointInterface<?> trajectoryPoint)
   {
      trajectory.addTrajectoryPoint(trajectoryPoint);
   }

   public void appendTrajectoryPoint(double position)
   {
      appendTrajectoryPoint(Double.NaN, position);
   }

   public void appendTrajectoryPoint(double time, double position)
   {
      appendTrajectoryPoint(time, position, Double.NaN);
   }

   public void appendTrajectoryPoint(double time, double position, double velocity)
   {
      trajectory.addTrajectoryPoint(time, position, velocity);
   }

   public void computeTrajectoryPointTimes(double firstTrajectoryPointTime, double trajectoryTime)
   {
      int numberOfTrajectoryPoints = getNumberOfTrajectoryPoints();
      if (numberOfTrajectoryPoints == 0)
         throw new RuntimeException("There is no trajectory point.");

      if (numberOfTrajectoryPoints == 1)
      {
         trajectory.getTrajectoryPoint(0).setTime(trajectoryTime);
         return;
      }

      double totalLength = 0.0;

      for (int i = 0; i < numberOfTrajectoryPoints - 1; i++)
         totalLength += Math.abs(trajectory.getTrajectoryPoint(i + 1).getPosition() - trajectory.getTrajectoryPoint(i).getPosition());

      trajectory.getTrajectoryPoint(0).setTime(firstTrajectoryPointTime);
      trajectory.getTrajectoryPoint(trajectory.getNumberOfTrajectoryPoints() - 1).setTime(firstTrajectoryPointTime + trajectoryTime);
      double time = firstTrajectoryPointTime;

      if (totalLength > EPSILON * getNumberOfTrajectoryPoints())
      {
         for (int i = 1; i < numberOfTrajectoryPoints - 1; i++)
         {
            double subLength = Math.abs(trajectory.getTrajectoryPoint(i).getPosition() - trajectory.getTrajectoryPoint(i - 1).getPosition());
            time += trajectoryTime * (subLength / totalLength);
            trajectory.getTrajectoryPoint(i).setTime(time);
         }
      }
      else
      {
         for (int i = 1; i < numberOfTrajectoryPoints - 1; i++)
         {
            time += trajectoryTime / getNumberOfTrajectoryPoints();
            trajectory.getTrajectoryPoint(i).setTime(time);
         }
      }
   }

   public void computeTrajectoryPointVelocities(boolean startAndFinishWithZeroVelocity)
   {
      int numberOfTrajectoryPoints = getNumberOfTrajectoryPoints();
      if (numberOfTrajectoryPoints < 3)
         throw new RuntimeException("Need at least 3 trajectory points.");

      SimpleTrajectoryPoint1D firstTrajectoryPoint;
      SimpleTrajectoryPoint1D secondTrajectoryPoint;
      SimpleTrajectoryPoint1D thirdTrajectoryPoint;

      if (startAndFinishWithZeroVelocity)
      {
         trajectory.getTrajectoryPoint(0).setVelocity(0.0);
         trajectory.getTrajectoryPoint(numberOfTrajectoryPoints - 1).setVelocity(0.0);

         if (numberOfTrajectoryPoints == 3)
         {
            firstTrajectoryPoint = trajectory.getTrajectoryPoint(0);
            secondTrajectoryPoint = trajectory.getTrajectoryPoint(1);
            thirdTrajectoryPoint = trajectory.getTrajectoryPoint(2);
            secondTrajectoryPoint
                  .setVelocity(compute2ndTrajectoryPointVelocityWithVelocityConstraint(firstTrajectoryPoint, secondTrajectoryPoint, thirdTrajectoryPoint));
            return;
         }
      }
      else
      {
         firstTrajectoryPoint = trajectory.getTrajectoryPoint(0);
         secondTrajectoryPoint = trajectory.getTrajectoryPoint(1);
         thirdTrajectoryPoint = trajectory.getTrajectoryPoint(2);
         firstTrajectoryPoint
               .setVelocity(computeTrajectoryPointVelocity(firstTrajectoryPoint, secondTrajectoryPoint, thirdTrajectoryPoint, TrajectoryPoint.FIRST));

         firstTrajectoryPoint = trajectory.getTrajectoryPoint(numberOfTrajectoryPoints - 3);
         secondTrajectoryPoint = trajectory.getTrajectoryPoint(numberOfTrajectoryPoints - 2);
         thirdTrajectoryPoint = trajectory.getTrajectoryPoint(numberOfTrajectoryPoints - 1);
         thirdTrajectoryPoint
               .setVelocity(computeTrajectoryPointVelocity(firstTrajectoryPoint, secondTrajectoryPoint, thirdTrajectoryPoint, TrajectoryPoint.THIRD));
      }

      for (int i = 1; i < numberOfTrajectoryPoints - 1; i++)
      {
         firstTrajectoryPoint = trajectory.getTrajectoryPoint(i - 1);
         secondTrajectoryPoint = trajectory.getTrajectoryPoint(i);
         thirdTrajectoryPoint = trajectory.getTrajectoryPoint(i + 1);
         secondTrajectoryPoint
               .setVelocity(computeTrajectoryPointVelocity(firstTrajectoryPoint, secondTrajectoryPoint, thirdTrajectoryPoint, TrajectoryPoint.SECOND));
      }
   }

   public boolean shouldVelocityBeZero(OneDoFTrajectoryPointInterface<?> firstTrajectoryPoint, OneDoFTrajectoryPointInterface<?> secondTrajectoryPoint)
   {
      double deltaPosition = Math.abs(secondTrajectoryPoint.getPosition() - firstTrajectoryPoint.getPosition());
      double deltaTime = Math.abs(secondTrajectoryPoint.getTime() - firstTrajectoryPoint.getTime());
      return MathTools.epsilonEquals(0.0, deltaPosition / deltaTime, 1.0e-7);
   }

   private double compute2ndTrajectoryPointVelocityWithVelocityConstraint(OneDoFTrajectoryPointInterface<?> firstTrajectoryPoint,
         OneDoFTrajectoryPointInterface<?> secondTrajectoryPoint, OneDoFTrajectoryPointInterface<?> thirdTrajectoryPoint)
   {
      double t0 = firstTrajectoryPoint.getTime();
      double z0 = firstTrajectoryPoint.getPosition();
      double zd0 = firstTrajectoryPoint.getVelocity();

      double tIntermediate = secondTrajectoryPoint.getTime();
      double zIntermediate = secondTrajectoryPoint.getPosition();

      double tf = thirdTrajectoryPoint.getTime();
      double zf = thirdTrajectoryPoint.getPosition();
      double zdf = thirdTrajectoryPoint.getVelocity();

      if (MathTools.epsilonEquals(tf, t0, EPSILON))
         return 0.0;
      else if (MathTools.epsilonEquals(t0, tIntermediate, EPSILON))
         tIntermediate += 0.001 * (tf - t0);
      else if (MathTools.epsilonEquals(tIntermediate, tf, EPSILON))
         tIntermediate -= 0.001 * (tf - t0);

      polynomial.setQuarticUsingWayPoint(t0, tIntermediate, tf, z0, zd0, zIntermediate, zf, zdf);
      polynomial.compute(tIntermediate);

      return polynomial.getVelocity();
   }

   private enum TrajectoryPoint
   {
      FIRST, SECOND, THIRD
   };

   private double computeTrajectoryPointVelocity(OneDoFTrajectoryPointInterface<?> firstTrajectoryPoint, OneDoFTrajectoryPointInterface<?> secondTrajectoryPoint,
         OneDoFTrajectoryPointInterface<?> thirdTrajectoryPoint, TrajectoryPoint trajectoryPointToComputeVelocityOf)
   {
      double t0 = firstTrajectoryPoint.getTime();
      double z0 = firstTrajectoryPoint.getPosition();

      double tIntermediate = secondTrajectoryPoint.getTime();
      double zIntermediate = secondTrajectoryPoint.getPosition();

      double tf = thirdTrajectoryPoint.getTime();
      double zf = thirdTrajectoryPoint.getPosition();

      if (MathTools.epsilonEquals(tf, t0, EPSILON))
         return 0.0;
      else if (MathTools.epsilonEquals(t0, tIntermediate, EPSILON))
         tIntermediate += 0.001 * (tf - t0);
      else if (MathTools.epsilonEquals(tIntermediate, tf, EPSILON))
         tIntermediate -= 0.001 * (tf - t0);

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

      return polynomial.getVelocity();
   }

   public int getNumberOfTrajectoryPoints()
   {
      return trajectory.getNumberOfTrajectoryPoints();
   }

   public RecyclingArrayList<? extends OneDoFTrajectoryPointInterface<?>> getTrajectoryPoints()
   {
      return trajectory.getTrajectoryPoints();
   }

   public SimpleTrajectoryPoint1DList getTrajectoryData()
   {
      return trajectory;
   }
}
