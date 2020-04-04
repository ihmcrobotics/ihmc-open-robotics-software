package us.ihmc.robotics.math.trajectories;

import java.security.InvalidParameterException;
import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.dataStructures.ComplexNumber;
import us.ihmc.robotics.math.FastFourierTransform;

public class TrajectoryMathTools
{
   private final int maxNumberOfCoefficients;
   private final Trajectory tempTraj1;
   private final Trajectory tempTraj2;
   private final Trajectory3D tempTraj3D;
   private final TDoubleArrayList tempTimeList = new TDoubleArrayList(4);
   private final FastFourierTransform fft;
   private final ComplexNumber[] tempComplex1;
   private final ComplexNumber[] tempComplex2;


   public TrajectoryMathTools(int maxNumberOfCoefficients)
   {
      this.maxNumberOfCoefficients = (int) Math.pow(2, Math.ceil(Math.log(maxNumberOfCoefficients) / Math.log(2.0))); // Rounding up the nearest power of two
      this.tempTraj1 = new Trajectory(this.maxNumberOfCoefficients);
      this.tempTraj2 = new Trajectory(this.maxNumberOfCoefficients);
      this.tempTraj3D = new Trajectory3D(this.maxNumberOfCoefficients);
      this.fft = new FastFourierTransform(this.maxNumberOfCoefficients);
      this.tempComplex1 = ComplexNumber.getComplexArray(this.maxNumberOfCoefficients);
      this.tempComplex2 = ComplexNumber.getComplexArray(this.maxNumberOfCoefficients);
   }

   public static void scale(Trajectory scaledTrajectoryToPack, Trajectory trajectoryToScale, double scalar)
   {
      scaledTrajectoryToPack.set(trajectoryToScale);
      for (int i = 0; i < trajectoryToScale.getNumberOfCoefficients(); i++)
         scaledTrajectoryToPack.setDirectlyFast(i, trajectoryToScale.getCoefficient(i) * scalar);
   }

   public static void scale(Trajectory trajectoryToScale, double scalar)
   {
      for (int i = 0; i < trajectoryToScale.getNumberOfCoefficients(); i++)
         trajectoryToScale.setDirectlyFast(i, trajectoryToScale.getCoefficient(i) * scalar);
   }

   /**
    * Add two trajectories that have the same start and end times. 
    * Throws runtime exception in case the start and end time values do not match
    * @param trajToPack
    * @param traj1
    * @param traj2
    */
   public static void add(Trajectory trajToPack, Trajectory traj1, Trajectory traj2)
   {
      validatePackingTrajectoryForLinearCombination(trajToPack, traj1, traj2);
      validateTrajectoryTimes(traj1, traj2);
      trajToPack.setTime(traj1.getInitialTime(), traj2.getFinalTime());
      setCoeffsByAddition(trajToPack, traj1, traj2);
   }

   /**
    * Adds two trajectories by taking the intersection of the time intervals over which they are defined.
    * Throws runtime exception in case null intersection is found between the two trajectories.
    */
   public static void addByTrimming(Trajectory3D segmentToPack, Trajectory3D segment1, Trajectory3D segment2)
   {
      for (int direction = 0; direction < 3; direction++)
         addByTrimming(segmentToPack.getTrajectory(direction), segment1.getTrajectory(direction), segment2.getTrajectory(direction));
   }

   /**
    * Adds two trajectories by taking the intersection of the time intervals over which they are defined.
    * Throws runtime exception in case null intersection is found between the two trajectories.
    */
   public static void addByTrimming(Trajectory segmentToPack, Trajectory segment1, Trajectory segment2)
   {
      validatePackingTrajectoryForLinearCombination(segmentToPack, segment1, segment2);
      setTimeIntervalByTrimming(segmentToPack, segment1, segment2);
      setCoeffsByAddition(segmentToPack, segment1, segment2);
   }

   private static void setCoeffsByAddition(Trajectory trajectoryToPack, Trajectory trajectory1, Trajectory trajectory2)
   {
      int numberOfCoeffsToSet = Math.max(trajectory1.getNumberOfCoefficients(), trajectory2.getNumberOfCoefficients());

      for (int i = 0; i < numberOfCoeffsToSet; i++)
      {
         double coefficient = 0.0;
         if (i < trajectory1.getNumberOfCoefficients())
            coefficient += trajectory1.getCoefficient(i);
         if (i < trajectory2.getNumberOfCoefficients())
            coefficient += trajectory2.getCoefficient(i);
         trajectoryToPack.setDirectlyFast(i, coefficient);
      }
      trajectoryToPack.reshape(numberOfCoeffsToSet);
   }

   /**
    * Subtracts {@code traj2} from {@code traj1} in case the two have the same start and end times
    * Throws runtime exception in case the start and end time values do not match or if {@code trajToPack} is not large enough to store the result
    * @param trajToPack
    * @param traj1
    * @param traj2
    */
   public static void subtract(Trajectory trajToPack, Trajectory traj1, Trajectory traj2)
   {
      validatePackingTrajectoryForLinearCombination(trajToPack, traj1, traj2);
      validateTrajectoryTimes(traj1, traj2);
      trajToPack.setTime(traj1.getInitialTime(), traj2.getFinalTime());
      setCoeffsBySubtraction(trajToPack, traj1, traj2);
   }

   /**
    * Subtracts {@code traj2} from {@code traj1} in case the two have the same start and end times
    * Throws runtime exception in case the start and end time values do not match or if {@code trajToPack} is not large enough to store the result
    * @param traj1
    * @param traj2
    */
   public static void subtractEquals(Trajectory traj1, Trajectory traj2)
   {
      validatePackingTrajectoryForLinearCombination(traj1, traj2);
      validateTrajectoryTimes(traj1, traj2);
      traj1.setTime(traj1.getInitialTime(), traj2.getFinalTime());
      setCoeffsBySubtraction(traj1, traj1, traj2);
   }

   /**
    * Subtracts the two trajectories by taking the intersection of the time intervals over which the two are defined
    * @param trajToPack
    * @param traj1
    * @param traj2
    */
   public static void subtractByTrimming(Trajectory trajToPack, Trajectory traj1, Trajectory traj2)
   {
      validatePackingTrajectoryForLinearCombination(trajToPack, traj1, traj2);
      setTimeIntervalByTrimming(trajToPack, traj1, traj2);
      setCoeffsBySubtraction(trajToPack, traj1, traj2);
   }

   private static void setCoeffsBySubtraction(Trajectory trajectoryToPack, Trajectory trajectory1, Trajectory trajectory2)
   {
      int numberOfCoeffsToSet = Math.max(trajectory1.getNumberOfCoefficients(), trajectory2.getNumberOfCoefficients());
      for (int i = 0; i < numberOfCoeffsToSet; i++)
      {
         double coefficient = 0.0;
         if (i < trajectory1.getNumberOfCoefficients())
            coefficient += trajectory1.getCoefficient(i);
         if (i < trajectory2.getNumberOfCoefficients())
            coefficient -= trajectory2.getCoefficient(i);
         trajectoryToPack.setDirectlyFast(i, coefficient);
      }
      trajectoryToPack.reshape(numberOfCoeffsToSet);
   }

   public void multiply(Trajectory trajToPack, Trajectory traj1, Trajectory traj2)
   {
      validatePackingTrajectoryForMultiplication(trajToPack, traj1, traj2);
      validateTrajectoryTimes(traj1, traj2);
      trajToPack.setTime(traj1.getInitialTime(), traj2.getFinalTime());
      setCoefficientsByMultiplication(trajToPack, traj1, traj2);
   }

   public void multiplySubtract(Trajectory trajToPack, Trajectory traj1, Trajectory traj2)
   {
      validatePackingTrajectoryForMultiplication(trajToPack, traj1, traj2);
      validateTrajectoryTimes(traj1, traj2);
      trajToPack.setTime(traj1.getInitialTime(), traj2.getFinalTime());
      subtractCoefficientsByMultiplication(trajToPack, traj1, traj2);
   }

   public void multiplyByTrimming(Trajectory trajToPack, Trajectory traj1, Trajectory traj2)
   {
      validatePackingTrajectoryForMultiplication(trajToPack, traj1, traj2);
      setTimeIntervalByTrimming(trajToPack, traj1, traj2);
      setCoefficientsByMultiplication(trajToPack, traj1, traj2);
   }

   private void setCoefficientsByMultiplication(Trajectory trajectoryToPack, Trajectory trajectory1, Trajectory trajectory2)
   {
      int numberOfCoeffsToSet = trajectory1.getNumberOfCoefficients() + trajectory2.getNumberOfCoefficients() - 1;

      fft.setCoefficients(trajectory1.getCoefficients(), trajectory1.getNumberOfCoefficients());
      ComplexNumber[] tempComplexReference = fft.getForwardTransform();
      ComplexNumber.copyComplexArray(tempComplex1, tempComplexReference);

      fft.setCoefficients(trajectory2.getCoefficients(), trajectory2.getNumberOfCoefficients());
      tempComplexReference = fft.getForwardTransform();
      ComplexNumber.copyComplexArray(tempComplex2, tempComplexReference);

      for (int i = 0; i < tempComplex1.length; i++)
         tempComplex1[i].timesAndStore(tempComplex2[i]);

      fft.setCoefficients(tempComplex1);
      tempComplexReference = fft.getInverseTransform();

      for (int i = 0; i < numberOfCoeffsToSet; i++)
         trajectoryToPack.setDirectlyFast(i, tempComplexReference[i].real());
      trajectoryToPack.reshape(numberOfCoeffsToSet);
   }

   public static void multiplyNaive(Trajectory trajToPack, Trajectory traj1, Trajectory traj2)
   {
      validatePackingTrajectoryForMultiplication(trajToPack, traj1, traj2);
      validateTrajectoryTimes(traj1, traj2);
      trajToPack.setTime(traj1.getInitialTime(), traj2.getFinalTime());
      setCoefficientsByNaiveMultiplication(trajToPack, traj1, traj2);
   }

   /*
    * TODO The FFT algorithm is supposed to be faster than simple brute-force multiplication, but it
    * appears to be even slower. It is possible that either the FFT implementation is slow or that
    * the fact that it requires the use of temporary variables prevents a good usage of the memory.
    * I think we could implement a divide-and-conquer alogrithm that could still be static and thus
    * definitely faster than the FFT based or naive approach. (Sylvain)
    */
   private static void setCoefficientsByNaiveMultiplication(Trajectory trajToPack, Trajectory traj1, Trajectory traj2)
   {
      int aLength = traj1.getNumberOfCoefficients();
      int bLength = traj2.getNumberOfCoefficients();
      int n = aLength + bLength;
      trajToPack.reshape(n - 1);

      double[] a = traj1.getCoefficients();
      double[] b = traj2.getCoefficients();

      for (int k = 0; k < n; k++)
      {
         double ck = 0.0;

         for (int i = Math.max(k - bLength + 1, 0); i <= Math.min(aLength - 1, k); i++)
         {
            ck += a[i] * b[k - i];
         }

         trajToPack.setDirectly(k, ck);
      }
   }

   private void addCoefficientsByMultiplication(Trajectory trajectoryToPack, Trajectory trajectory1, Trajectory trajectory2)
   {
      int numberOfCoeffsToSet = trajectory1.getNumberOfCoefficients() + trajectory2.getNumberOfCoefficients() - 1;

      fft.setCoefficients(trajectory1.getCoefficients(), trajectory1.getNumberOfCoefficients());
      ComplexNumber[] tempComplexReference = fft.getForwardTransform();
      ComplexNumber.copyComplexArray(tempComplex1, tempComplexReference);

      fft.setCoefficients(trajectory2.getCoefficients(), trajectory2.getNumberOfCoefficients());
      tempComplexReference = fft.getForwardTransform();
      ComplexNumber.copyComplexArray(tempComplex2, tempComplexReference);

      for (int i = 0; i < tempComplex1.length; i++)
         tempComplex1[i].timesAndStore(tempComplex2[i]);

      fft.setCoefficients(tempComplex1);
      tempComplexReference = fft.getInverseTransform();

      for (int i = 0; i < numberOfCoeffsToSet; i++)
         trajectoryToPack.setDirectlyFast(i, tempComplexReference[i].real() + trajectoryToPack.getCoefficient(i));
      trajectoryToPack.reshape(numberOfCoeffsToSet);
   }

   private void subtractCoefficientsByMultiplication(Trajectory trajectoryToPack, Trajectory trajectory1, Trajectory trajectory2)
   {
      int numberOfCoeffsToSet = trajectory1.getNumberOfCoefficients() + trajectory2.getNumberOfCoefficients() - 1;

      fft.setCoefficients(trajectory1.getCoefficients(), trajectory1.getNumberOfCoefficients());
      ComplexNumber[] tempComplexReference = fft.getForwardTransform();
      ComplexNumber.copyComplexArray(tempComplex1, tempComplexReference);

      fft.setCoefficients(trajectory2.getCoefficients(), trajectory2.getNumberOfCoefficients());
      tempComplexReference = fft.getForwardTransform();
      ComplexNumber.copyComplexArray(tempComplex2, tempComplexReference);

      for (int i = 0; i < tempComplex1.length; i++)
         tempComplex1[i].timesAndStore(tempComplex2[i]);

      fft.setCoefficients(tempComplex1);
      tempComplexReference = fft.getInverseTransform();

      for (int i = 0; i < numberOfCoeffsToSet; i++)
         trajectoryToPack.setDirectlyFast(i, trajectoryToPack.getCoefficient(i) - tempComplexReference[i].real());
      trajectoryToPack.reshape(numberOfCoeffsToSet);
   }

   public static void scale(Trajectory3D trajToPack, Trajectory3D traj, double scalarX, double scalarY, double scalarZ)
   {
      scale(trajToPack.getTrajectoryX(), traj.getTrajectoryX(), scalarX);
      scale(trajToPack.getTrajectoryY(), traj.getTrajectoryY(), scalarY);
      scale(trajToPack.getTrajectoryZ(), traj.getTrajectoryZ(), scalarZ);
   }

   private static void scale(Trajectory3D traj, double scalarX, double scalarY, double scalarZ)
   {
      scale(traj.getTrajectoryX(), scalarX);
      scale(traj.getTrajectoryY(), scalarY);
      scale(traj.getTrajectoryZ(), scalarZ);
   }

   public static void scale(Trajectory3D trajToPack, Trajectory3D traj, double scalar)
   {
      scale(trajToPack, traj, scalar, scalar, scalar);
   }

   public static void scale(double scalar, Trajectory3D traj)
   {
      scale(traj, scalar, scalar, scalar);
   }

   public static void add(Trajectory3D trajToPack, Trajectory3D traj1, Trajectory3D traj2)
   {
      for (int direction = 0; direction < 3; direction++)
         add(trajToPack.getTrajectory(direction), traj1.getTrajectory(direction), traj2.getTrajectory(direction));
   }

   public static void subtract(Trajectory3D trajToPack, Trajectory3D traj1, Trajectory3D traj2)
   {
      if (trajToPack == traj1)
         throw new RuntimeException("Cannot set to the container.");

      for (int direction = 0; direction < 3; direction++)
         subtract(trajToPack.getTrajectory(direction), traj1.getTrajectory(direction), traj2.getTrajectory(direction));
   }

   public static void subtractEquals(Trajectory3D traj1, Trajectory3D traj2)
   {
      for (int direction = 0; direction < 3; direction++)
         subtractEquals(traj1.getTrajectory(direction), traj2.getTrajectory(direction));
   }

   public static void subtractByTrimming(Trajectory3D trajToPack, Trajectory3D traj1, Trajectory3D traj2)
   {
      for (int direction = 0; direction < 3; direction++)
         subtractByTrimming(trajToPack.getTrajectory(direction), traj1.getTrajectory(direction), traj2.getTrajectory(direction));
   }

   public void dotProduct(Trajectory3D trajToPack, Trajectory3D traj1, Trajectory3D traj2)
   {
      for (int direction = 0; direction < 3; direction++)
         multiply(trajToPack.getTrajectory(direction), traj1.getTrajectory(direction), traj2.getTrajectory(direction));
   }

   public void dotProductByTrimming(Trajectory3D trajToPack, Trajectory3D traj1, Trajectory3D traj2)
   {
      for (int direction = 0; direction < 3; direction++)
         multiplyByTrimming(trajToPack.getTrajectory(direction), traj1.getTrajectory(direction), traj2.getTrajectory(direction));
   }

   public void dotProduct(Trajectory trajToPackX, Trajectory trajToPackY, Trajectory trajToPackZ, Trajectory traj1X, Trajectory traj1Y, Trajectory traj1Z,
                          Trajectory traj2X, Trajectory traj2Y, Trajectory traj2Z)
   {
      multiply(trajToPackX, traj1X, traj2X);
      multiply(trajToPackY, traj1Y, traj2Y);
      multiply(trajToPackZ, traj1Z, traj2Z);
   }

   public void dotProductByTrimming(Trajectory trajToPackX, Trajectory trajToPackY, Trajectory trajToPackZ, Trajectory traj1X, Trajectory traj1Y,
                                    Trajectory traj1Z, Trajectory traj2X, Trajectory traj2Y, Trajectory traj2Z)
   {
      multiplyByTrimming(trajToPackX, traj1X, traj2X);
      multiplyByTrimming(trajToPackY, traj1Y, traj2Y);
      multiplyByTrimming(trajToPackZ, traj1Z, traj2Z);
   }

   /**
    * trajToPack = traj1 x traj2
    * @param trajToPack
    * @param traj1
    * @param traj2
    */
   public void crossProduct(Trajectory3D trajToPack, Trajectory3D traj1, Trajectory3D traj2)
   {
      if (trajToPack == traj1)
         throw new RuntimeException("Trajectory to pack must be different from one of its inputs.");

      multiply(trajToPack.xTrajectory, traj1.yTrajectory, traj2.zTrajectory);
      multiplySubtract(trajToPack.xTrajectory, traj1.zTrajectory, traj2.yTrajectory);

      multiply(trajToPack.yTrajectory, traj1.zTrajectory, traj2.xTrajectory);
      multiplySubtract(trajToPack.yTrajectory, traj1.xTrajectory, traj2.zTrajectory);

      multiply(trajToPack.zTrajectory, traj1.xTrajectory, traj2.yTrajectory);
      multiplySubtract(trajToPack.zTrajectory, traj1.yTrajectory, traj2.xTrajectory);
   }

   /**
    * traj1 = traj1 x traj2;
    * @param traj1
    * @param traj2
    */
   public void crossProduct(Trajectory3D traj1, Trajectory3D traj2)
   {
      multiply(tempTraj3D.xTrajectory, traj1.yTrajectory, traj2.zTrajectory);
      multiplySubtract(tempTraj3D.xTrajectory, traj1.zTrajectory, traj2.yTrajectory);

      multiply(tempTraj3D.yTrajectory, traj1.zTrajectory, traj2.xTrajectory);
      multiplySubtract(tempTraj3D.yTrajectory, traj1.xTrajectory, traj2.zTrajectory);

      multiply(tempTraj3D.zTrajectory, traj1.xTrajectory, traj2.yTrajectory);
      multiplySubtract(tempTraj3D.zTrajectory, traj1.yTrajectory, traj2.xTrajectory);

      traj1.xTrajectory.set(tempTraj3D.xTrajectory);
      traj1.yTrajectory.set(tempTraj3D.yTrajectory);
      traj1.zTrajectory.set(tempTraj3D.zTrajectory);
   }

   /*
    * public void crossProductByTrimming(Trajectory3D trajToPack, Trajectory3D
    * traj1, Trajectory3D traj2) {
    * crossProductByTrimming(trajToPack.getTrajectoryX(),
    * trajToPack.getTrajectoryY(), trajToPack.getTrajectoryZ(),
    * traj1.getTrajectoryX(), traj1.getTrajectoryY(), traj1.getTrajectoryZ(),
    * traj2.getTrajectoryX(), traj2.getTrajectoryY(), traj2.getTrajectoryZ()); }
    * public void crossProductByTrimming(Trajectory trajToPackX, Trajectory
    * trajToPackY, Trajectory trajToPackZ, Trajectory traj1X, Trajectory traj1Y,
    * Trajectory traj1Z, Trajectory traj2X, Trajectory traj2Y, Trajectory
    * traj2Z) { multiplyByTrimming(tempTraj1, traj1Y, traj2Z);
    * multiplyByTrimming(tempTraj2, traj1Z, traj2Y);
    * subtractByTrimming(tempTraj3.xTrajectory, tempTraj1, tempTraj2);
    * multiplyByTrimming(tempTraj1, traj1X, traj2Z);
    * multiplyByTrimming(tempTraj2, traj1Z, traj2X);
    * subtractByTrimming(tempTraj3.yTrajectory, tempTraj2, tempTraj1);
    * multiplyByTrimming(tempTraj1, traj1X, traj2Y);
    * multiplyByTrimming(tempTraj2, traj1Y, traj2X);
    * subtractByTrimming(tempTraj3.zTrajectory, tempTraj1, tempTraj2);
    * trajToPackX.set(tempTraj3.xTrajectory);
    * trajToPackY.set(tempTraj3.yTrajectory);
    * trajToPackZ.set(tempTraj3.zTrajectory); }
    */

   public static void validateTrajectoryTimes(Trajectory traj1, Trajectory traj2)
   {
      if (Math.abs(traj1.getInitialTime() - traj2.getInitialTime()) > Epsilons.ONE_THOUSANDTH
            || Math.abs(traj1.getFinalTime() - traj2.getFinalTime()) > Epsilons.ONE_THOUSANDTH)
      {
         PrintTools.warn("Time mismatch in trajectories being added");
         throw new InvalidParameterException();
      }
   }

   private static void setTimeIntervalByTrimming(Trajectory segmentToPack, Trajectory segment1, Trajectory segment2)
   {
      double latestStartingTime = Math.max(segment1.getInitialTime(), segment2.getInitialTime());
      double earliestEndingTime = Math.min(segment1.getFinalTime(), segment2.getFinalTime());
      if (earliestEndingTime <= latestStartingTime)
      {
         PrintTools.debug(segment1.toString());
         PrintTools.debug(segment2.toString());
         throw new RuntimeException("Got null intersection for time intervals during trajectory operation");
      }
      segmentToPack.setInitialTime(latestStartingTime);
      segmentToPack.setFinalTime(earliestEndingTime);
   }

   private static void validatePackingTrajectoryForLinearCombination(Trajectory trajToPack, Trajectory traj1, Trajectory traj2)
   {
      if (trajToPack.getMaximumNumberOfCoefficients() < Math.max(traj1.getNumberOfCoefficients(), traj2.getNumberOfCoefficients()))
      {
         PrintTools.warn("Not enough coefficients to store result of trajectory operation. Needed: " + Math
               .max(traj1.getNumberOfCoefficients(), traj2.getNumberOfCoefficients()) + " Available: " + trajToPack.getMaximumNumberOfCoefficients());
         throw new InvalidParameterException();
      }
   }

   public static void validatePackingTrajectoryForLinearCombination(Trajectory traj1, Trajectory traj2)
   {
      if (traj1.getNumberOfCoefficients() != traj2.getNumberOfCoefficients())
      {
         PrintTools.warn("Inequal number of coefficients. Traj1: " + traj1.getNumberOfCoefficients() + ", Traj2: " + traj2.getNumberOfCoefficients());
         throw new InvalidParameterException();
      }
   }

   public static void validatePackingTrajectoryForMultiplication(Trajectory trajToPack, Trajectory traj1, Trajectory traj2)
   {
      if (trajToPack.getMaximumNumberOfCoefficients() < traj1.getNumberOfCoefficients() + traj2.getNumberOfCoefficients() - 1)
      {
         PrintTools.warn("Not enough coefficients to store result of trajectory multplication");
         throw new InvalidParameterException();
      }
   }

   /**
    * Add trajectories that do not have the same initial and final time. Trajectories added are assumed to be zero where they are not defined 
    * @param trajListToPack
    * @param traj1
    * @param traj2
    * @param TIME_EPSILON
    */
   public int add(List<Trajectory> trajListToPack, Trajectory traj1, Trajectory traj2, double TIME_EPSILON)
   {
      checkZeroTimeTrajectory(traj1, TIME_EPSILON);
      checkZeroTimeTrajectory(traj2, TIME_EPSILON);
      int numberOfSegments = getSegmentTimeList(tempTimeList, traj1, traj2, TIME_EPSILON);
      for (int i = 0; i < numberOfSegments; i++)
      {
         Trajectory segmentTrajToPack = trajListToPack.get(i);
         setCurrentSegmentPolynomial(tempTraj1, traj1, tempTimeList.get(i), tempTimeList.get(i + 1), TIME_EPSILON);
         setCurrentSegmentPolynomial(tempTraj2, traj2, tempTimeList.get(i), tempTimeList.get(i + 1), TIME_EPSILON);
         add(segmentTrajToPack, tempTraj1, tempTraj2);
      }
      return numberOfSegments;
   }

   /**
    * Subtract trajectories that do not have the same initial and final time. Trajectories added are assumed to be zero where they are not defined 
    * @param trajListToPack
    * @param traj1
    * @param traj2
    * @param TIME_EPSILON
    */
   public int subtract(List<Trajectory> trajListToPack, Trajectory traj1, Trajectory traj2, double TIME_EPSILON)
   {
      checkZeroTimeTrajectory(traj1, TIME_EPSILON);
      checkZeroTimeTrajectory(traj2, TIME_EPSILON);
      int numberOfSegments = getSegmentTimeList(tempTimeList, traj1, traj2, TIME_EPSILON);
      for (int i = 0; i < numberOfSegments; i++)
      {
         Trajectory segmentTrajToPack = trajListToPack.get(i);
         setCurrentSegmentPolynomial(tempTraj1, traj1, tempTimeList.get(i), tempTimeList.get(i + 1), TIME_EPSILON);
         setCurrentSegmentPolynomial(tempTraj2, traj2, tempTimeList.get(i), tempTimeList.get(i + 1), TIME_EPSILON);
         subtract(segmentTrajToPack, tempTraj1, tempTraj2);
      }
      return numberOfSegments;
   }

   /**
    * Multiply trajectories that do not have the same initial and final time. Trajectories added are assumed to be zero where they are not defined 
    * @param trajListToPack
    * @param traj1
    * @param traj2
    * @param TIME_EPSILON
    */
   public int multiply(List<Trajectory> trajListToPack, Trajectory traj1, Trajectory traj2, double TIME_EPSILON)
   {
      checkZeroTimeTrajectory(traj1, TIME_EPSILON);
      checkZeroTimeTrajectory(traj2, TIME_EPSILON);
      int numberOfSegments = getSegmentTimeList(tempTimeList, traj1, traj2, TIME_EPSILON);
      for (int i = 0; i < numberOfSegments; i++)
      {
         Trajectory segmentTrajToPack = trajListToPack.get(i);
         setCurrentSegmentPolynomial(tempTraj1, traj1, tempTimeList.get(i), tempTimeList.get(i + 1), TIME_EPSILON);
         setCurrentSegmentPolynomial(tempTraj2, traj2, tempTimeList.get(i), tempTimeList.get(i + 1), TIME_EPSILON);
         multiply(segmentTrajToPack, tempTraj1, tempTraj2);
      }
      return numberOfSegments;
   }

   private static void setCurrentSegmentPolynomial(Trajectory trajToPack, Trajectory traj, double segmentStartTime, double segmentFinalTime,
                                                   double TIME_EPSILON)
   {
      trajToPack.set(traj);
      trajToPack.setInitialTime(segmentStartTime);
      trajToPack.setFinalTime(segmentFinalTime);
      if (traj.getInitialTime() > segmentStartTime + TIME_EPSILON || traj.getFinalTime() < segmentFinalTime - TIME_EPSILON)
         trajToPack.setZero();
   }

   static void setCurrentSegmentPolynomial(Trajectory3D trajToPack, Trajectory3D traj, double segmentStartTime, double segmentFinalTime,
                                                   double TIME_EPSILON)
   {
      for (int i = 0; i < 3; i++)
         setCurrentSegmentPolynomial(trajToPack.getTrajectory(i), traj.getTrajectory(i), segmentStartTime, segmentFinalTime, TIME_EPSILON);
   }

   public static int getSegmentTimeList(TDoubleArrayList trajTimeListToPack, Trajectory traj1, Trajectory traj2, double TIME_EPSILON)
   {
      trajTimeListToPack.clear();
      trajTimeListToPack.add(traj1.getInitialTime());
      trajTimeListToPack.add(traj1.getFinalTime());
      trajTimeListToPack.add(traj2.getInitialTime());
      trajTimeListToPack.add(traj2.getFinalTime());

      int tempTimeArrayLength = trajTimeListToPack.size();
      for (int i = 0, j = 0, k = 2; k < tempTimeArrayLength; i++)
      {
         if (Math.abs(trajTimeListToPack.get(j) - trajTimeListToPack.get(k)) < TIME_EPSILON)
         {
            tempTimeArrayLength--;
            trajTimeListToPack.set(k, trajTimeListToPack.get(tempTimeArrayLength));
            trajTimeListToPack.set(tempTimeArrayLength, Double.POSITIVE_INFINITY);
         }
         if (trajTimeListToPack.get(j) > trajTimeListToPack.get(k))
         {
            trajTimeListToPack.set(j, trajTimeListToPack.get(j) + trajTimeListToPack.get(k));
            trajTimeListToPack.set(k, trajTimeListToPack.get(j) - trajTimeListToPack.get(k));
            trajTimeListToPack.set(j, trajTimeListToPack.get(j) - trajTimeListToPack.get(k));
         }
         j++;
         k += i;
      }
      if (trajTimeListToPack.get(1) > trajTimeListToPack.get(2))
      {
         trajTimeListToPack.set(1, trajTimeListToPack.get(1) + trajTimeListToPack.get(2));
         trajTimeListToPack.set(2, trajTimeListToPack.get(1) - trajTimeListToPack.get(2));
         trajTimeListToPack.set(1, trajTimeListToPack.get(1) - trajTimeListToPack.get(2));
      }
      return tempTimeArrayLength - 1;
   }

   public static void checkZeroTimeTrajectory(Trajectory trajectory, double TIME_EPSILON)
   {
      if (Math.abs(trajectory.getFinalTime() - trajectory.getInitialTime()) < TIME_EPSILON)
         throw new RuntimeException(
               "Cannot operate with null trajectory, start time: " + trajectory.getInitialTime() + " end time: " + trajectory.getFinalTime() + " epsilon: "
                     + TIME_EPSILON);
   }

   public void addTimeOffset(Trajectory trajectory, double timeOffset)
   {
      int index = 1;
      int n = trajectory.getNumberOfCoefficients();
      double factorial = 1;
      double power = 1;
      for (index = 0; index < maxNumberOfCoefficients; index++)
      {
         tempComplex1[index].setToZero();
         tempComplex2[index].setToZero();
      }

      for (index = 1; index <= n; index++)
      {
         tempComplex1[index - 1].setToPurelyReal(power / factorial);
         tempComplex2[n - index].setToPurelyReal(factorial * trajectory.getCoefficient(index - 1));
         power *= -timeOffset;
         factorial *= (index);
      }

      fft.setCoefficients(tempComplex1);
      ComplexNumber[] tempComplexReference = fft.getForwardTransform();
      ComplexNumber.copyComplexArray(tempComplex1, tempComplexReference);

      fft.setCoefficients(tempComplex2);
      tempComplexReference = fft.getForwardTransform();
      ComplexNumber.copyComplexArray(tempComplex2, tempComplexReference);

      for (int i = 0; i < tempComplex1.length; i++)
         tempComplex1[i].timesAndStore(tempComplex2[i]);

      fft.setCoefficients(tempComplex1);
      tempComplexReference = fft.getInverseTransform();

      factorial = 1;
      for (index = 1; index <= n; index++)
      {
         trajectory.setDirectlyFast(index - 1, tempComplexReference[n - index].real() / factorial);
         factorial *= index;
      }
      trajectory.setTime(trajectory.getInitialTime() + timeOffset, trajectory.getFinalTime() + timeOffset);
   }

   public static void getIntegral(Trajectory trajectoryToPack, Trajectory trajectoryToIntegrate)
   {
      if (trajectoryToPack.getMaximumNumberOfCoefficients() < trajectoryToIntegrate.getNumberOfCoefficients() + 1)
         throw new InvalidParameterException("Not enough coefficients to store result of trajectory integration");

      for (int i = trajectoryToIntegrate.getNumberOfCoefficients(); i > 0; i--)
         trajectoryToPack.setDirectlyFast(i, trajectoryToIntegrate.getCoefficient(i - 1) / (i));
      trajectoryToIntegrate.compute(trajectoryToIntegrate.getInitialTime());
      double position = trajectoryToIntegrate.getPosition();
      trajectoryToPack.setDirectly(0, -position);
      trajectoryToPack.reshape(trajectoryToIntegrate.getNumberOfCoefficients() + 1);
      trajectoryToPack.setTime(trajectoryToIntegrate.getInitialTime(), trajectoryToIntegrate.getFinalTime());
   }

   public static void getDerivative(Trajectory derivativeToPack, Trajectory trajectoryToDifferentiate)
   {
      if (derivativeToPack.getMaximumNumberOfCoefficients() < trajectoryToDifferentiate.getNumberOfCoefficients() - 1)
      {
         throw new InvalidParameterException("Not enough coefficients to store the result of differentiation");
      }

      derivativeToPack.reshape(Math.max(trajectoryToDifferentiate.getNumberOfCoefficients() - 1, 1));
      if (trajectoryToDifferentiate.getNumberOfCoefficients() == 1)
      {
         derivativeToPack.setConstant(trajectoryToDifferentiate.getInitialTime(), trajectoryToDifferentiate.getFinalTime(), 0);
         return;
      }

      for (int i = trajectoryToDifferentiate.getNumberOfCoefficients() - 1; i > 0; i--)
      {
         derivativeToPack.setDirectlyFast(i - 1, i * trajectoryToDifferentiate.getCoefficient(i));
      }
      derivativeToPack.setTime(trajectoryToDifferentiate.getInitialTime(), trajectoryToDifferentiate.getFinalTime());
   }

   public static void addSegmentedTrajectories(SegmentedFrameTrajectory3D trajectoryToPack, SegmentedFrameTrajectory3D trajectory1,
                                               SegmentedFrameTrajectory3D trajectory2, double epsilon)
   {
      addSegmentedTrajectories(trajectoryToPack, trajectory1, trajectory2, 0.0, epsilon);
   }

   public static void addSegmentedTrajectories(SegmentedFrameTrajectory3D trajectoryToPack, SegmentedFrameTrajectory3D trajectory1,
                                               SegmentedFrameTrajectory3D trajectory2, double minimumSegmentLength, double epsilon)
   {
      trajectoryToPack.reset();
      double currentTime = Math.min(trajectory1.getSegment(0).getInitialTime(), trajectory2.getSegment(0).getInitialTime());
      for (int trajectory1Index = 0, trajectory2Index = 0;
           trajectory1Index < trajectory1.getNumberOfSegments() || trajectory2Index < trajectory2.getNumberOfSegments(); )
      {
         FrameTrajectory3D firstSegment, secondSegment;
         FrameTrajectory3D currentTrajectoryToPack = trajectoryToPack.add();

         boolean trajectory2SegmentBeforeTrajectory1 = false;
         boolean trajectory1SegmentBeforeTrajectory2 = false;
         boolean trajectory1IsOver = trajectory1Index >= trajectory1.getNumberOfSegments();
         boolean trajectory2IsOver = trajectory2Index >= trajectory2.getNumberOfSegments();

         boolean trajectory1IsFirst;
         if (!trajectory1IsOver)
         {
            trajectory2SegmentBeforeTrajectory1 = (trajectory2Index < trajectory2.getNumberOfSegments() && (
                  trajectory2.getSegment(trajectory2Index).getFinalTime() < trajectory1.getSegment(trajectory1Index).getInitialTime() - epsilon));
         }
         if (!trajectory2IsOver)
         {
            trajectory1SegmentBeforeTrajectory2 = (trajectory1Index < trajectory1.getNumberOfSegments() && (
                  trajectory1.getSegment(trajectory1Index).getFinalTime() < trajectory2.getSegment(trajectory2Index).getInitialTime() - epsilon));
         }

         // Select the one that is ahead or set if no intersection
         if (trajectory1IsOver)
         { // No intersection because the first trajectory is over
            setCurrentSegmentPolynomial(currentTrajectoryToPack, trajectory2.getSegment(trajectory2Index), currentTime,
                                        trajectory2.getSegment(trajectory2Index).getFinalTime(), epsilon);
            currentTime = trajectory2.getSegment(trajectory2Index++).getFinalTime();
            continue;
         }
         else if (trajectory2SegmentBeforeTrajectory1)
         { // No intersection. The second trajectory segment ends before the first trajectory segment starts
            FrameTrajectory3D trajectory2Segment = trajectory2.getSegment(trajectory2Index);
            setCurrentSegmentPolynomial(currentTrajectoryToPack, trajectory2Segment, currentTime, trajectory2Segment.getFinalTime(), epsilon);
            currentTime = trajectory2Segment.getFinalTime();
            trajectory2Index++;
            continue;
         }
         else if (trajectory2IsOver || trajectory1SegmentBeforeTrajectory2)
         { // No intersection. Either the second trajectory is over, or the first trajectory segment ends before the second trajectory segment starts
            setCurrentSegmentPolynomial(currentTrajectoryToPack, trajectory1.getSegment(trajectory1Index), currentTime,
                                        trajectory1.getSegment(trajectory1Index).getFinalTime(), epsilon);
            currentTime = trajectory1.getSegment(trajectory1Index++).getFinalTime();
            continue;
         }
         else if (trajectory1.getSegment(trajectory1Index).getInitialTime() < trajectory2.getSegment(trajectory2Index).getInitialTime())
         { // trajectory 1 segment starts first
            firstSegment = trajectory1.getSegment(trajectory1Index);
            secondSegment = trajectory2.getSegment(trajectory2Index);
            trajectory1IsFirst = true;
         }
         else
         { // trajectory 2 segment starts first, or they both start at the same time
            secondSegment = trajectory1.getSegment(trajectory1Index);
            firstSegment = trajectory2.getSegment(trajectory2Index);
            trajectory1IsFirst = false;
         }

         boolean semgentsDontStartAtTheSameTime = firstSegment.getInitialTime() < secondSegment.getInitialTime() - epsilon;
         boolean readingFromBeforeSecondSegment = currentTime < secondSegment.getInitialTime() - epsilon;
         boolean startDifferenceIsTooSmall = secondSegment.getInitialTime() - firstSegment.getInitialTime() < minimumSegmentLength;
         if (semgentsDontStartAtTheSameTime && readingFromBeforeSecondSegment)
         { // the first segment starts before the second segment, so add that first bit in, as it does not get included in the add by trimming function
            if (startDifferenceIsTooSmall)
            { // its too short to change anything
               secondSegment.setInitialTimeMaintainingBounds(firstSegment.getInitialTime());
            }
            else
            {
               setCurrentSegmentPolynomial(currentTrajectoryToPack, firstSegment, currentTime, secondSegment.getInitialTime(), epsilon);
               currentTrajectoryToPack = trajectoryToPack.add();
            }
         }

         boolean segmentsDontEndAtTheSameTime = secondSegment.getFinalTime() > firstSegment.getFinalTime() + epsilon;
         boolean endDifferenceIsTooSmall = secondSegment.getFinalTime() - firstSegment.getFinalTime() < minimumSegmentLength;
         if (segmentsDontEndAtTheSameTime && endDifferenceIsTooSmall)
         { // they don't end at the same time, but the difference is too small to include, so stretch the first one to the end one.
            firstSegment.setFinalTimeMaintainingBounds(secondSegment.getFinalTime());

            // need to shift the time of the next segment forward, too
            if (trajectory1IsFirst && trajectory1Index + 1 < trajectory1.getNumberOfSegments())
               trajectory1.getSegment(trajectory1Index + 1).setInitialTimeMaintainingBounds(secondSegment.getFinalTime());
            else if (!trajectory1IsFirst && trajectory2Index + 1 < trajectory2.getNumberOfSegments())
               trajectory2.getSegment(trajectory2Index + 1).setInitialTimeMaintainingBounds(secondSegment.getFinalTime());
         }

         addByTrimming(currentTrajectoryToPack, firstSegment, secondSegment);

         currentTime = Math.min(firstSegment.getFinalTime(), secondSegment.getFinalTime());

         if (currentTime < trajectory2.getSegment(trajectory2Index).getFinalTime() - epsilon)
         { // we haven't reached the end of trajectory 2, so only advance trajectory 1
            trajectory1Index++;
         }
         else if (currentTime < trajectory1.getSegment(trajectory1Index).getFinalTime() - epsilon)
         //         else if (currentTime < trajectory1.getSegment(trajectory2Index).getFinalTime() - epsilon) // FIXME why does this work?
         { // we haven't reached the end of trajectory 1, so only advance trajectory 2
            trajectory2Index++;
         }
         else
         { // we've reached the end of both trajectories, so advance them both.
            trajectory1Index++;
            trajectory2Index++;
         }
      }
   }

   public static void getDerivative(FrameTrajectory3D trajectoryToPack, FrameTrajectory3D trajectoryToDifferentiate)
   {
      for (int i = 0; i < 3; i++)
         getDerivative(trajectoryToPack.getTrajectory(i), trajectoryToDifferentiate.getTrajectory(i));
   }

   public static boolean epsilonEquals(FrameTrajectory3D trajectory1, FrameTrajectory3D trajectory2, double epsilon)
   {
      return (trajectory1.getReferenceFrame() == trajectory2.getReferenceFrame() && epsilonEquals((Trajectory3D) trajectory1, (Trajectory3D) trajectory2,
                                                                                                  epsilon));
   }

   public static boolean epsilonEquals(Trajectory3D trajectory1, Trajectory3D trajectory2, double epsilon)
   {
      return (epsilonEquals(trajectory1.getTrajectoryX(), trajectory2.getTrajectoryX(), epsilon) && epsilonEquals(trajectory1.getTrajectoryY(),
                                                                                                                  trajectory2.getTrajectoryY(), epsilon)
            && epsilonEquals(trajectory1.getTrajectoryZ(), trajectory2.getTrajectoryZ(), epsilon));
   }

   public static boolean epsilonEquals(Trajectory trajectory1, Trajectory trajectory2, double epsilon)
   {
      boolean result = true;
      if (!MathTools.epsilonEquals(trajectory1.getInitialTime(), trajectory2.getInitialTime(), epsilon))
         return false;
      if (!MathTools.epsilonEquals(trajectory1.getFinalTime(), trajectory2.getFinalTime(), epsilon))
         return false;
      for (int i = 0; i < 3; i++)
      {
         result &= trajectory1.getNumberOfCoefficients() == trajectory2.getNumberOfCoefficients();
         if (result)
         {
            for (int j = 0; j < trajectory1.getNumberOfCoefficients(); j++)
               result &= MathTools.epsilonEquals(trajectory1.getCoefficient(j), trajectory2.getCoefficient(j), epsilon);
         }
         else
            return false;
      }

      return result;
   }

   public static void removeShortSegments(SegmentedFrameTrajectory3D trajectory, double minimumSegmentDuration)
   {
      int numberOfSegments = trajectory.getNumberOfSegments();

      // check last segment duration
      while (trajectory.getSegment(numberOfSegments - 1).getDuration() < minimumSegmentDuration && numberOfSegments > 1)
      { // stretch time of previous segment end forward
         double finalTime = trajectory.getSegment(numberOfSegments - 1).getFinalTime();
         trajectory.getSegment(numberOfSegments - 2).setFinalTimeMaintainingBounds(finalTime);
         trajectory.removeSegment(numberOfSegments - 1);
         numberOfSegments = trajectory.getNumberOfSegments();
      }

      // check first segment duration
      while (trajectory.getSegment(0).getDuration() < minimumSegmentDuration && numberOfSegments > 1)
      { // pull time of next segment start back
         double initialTime = trajectory.getSegment(0).getInitialTime();
         trajectory.getSegment(1).setInitialTimeMaintainingBounds(initialTime);
         trajectory.removeSegment(0);
         numberOfSegments = trajectory.getNumberOfSegments();
      }


      // iterate over all the internal segments
      int currentSegmentIndex = 1;
      while (currentSegmentIndex < trajectory.getNumberOfSegments())
      {
         while (trajectory.getSegment(currentSegmentIndex).getDuration() < minimumSegmentDuration && currentSegmentIndex < trajectory.getNumberOfSegments())
         {
             boolean hasNextSegment = currentSegmentIndex < trajectory.getNumberOfSegments() - 1;
             boolean nextSegmentIsLarger = hasNextSegment && trajectory.getSegment(currentSegmentIndex - 1).getDuration() < trajectory.getSegment(currentSegmentIndex + 1).getDuration();

             if (nextSegmentIsLarger)
             { // stretch time of previous segment end forward
                double finalTime = trajectory.getSegment(currentSegmentIndex).getFinalTime();
                trajectory.getSegment(currentSegmentIndex - 1).setFinalTimeMaintainingBounds(finalTime);
             }
             else
             { // pull time of previous segment start back
                double initialTime = trajectory.getSegment(currentSegmentIndex).getInitialTime();
                trajectory.getSegment(currentSegmentIndex + 1).setInitialTimeMaintainingBounds(initialTime);
             }
            trajectory.removeSegment(currentSegmentIndex);
         }

         currentSegmentIndex++;
      }
   }

   private final RecyclingArrayList<Point3D> initialBounds = new RecyclingArrayList<>(Point3D::new);
   private final RecyclingArrayList<Point3D> midpointBounds = new RecyclingArrayList<>(Point3D::new);
   private final RecyclingArrayList<Point3D> finalBounds = new RecyclingArrayList<>(Point3D::new);


   public void resampleTrajectoryToMatchWaypoints(SegmentedFrameTrajectory3D trajectoryToResample, SegmentedFrameTrajectory3D trajectoryToMatch, double minimumTimeDeltaForResample)
   {
      stretchTrajectoriesToMatchBounds(trajectoryToResample, trajectoryToMatch);

      for (int currentSegmentIndex = 0; currentSegmentIndex < trajectoryToResample.getNumberOfSegments() - 1; currentSegmentIndex++)
      {
         double waypointTime = trajectoryToResample.getSegment(currentSegmentIndex).getFinalTime();
         int closestOtherKnotIndex = 0;
         double minDistanceToOtherKnot = Double.POSITIVE_INFINITY;
         for (int otherKnotIndex = 0; otherKnotIndex < trajectoryToMatch.getNumberOfSegments() - 1; otherKnotIndex++)
         {
            double timeDelta = Math.abs(trajectoryToMatch.getSegment(otherKnotIndex).getFinalTime() - waypointTime);
            if (timeDelta < minDistanceToOtherKnot)
            {
               minDistanceToOtherKnot = timeDelta;
               closestOtherKnotIndex = otherKnotIndex;
            }
         }

         if (minDistanceToOtherKnot > minimumTimeDeltaForResample)
            continue;


         double timeToResampleTo = trajectoryToMatch.getSegment(closestOtherKnotIndex).getFinalTime();
         FrameTrajectory3D nextTrajectory = trajectoryToResample.getSegment(currentSegmentIndex + 1);
         FrameTrajectory3D currentTrajectory = trajectoryToResample.getSegment(currentSegmentIndex);
         FrameTrajectory3D trajectoryAtTime = trajectoryToResample.getCurrentSegment(timeToResampleTo);

         double startTime = currentTrajectory.getInitialTime();
         double endTime = nextTrajectory.getFinalTime();

         int maxOrder = Math.max(currentTrajectory.getNumberOfCoefficients(), nextTrajectory.getNumberOfCoefficients());

         int currentNumberOfInitialBounds = Math.floorDiv(currentTrajectory.getNumberOfCoefficients(), 2);
         int currentNumberOfFinalBounds = maxOrder - currentNumberOfInitialBounds;
         int nextNumberOfFinalBounds = Math.floorDiv(currentTrajectory.getNumberOfCoefficients(), 2);
         int nextNumberOfInitialBounds = maxOrder - nextNumberOfFinalBounds;

         // get bounds
         initialBounds.clear();
         midpointBounds.clear();
         finalBounds.clear();

         for (int order = 0; order < currentNumberOfInitialBounds; order++)
         {
            for (Axis3D axis : Axis3D.values)
            {
               initialBounds.getAndGrowIfNeeded(order).setElement(axis.ordinal(), currentTrajectory.getDerivative(axis.ordinal(), order, startTime));
            }
         }
         for (int order = 0; order < Math.max(currentNumberOfFinalBounds, nextNumberOfInitialBounds); order++)
         {
            for (Axis3D axis : Axis3D.values)
            {
               midpointBounds.getAndGrowIfNeeded(order).setElement(axis.ordinal(), trajectoryAtTime.getDerivative(axis.ordinal(), order, timeToResampleTo));
            }
         }
         for (int order = 0; order < nextNumberOfFinalBounds; order++)
         {
            for (Axis3D axis : Axis3D.values)
            {
               finalBounds.getAndGrowIfNeeded(order).setElement(axis.ordinal(), nextTrajectory.getDerivative(axis.ordinal(), order, endTime));
            }
         }

         currentTrajectory.reshape(maxOrder);
         currentTrajectory.setTime(startTime, timeToResampleTo);

         int currentConstraintRow = 0;
         for (int initialConstraintOrder = 0; initialConstraintOrder < currentNumberOfInitialBounds; initialConstraintOrder++, currentConstraintRow++)
         {
            currentTrajectory.setConstraintRow(currentConstraintRow, startTime, initialBounds.get(initialConstraintOrder), initialConstraintOrder);
         }

         for (int finalConstraintOrder = 0; finalConstraintOrder < currentNumberOfFinalBounds; finalConstraintOrder++, currentConstraintRow++)
         {
            currentTrajectory.setConstraintRow(currentConstraintRow, timeToResampleTo, midpointBounds.get(finalConstraintOrder), finalConstraintOrder);
         }

         currentTrajectory.solveForCoefficients();
         currentTrajectory.setCoefficientVariables();

         nextTrajectory.reshape(maxOrder);
         nextTrajectory.setTime(timeToResampleTo, endTime);

         int nextConstraintRow = 0;
         for (int initialConstraintOrder = 0; initialConstraintOrder < nextNumberOfInitialBounds; initialConstraintOrder++, nextConstraintRow++)
         {
            nextTrajectory.setConstraintRow(nextConstraintRow, timeToResampleTo, midpointBounds.get(initialConstraintOrder), initialConstraintOrder);
         }

         for (int finalConstraintOrder = 0; finalConstraintOrder < nextNumberOfFinalBounds; finalConstraintOrder++, nextConstraintRow++)
         {
            nextTrajectory.setConstraintRow(nextConstraintRow, endTime, finalBounds.get(finalConstraintOrder), finalConstraintOrder);
         }

         nextTrajectory.solveForCoefficients();
         nextTrajectory.setCoefficientVariables();
      }
   }

   public static void stretchTrajectoriesToMatchBounds(SegmentedFrameTrajectory3D trajectory1, SegmentedFrameTrajectory3D trajectory2)
   {
      FrameTrajectory3D firstSegment1 = trajectory1.getFirstSegment();
      FrameTrajectory3D firstSegment2 = trajectory2.getFirstSegment();

      FrameTrajectory3D lastSegment1 = trajectory1.getLastSegment();
      FrameTrajectory3D lastSegment2 = trajectory2.getLastSegment();

      if (firstSegment1.getInitialTime() < firstSegment2.getInitialTime())
         firstSegment2.setInitialTimeMaintainingBounds(firstSegment1.getInitialTime());
      else
         firstSegment1.setInitialTimeMaintainingBounds(firstSegment2.getInitialTime());

      if (lastSegment1.getFinalTime() > lastSegment2.getFinalTime())
         lastSegment2.setFinalTimeMaintainingBounds(lastSegment1.getFinalTime());
      else
         lastSegment1.setFinalTimeMaintainingBounds(lastSegment2.getFinalTime());
   }

   public static void shrinkTrajectoriesToMatchBounds(SegmentedFrameTrajectory3D trajectory1, SegmentedFrameTrajectory3D trajectory2)
   {
      FrameTrajectory3D firstSegment1 = trajectory1.getFirstSegment();
      FrameTrajectory3D firstSegment2 = trajectory2.getFirstSegment();

      FrameTrajectory3D lastSegment1 = trajectory1.getLastSegment();
      FrameTrajectory3D lastSegment2 = trajectory2.getLastSegment();

      if (firstSegment1.getInitialTime() < firstSegment2.getInitialTime())
         firstSegment1.setInitialTimeMaintainingBounds(firstSegment2.getInitialTime());
      else
         firstSegment2.setInitialTimeMaintainingBounds(firstSegment1.getInitialTime());

      if (lastSegment1.getFinalTime() > lastSegment2.getFinalTime())
         lastSegment1.setFinalTimeMaintainingBounds(lastSegment2.getFinalTime());
      else
         lastSegment2.setFinalTimeMaintainingBounds(lastSegment1.getFinalTime());
   }


   public static void combineSegments(SegmentedFrameTrajectory3D trajectory, int firstSegmentIndex)
   {
      int secondSegmentIndex = firstSegmentIndex + 1;
      FrameTrajectory3D segmentToRemove = trajectory.getSegment(firstSegmentIndex);
      FrameTrajectory3D combinedSegment = trajectory.getSegment(secondSegmentIndex);
      double combinedStartTime = segmentToRemove.getInitialTime();
      double combinedFinalTime = combinedSegment.getFinalTime();
      int combinedNumberOfIndices = Math.max(segmentToRemove.getNumberOfCoefficients(), combinedSegment.getNumberOfCoefficients());

      segmentToRemove.compute(combinedStartTime);
      combinedSegment.compute(combinedFinalTime);

      if (combinedNumberOfIndices > 5)
      {
         combinedSegment.setQuintic(combinedStartTime, combinedFinalTime, segmentToRemove.getPosition(), segmentToRemove.getVelocity(),
                                    segmentToRemove.getAcceleration(), combinedSegment.getPosition(), combinedSegment.getVelocity(),
                                    segmentToRemove.getAcceleration());
      }
      else if (combinedNumberOfIndices > 4)
      {
         combinedSegment.setQuartic(combinedStartTime, combinedFinalTime, segmentToRemove.getPosition(), segmentToRemove.getVelocity(),
                                    segmentToRemove.getAcceleration(), combinedSegment.getPosition(), combinedSegment.getVelocity());
      }
      else if (combinedNumberOfIndices > 3)
      {
         combinedSegment
               .setCubic(combinedStartTime, combinedFinalTime, segmentToRemove.getPosition(), segmentToRemove.getVelocity(), combinedSegment.getPosition(),
                         combinedSegment.getVelocity());
      }
      else if (combinedNumberOfIndices > 2)
      {
         combinedSegment
               .setQuadratic(combinedStartTime, combinedFinalTime, segmentToRemove.getPosition(), segmentToRemove.getVelocity(), combinedSegment.getPosition());
      }
      else if (combinedNumberOfIndices > 1)
      {
         combinedSegment.setLinear(combinedStartTime, combinedFinalTime, segmentToRemove.getPosition(), combinedSegment.getPosition());
         trajectory.removeSegment(firstSegmentIndex);
      }
      else if (combinedNumberOfIndices > 0)
      {
         combinedSegment.setConstant(combinedStartTime, combinedFinalTime, combinedSegment.getPosition());
         trajectory.removeSegment(firstSegmentIndex);
      }
      else
      {
         throw new RuntimeException("Yeah I don't know what to do here.");
      }
   }
}
