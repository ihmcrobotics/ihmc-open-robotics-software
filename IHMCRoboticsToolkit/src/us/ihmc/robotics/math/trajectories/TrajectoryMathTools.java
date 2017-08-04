package us.ihmc.robotics.math.trajectories;

import java.security.InvalidParameterException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.dataStructures.ComplexNumber;
import us.ihmc.robotics.math.FastFourierTransform;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class TrajectoryMathTools
{
   private static final int maxNumberOfCoefficients = 16;
   private static YoVariableRegistry testRegistry = new YoVariableRegistry("DummyRegistryForTrajectoryMath");
   private static YoTrajectory tempTraj1 = new YoTrajectory("TempTraj1", maxNumberOfCoefficients, testRegistry);
   private static YoTrajectory tempTraj2 = new YoTrajectory("TempTraj2", maxNumberOfCoefficients, testRegistry);
   private static YoTrajectory3D tempTraj3 = new YoTrajectory3D("TempTraj3D", maxNumberOfCoefficients, testRegistry);
   private static List<Double> tempTimeList = new ArrayList<>(Arrays.asList(0.0, 0.0, 0.0, 0.0));
   private static int tempTimeArrayLength = 4;
   private static FastFourierTransform fft = new FastFourierTransform(maxNumberOfCoefficients);
   private static ComplexNumber[] tempComplex1 = ComplexNumber.getComplexArray(maxNumberOfCoefficients);
   private static ComplexNumber[] tempComplex2 = ComplexNumber.getComplexArray(maxNumberOfCoefficients);
   private static ComplexNumber[] tempComplexReference;

   public static void scale(YoTrajectory scaledTrajectoryToPack, YoTrajectory trajectoryToScale, double scalar)
   {
      YoPolynomial polynomial = scaledTrajectoryToPack.getPolynomial();
      for (int i = 0; i < trajectoryToScale.getNumberOfCoefficients(); i++)
         polynomial.setDirectlyFast(i, trajectoryToScale.getCoefficient(i) * scalar);
   }

   /**
    * Add two trajectories that have the same start and end times. 
    * Throws runtime exception in case the start and end time values do not match
    * @param trajToPack
    * @param traj1
    * @param traj2
    */
   public static void add(YoTrajectory trajToPack, YoTrajectory traj1, YoTrajectory traj2)
   {
      validatePackingTrajectoryForLinearCombination(trajToPack, traj1, traj2);
      validateTrajectoryTimes(traj1, traj2);
      trajToPack.setTime(traj1.getInitialTime(), traj2.getFinalTime());
      setCoeffsByAddition(trajToPack, traj1, traj2);
   }

   /**
    * Adds two trajectories by taking the intersection of the time intervals over which they are defined.
    * Throws runtime exception in case null intersection is found between the two trajectories
    * @param trajToPack
    * @param traj1
    * @param traj2
    */
   public static void addByTrimming(YoTrajectory trajToPack, YoTrajectory traj1, YoTrajectory traj2)
   {
      validatePackingTrajectoryForLinearCombination(trajToPack, traj1, traj2);
      setTimeIntervalByTrimming(trajToPack, traj1, traj2);
      setCoeffsByAddition(trajToPack, traj1, traj2);
   }

   private static void setCoeffsByAddition(YoTrajectory trajectoryToPack, YoTrajectory trajectory1, YoTrajectory trajectory2)
   {
      int numberOfCoeffsToSet = Math.max(trajectory1.getNumberOfCoefficients(), trajectory2.getNumberOfCoefficients());
      YoPolynomial polynomial = trajectoryToPack.getPolynomial();

      for (int i = 0; i < numberOfCoeffsToSet; i++)
      {
         double coefficient = 0.0;
         if (i < trajectory1.getNumberOfCoefficients())
            coefficient += trajectory1.getCoefficient(i);
         if (i < trajectory2.getNumberOfCoefficients())
            coefficient += trajectory2.getCoefficient(i);
         polynomial.setDirectlyFast(i, coefficient);
      }
      polynomial.reshape(numberOfCoeffsToSet);
   }

   /**
    * Subtracts {@code traj2} from {@code traj1} in case the two have the same start and end times
    * Throws runtime exception in case the start and end time values do not match or if {@code trajToPack} is not large enough to store the result
    * @param trajToPack
    * @param traj1
    * @param traj2
    */
   public static void subtract(YoTrajectory trajToPack, YoTrajectory traj1, YoTrajectory traj2)
   {
      validatePackingTrajectoryForLinearCombination(trajToPack, traj1, traj2);
      validateTrajectoryTimes(traj1, traj2);
      trajToPack.setTime(traj1.getInitialTime(), traj2.getFinalTime());
      setCoeffsBySubtraction(trajToPack, traj1, traj2);
   }

   /**
    * Subtracts the two trajectories by taking the intersection of the time intervals over which the two are defined
    * @param trajToPack
    * @param traj1
    * @param traj2
    */
   public static void subtractByTrimming(YoTrajectory trajToPack, YoTrajectory traj1, YoTrajectory traj2)
   {
      validatePackingTrajectoryForLinearCombination(trajToPack, traj1, traj2);
      setTimeIntervalByTrimming(trajToPack, traj1, traj2);
      setCoeffsBySubtraction(trajToPack, traj1, traj2);
   }

   private static void setCoeffsBySubtraction(YoTrajectory trajectoryToPack, YoTrajectory trajectory1, YoTrajectory trajectory2)
   {
      int numberOfCoeffsToSet = Math.max(trajectory1.getNumberOfCoefficients(), trajectory2.getNumberOfCoefficients());
      YoPolynomial polynomial = trajectoryToPack.getPolynomial();
      for (int i = 0; i < numberOfCoeffsToSet; i++)
      {
         double coefficient = 0.0;
         if (i < trajectory1.getNumberOfCoefficients())
            coefficient += trajectory1.getCoefficient(i);
         if (i < trajectory2.getNumberOfCoefficients())
            coefficient -= trajectory2.getCoefficient(i);
         polynomial.setDirectlyFast(i, coefficient);
      }
      polynomial.reshape(numberOfCoeffsToSet);
   }

   public static void multiply(YoTrajectory trajToPack, YoTrajectory traj1, YoTrajectory traj2)
   {
      validatePackingTrajectoryForMultiplication(trajToPack, traj1, traj2);
      validateTrajectoryTimes(traj1, traj2);
      trajToPack.setTime(traj1.getInitialTime(), traj2.getFinalTime());
      setCoeffsByMultiplication(trajToPack, traj1, traj2);
   }

   public static void multiplyByTrimming(YoTrajectory trajToPack, YoTrajectory traj1, YoTrajectory traj2)
   {
      validatePackingTrajectoryForMultiplication(trajToPack, traj1, traj2);
      setTimeIntervalByTrimming(trajToPack, traj1, traj2);
      setCoeffsByMultiplication(trajToPack, traj1, traj2);
   }

   private static void setCoeffsByMultiplication(YoTrajectory trajectoryToPack, YoTrajectory trajectory1, YoTrajectory trajectory2)
   {
      int numberOfCoeffsToSet = trajectory1.getNumberOfCoefficients() + trajectory2.getNumberOfCoefficients() - 1;

      fft.setCoefficients(trajectory1.getCoefficients());
      tempComplexReference = fft.getForwardTransform();
      ComplexNumber.copyComplexArray(tempComplex1, tempComplexReference);

      fft.setCoefficients(trajectory2.getCoefficients());
      tempComplexReference = fft.getForwardTransform();
      ComplexNumber.copyComplexArray(tempComplex2, tempComplexReference);

      for (int i = 0; i < tempComplex1.length; i++)
         tempComplex1[i].timesAndStore(tempComplex2[i]);

      fft.setCoefficients(tempComplex1);
      tempComplexReference = fft.getInverseTransform();

      YoPolynomial polynomial = trajectoryToPack.getPolynomial();
      for (int i = 0; i < numberOfCoeffsToSet; i++)
         polynomial.setDirectlyFast(i, tempComplexReference[i].real());
      polynomial.reshape(numberOfCoeffsToSet);
   }

   public static void scale(YoTrajectory3D trajToPack, YoTrajectory3D traj, double scalarX, double scalarY, double scalarZ)
   {
      scale(trajToPack.getYoTrajectoryX(), traj.getYoTrajectoryX(), scalarX);
      scale(trajToPack.getYoTrajectoryY(), traj.getYoTrajectoryY(), scalarY);
      scale(trajToPack.getYoTrajectoryZ(), traj.getYoTrajectoryZ(), scalarZ);
   }

   public static void scale(YoTrajectory3D trajToPack, YoTrajectory3D traj, double scalar)
   {
      scale(trajToPack, traj, scalar, scalar, scalar);
   }

   public static void add(YoTrajectory3D trajToPack, YoTrajectory3D traj1, YoTrajectory3D traj2)
   {
      for (int direction = 0; direction < 3; direction++)
         add(trajToPack.getYoTrajectory(direction), traj1.getYoTrajectory(direction), traj2.getYoTrajectory(direction));
   }

   public static void addByTrimming(YoTrajectory3D trajToPack, YoTrajectory3D traj1, YoTrajectory3D traj2)
   {
      for (int direction = 0; direction < 3; direction++)
         addByTrimming(trajToPack.getYoTrajectory(direction), traj1.getYoTrajectory(direction), traj2.getYoTrajectory(direction));
   }

   public static void subtract(YoTrajectory3D trajToPack, YoTrajectory3D traj1, YoTrajectory3D traj2)
   {
      for (int direction = 0; direction < 3; direction++)
         subtract(trajToPack.getYoTrajectory(direction), traj1.getYoTrajectory(direction), traj2.getYoTrajectory(direction));
   }

   public static void subtractByTrimming(YoTrajectory3D trajToPack, YoTrajectory3D traj1, YoTrajectory3D traj2)
   {
      for (int direction = 0; direction < 3; direction++)
         subtractByTrimming(trajToPack.getYoTrajectory(direction), traj1.getYoTrajectory(direction), traj2.getYoTrajectory(direction));
   }

   public static void dotProduct(YoTrajectory3D trajToPack, YoTrajectory3D traj1, YoTrajectory3D traj2)
   {
      for (int direction = 0; direction < 3; direction++)
         multiply(trajToPack.getYoTrajectory(direction), traj1.getYoTrajectory(direction), traj2.getYoTrajectory(direction));
   }

   public static void dotProductByTrimming(YoTrajectory3D trajToPack, YoTrajectory3D traj1, YoTrajectory3D traj2)
   {
      for (int direction = 0; direction < 3; direction++)
         multiplyByTrimming(trajToPack.getYoTrajectory(direction), traj1.getYoTrajectory(direction), traj2.getYoTrajectory(direction));
   }

   public static void dotProduct(YoTrajectory trajToPackX, YoTrajectory trajToPackY, YoTrajectory trajToPackZ, YoTrajectory traj1X, YoTrajectory traj1Y,
                                 YoTrajectory traj1Z, YoTrajectory traj2X, YoTrajectory traj2Y, YoTrajectory traj2Z)
   {
      multiply(trajToPackX, traj1X, traj2X);
      multiply(trajToPackY, traj1Y, traj2Y);
      multiply(trajToPackZ, traj1Z, traj2Z);
   }

   public static void dotProductByTrimming(YoTrajectory trajToPackX, YoTrajectory trajToPackY, YoTrajectory trajToPackZ, YoTrajectory traj1X,
                                           YoTrajectory traj1Y, YoTrajectory traj1Z, YoTrajectory traj2X, YoTrajectory traj2Y, YoTrajectory traj2Z)
   {
      multiplyByTrimming(trajToPackX, traj1X, traj2X);
      multiplyByTrimming(trajToPackY, traj1Y, traj2Y);
      multiplyByTrimming(trajToPackZ, traj1Z, traj2Z);
   }

   public static void crossProduct(YoTrajectory3D trajToPack, YoTrajectory3D traj1, YoTrajectory3D traj2)
   {
      crossProduct(trajToPack.getYoTrajectoryX(), trajToPack.getYoTrajectoryY(), trajToPack.getYoTrajectoryZ(), traj1.getYoTrajectoryX(),
                   traj1.getYoTrajectoryY(), traj1.getYoTrajectoryZ(), traj2.getYoTrajectoryX(), traj2.getYoTrajectoryY(), traj2.getYoTrajectoryZ());
   }

   public static void crossProductByTrimming(YoTrajectory3D trajToPack, YoTrajectory3D traj1, YoTrajectory3D traj2)
   {
      crossProductByTrimming(trajToPack.getYoTrajectoryX(), trajToPack.getYoTrajectoryY(), trajToPack.getYoTrajectoryZ(), traj1.getYoTrajectoryX(),
                             traj1.getYoTrajectoryY(), traj1.getYoTrajectoryZ(), traj2.getYoTrajectoryX(), traj2.getYoTrajectoryY(), traj2.getYoTrajectoryZ());
   }

   public static void crossProduct(YoTrajectory xTrajectoryToPack, YoTrajectory yTrajectoryToPack, YoTrajectory zTrajectoryToPack,
                                   YoTrajectory traj1X, YoTrajectory traj1Y, YoTrajectory traj1Z, YoTrajectory traj2X, YoTrajectory traj2Y, YoTrajectory traj2Z)
   {
      multiply(tempTraj1, traj1Y, traj2Z);
      multiply(tempTraj2, traj1Z, traj2Y);
      subtract(tempTraj3.xTrajectory, tempTraj1, tempTraj2);

      multiply(tempTraj1, traj1X, traj2Z);
      multiply(tempTraj2, traj1Z, traj2X);
      subtract(tempTraj3.yTrajectory, tempTraj2, tempTraj1);

      multiply(tempTraj1, traj1X, traj2Y);
      multiply(tempTraj2, traj1Y, traj2X);
      subtract(tempTraj3.zTrajectory, tempTraj1, tempTraj2);
      xTrajectoryToPack.set(tempTraj3.xTrajectory);
      yTrajectoryToPack.set(tempTraj3.yTrajectory);
      zTrajectoryToPack.set(tempTraj3.zTrajectory);
   }

   public static void crossProductByTrimming(YoTrajectory trajToPackX, YoTrajectory trajToPackY, YoTrajectory trajToPackZ, YoTrajectory traj1X,
                                             YoTrajectory traj1Y, YoTrajectory traj1Z, YoTrajectory traj2X, YoTrajectory traj2Y, YoTrajectory traj2Z)
   {
      multiplyByTrimming(tempTraj1, traj1Y, traj2Z);
      multiplyByTrimming(tempTraj2, traj1Z, traj2Y);
      subtractByTrimming(tempTraj3.xTrajectory, tempTraj1, tempTraj2);

      multiplyByTrimming(tempTraj1, traj1X, traj2Z);
      multiplyByTrimming(tempTraj2, traj1Z, traj2X);
      subtractByTrimming(tempTraj3.yTrajectory, tempTraj2, tempTraj1);

      multiplyByTrimming(tempTraj1, traj1X, traj2Y);
      multiplyByTrimming(tempTraj2, traj1Y, traj2X);
      subtractByTrimming(tempTraj3.zTrajectory, tempTraj1, tempTraj2);
      trajToPackX.set(tempTraj3.xTrajectory);
      trajToPackY.set(tempTraj3.yTrajectory);
      trajToPackZ.set(tempTraj3.zTrajectory);
   }

   public static void validateTrajectoryTimes(YoTrajectory traj1, YoTrajectory traj2)
   {
      if (Math.abs(traj1.getInitialTime() - traj2.getInitialTime()) > Epsilons.ONE_THOUSANDTH
            || Math.abs(traj1.getFinalTime() - traj2.getFinalTime()) > Epsilons.ONE_THOUSANDTH)
      {
         PrintTools.warn("Time mismatch in trajectories being added");
         throw new InvalidParameterException();
      }
   }

   private static void setTimeIntervalByTrimming(YoTrajectory trajToPack, YoTrajectory traj1, YoTrajectory traj2)
   {
      double latestStartingTime = Math.max(traj1.getInitialTime(), traj2.getInitialTime());
      double earliestEndingTime = Math.min(traj1.getFinalTime(), traj2.getFinalTime());
      if (earliestEndingTime <= latestStartingTime)
      {
         PrintTools.debug(traj1.toString());
         PrintTools.debug(traj2.toString());
         throw new RuntimeException("Got null intersection for time intervals during trajectory operation");
      }
      trajToPack.setInitialTime(latestStartingTime);
      trajToPack.setFinalTime(earliestEndingTime);
   }

   public static void validatePackingTrajectoryForLinearCombination(YoTrajectory trajToPack, YoTrajectory traj1, YoTrajectory traj2)
   {
      if (trajToPack.getMaximumNumberOfCoefficients() < Math.max(traj1.getNumberOfCoefficients(), traj2.getNumberOfCoefficients()))
      {
         PrintTools.warn("Not enough coefficients to store result of trajectory operation. Needed: "
               + Math.max(traj1.getNumberOfCoefficients(), traj2.getNumberOfCoefficients()) + " Available: " + trajToPack.getMaximumNumberOfCoefficients());
         throw new InvalidParameterException();
      }
   }

   public static void validatePackingTrajectoryForMultiplication(YoTrajectory trajToPack, YoTrajectory traj1, YoTrajectory traj2)
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
   public static int add(List<YoTrajectory> trajListToPack, YoTrajectory traj1, YoTrajectory traj2, double TIME_EPSILON)
   {
      checkZeroTimeTrajectory(traj1, TIME_EPSILON);
      checkZeroTimeTrajectory(traj2, TIME_EPSILON);
      int numberOfSegments = getSegmentTimeList(tempTimeList, traj1, traj2, TIME_EPSILON);
      for (int i = 0; i < numberOfSegments; i++)
      {
         YoTrajectory segmentTrajToPack = trajListToPack.get(i);
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
   public static int subtract(List<YoTrajectory> trajListToPack, YoTrajectory traj1, YoTrajectory traj2, double TIME_EPSILON)
   {
      checkZeroTimeTrajectory(traj1, TIME_EPSILON);
      checkZeroTimeTrajectory(traj2, TIME_EPSILON);
      int numberOfSegments = getSegmentTimeList(tempTimeList, traj1, traj2, TIME_EPSILON);
      for (int i = 0; i < numberOfSegments; i++)
      {
         YoTrajectory segmentTrajToPack = trajListToPack.get(i);
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
   public static int multiply(List<YoTrajectory> trajListToPack, YoTrajectory traj1, YoTrajectory traj2, double TIME_EPSILON)
   {
      checkZeroTimeTrajectory(traj1, TIME_EPSILON);
      checkZeroTimeTrajectory(traj2, TIME_EPSILON);
      int numberOfSegments = getSegmentTimeList(tempTimeList, traj1, traj2, TIME_EPSILON);
      for (int i = 0; i < numberOfSegments; i++)
      {
         YoTrajectory segmentTrajToPack = trajListToPack.get(i);
         setCurrentSegmentPolynomial(tempTraj1, traj1, tempTimeList.get(i), tempTimeList.get(i + 1), TIME_EPSILON);
         setCurrentSegmentPolynomial(tempTraj2, traj2, tempTimeList.get(i), tempTimeList.get(i + 1), TIME_EPSILON);
         multiply(segmentTrajToPack, tempTraj1, tempTraj2);
      }
      return numberOfSegments;
   }

   private static void setCurrentSegmentPolynomial(YoTrajectory trajToPack, YoTrajectory traj, double segmentStartTime, double segmentFinalTime,
                                                   double TIME_EPSILON)
   {
      trajToPack.set(traj);
      trajToPack.setInitialTime(segmentStartTime);
      trajToPack.setFinalTime(segmentFinalTime);
      if (traj.getInitialTime() > segmentStartTime + TIME_EPSILON || traj.getFinalTime() < segmentStartTime + TIME_EPSILON)
         trajToPack.setZero();
   }

   private static void setCurrentSegmentPolynomial(YoTrajectory3D trajToPack, YoTrajectory3D traj, double segmentStartTime, double segmentFinalTime,
                                                   double TIME_EPSILON)
   {
      for (int i = 0; i < 3; i++)
         setCurrentSegmentPolynomial(trajToPack.getYoTrajectory(i), traj.getYoTrajectory(i), segmentStartTime, segmentFinalTime, TIME_EPSILON);
   }

   public static int getSegmentTimeList(List<Double> trajTimeListToPack, YoTrajectory traj1, YoTrajectory traj2, double TIME_EPSILON)
   {
      trajTimeListToPack.set(0, traj1.getInitialTime());
      trajTimeListToPack.set(1, traj1.getFinalTime());
      trajTimeListToPack.set(2, traj2.getInitialTime());
      trajTimeListToPack.set(3, traj2.getFinalTime());

      tempTimeArrayLength = trajTimeListToPack.size();
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

   public static void checkZeroTimeTrajectory(YoTrajectory trajectory, double TIME_EPSILON)
   {
      if (Math.abs(trajectory.getFinalTime() - trajectory.getInitialTime()) < TIME_EPSILON)
         throw new RuntimeException("Cannot operate with null trajectory, start time: " + trajectory.getInitialTime() + " end time: "
               + trajectory.getFinalTime() + " epsilon: " + TIME_EPSILON);
   }

   public static void addTimeOffset(YoTrajectory trajectory, double timeOffset)
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
      tempComplexReference = fft.getForwardTransform();
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

   public static void getIntergal(YoTrajectory trajectoryToPack, YoTrajectory trajectoryToIntegrate)
   {
      if (trajectoryToPack.getMaximumNumberOfCoefficients() < trajectoryToIntegrate.getNumberOfCoefficients() + 1)
         throw new InvalidParameterException("Not enough coefficients to store result of trajectory integration");

      YoPolynomial polynomial = trajectoryToPack.getPolynomial();
      for (int i = trajectoryToIntegrate.getNumberOfCoefficients(); i > 0; i--)
         polynomial.setDirectlyFast(i, trajectoryToIntegrate.getCoefficient(i - 1) / (i));
      trajectoryToIntegrate.compute(trajectoryToIntegrate.getInitialTime());
      double position = trajectoryToIntegrate.getPosition();
      polynomial.setDirectly(0, -position);
      trajectoryToPack.reshape(trajectoryToIntegrate.getNumberOfCoefficients() + 1);
      trajectoryToPack.setTime(trajectoryToIntegrate.getInitialTime(), trajectoryToIntegrate.getFinalTime());
   }

   public static void getDerivative(YoTrajectory derivativeToPack, YoTrajectory trajectoryToDifferentiate)
   {
      if (derivativeToPack.getMaximumNumberOfCoefficients() < trajectoryToDifferentiate.getNumberOfCoefficients() - 1)
         throw new InvalidParameterException("Not enough coefficients to store the result of differentiation");

      YoPolynomial derivative = derivativeToPack.getPolynomial();
      YoPolynomial original = trajectoryToDifferentiate.getPolynomial();

      derivative.reshape(Math.max(trajectoryToDifferentiate.getNumberOfCoefficients() - 1, 1));
      if (trajectoryToDifferentiate.getNumberOfCoefficients() == 1)
         derivative.setConstant(0);
      for (int i = trajectoryToDifferentiate.getNumberOfCoefficients() - 1; i > 0; i--)
         derivative.setDirectlyFast(i - 1, i * original.getCoefficient(i));
      derivativeToPack.setTime(trajectoryToDifferentiate.getInitialTime(), trajectoryToDifferentiate.getFinalTime());
   }

   private static YoFrameTrajectory3D segmentTraj1, segmentTraj2;

   public static void addSegmentedTrajectories(YoSegmentedFrameTrajectory3D trajToPack, YoSegmentedFrameTrajectory3D traj1, YoSegmentedFrameTrajectory3D traj2,
                                               double TIME_EPSILON)
   {
      double currentTime = Math.min(traj1.getSegment(0).getInitialTime(), traj2.getSegment(0).getInitialTime());
      int k = 0;
      for (int i = 0, j = 0; i < traj1.getNumberOfSegments() || j < traj2.getNumberOfSegments(); k++)
      {
         // Select the one that is ahead or set if no intersection
         if (i >= traj1.getNumberOfSegments()
               || (j < traj2.getNumberOfSegments() && traj2.getSegment(j).getFinalTime() < traj1.getSegment(i).getInitialTime() - TIME_EPSILON))
         {
            setCurrentSegmentPolynomial(trajToPack.getSegment(k), traj2.getSegment(j), currentTime, traj2.getSegment(j).getFinalTime(), TIME_EPSILON);
            currentTime = traj2.getSegment(j++).getFinalTime();
            continue;
         }
         else if (j >= traj2.getNumberOfSegments()
               || (i < traj1.getNumberOfSegments() && traj1.getSegment(i).getFinalTime() < traj2.getSegment(j).getInitialTime() - TIME_EPSILON))
         {
            setCurrentSegmentPolynomial(trajToPack.getSegment(k), traj1.getSegment(i), currentTime, traj1.getSegment(i).getFinalTime(), TIME_EPSILON);
            currentTime = traj1.getSegment(i++).getFinalTime();
            continue;
         }
         else if (traj1.getSegment(i).getInitialTime() < traj2.getSegment(j).getInitialTime())
         {
            segmentTraj1 = traj1.getSegment(i);
            segmentTraj2 = traj2.getSegment(j);
         }
         else
         {
            segmentTraj2 = traj1.getSegment(i);
            segmentTraj1 = traj2.getSegment(j);
         }
         if (segmentTraj1.getInitialTime() < segmentTraj2.getInitialTime() - TIME_EPSILON && currentTime - TIME_EPSILON < segmentTraj1.getInitialTime())
            setCurrentSegmentPolynomial(trajToPack.getSegment(k++), segmentTraj1, currentTime, segmentTraj2.getInitialTime(), TIME_EPSILON);

         addByTrimming(trajToPack.getSegment(k), segmentTraj1, segmentTraj2);
         currentTime = Math.min(segmentTraj1.getFinalTime(), segmentTraj2.getFinalTime());
         if (traj1.getSegment(i).getFinalTime() < traj2.getSegment(j).getFinalTime() - TIME_EPSILON)
            i++;
         else if (traj2.getSegment(i).getFinalTime() < traj1.getSegment(j).getFinalTime() - TIME_EPSILON)
            j++;
         else
         {
            i++;
            j++;
         }
      }
      trajToPack.setNumberOfSegments(k);
   }

}
