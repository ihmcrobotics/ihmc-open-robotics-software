package us.ihmc.robotics.math.trajectories;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoConcatenatedSplines
{
   private static final double EPSILON = 1e-6;

   private final YoVariableRegistry registry;
   
   private final List<YoSpline3D> splines;
   private final List<ImmutablePair<YoDouble, YoDouble>> rangeList;
   private final YoDouble arcLength;
   
   private final YoFramePoint3D position;
   private final YoFrameVector3D velocity;
   private final YoFrameVector3D acceleration;
   private final ReferenceFrame referenceFrame;
   
   private final YoInteger currentSplineIndex;

   public YoConcatenatedSplines(int[] numberOfCoefficientsPerPolynomial, ReferenceFrame referenceFrame, int arcLengthCalculatorDivisionsPerPolynomial,
                                YoVariableRegistry parentRegistry, String namePrefix)
   {
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);

      this.referenceFrame = referenceFrame;

      this.splines = new ArrayList<YoSpline3D>();
      rangeList = new ArrayList<ImmutablePair<YoDouble, YoDouble>>();

      for (int i = 0; i < numberOfCoefficientsPerPolynomial.length; i++)
      {
         splines.add(new YoSpline3D(numberOfCoefficientsPerPolynomial[i], arcLengthCalculatorDivisionsPerPolynomial, referenceFrame, registry,
                                    namePrefix + "Spline" + i));
         rangeList.add(new ImmutablePair<YoDouble, YoDouble>(new YoDouble(namePrefix + "Range" + i + "First", registry),
                                new YoDouble(namePrefix + "Range" + i + "Second", registry)));
      }

      position = new YoFramePoint3D(namePrefix + "Position", referenceFrame, registry);
      velocity = new YoFrameVector3D(namePrefix + "Velocity", referenceFrame, registry);
      acceleration = new YoFrameVector3D(namePrefix + "Acceleration", referenceFrame, registry);
      arcLength = new YoDouble(namePrefix + "ArcLength", registry);
      
      currentSplineIndex = new YoInteger(namePrefix + "CurrentSplineIndex", registry);
   }

   public void setQuadraticQuinticQuadratic(double[] times, FramePoint3D[] positions, FrameVector3D initialVelocity, FrameVector3D finalVelocity)
   {
      MathTools.checkEquals(times.length, 4);
      MathTools.checkEquals(positions.length, 4);

      int[] reorderedSplineIndices = new int[] {0, 2, 1};

      for (int i : reorderedSplineIndices)
      {
         YoSpline3D spline = splines.get(i);

         double t0 = times[i];
         double tf = times[i + 1];
         FramePoint3D p0 = positions[i];
         FramePoint3D pf = positions[i + 1];

         if (i == 0)
         {
            FrameVector3D pd0 = initialVelocity;
            spline.setQuadraticUsingInitialVelocity(t0, tf, p0, pd0, pf);
         }
         else if (i == 2)
         {
            FrameVector3D pdf = finalVelocity;
            spline.setQuadraticUsingFinalVelocity(t0, tf, p0, pf, pdf);
         }
         else if (i == 1)
         {
            splines.get(0).compute(times[i]);
            FrameVector3D pd0 = splines.get(0).getVelocityCopy();
            FrameVector3D pdd0 = splines.get(0).getAccelerationCopy();
            splines.get(2).compute(times[i + 1]);
            FrameVector3D pdf = splines.get(2).getVelocityCopy();
            FrameVector3D pddf = splines.get(2).getAccelerationCopy();
            spline.setQuintic(t0, tf, p0, pd0, pdd0, pf, pdf, pddf);
            splines.get(1).compute(splines.get(1).getT0());
         }

         rangeList.get(i).getLeft().set(t0);
         rangeList.get(i).getRight().set(tf);
      }

      setArcLength();
   }

   public void setCubicLinearQuinticLinearCubic(double[] fixedPointTimes, FramePoint3D[] fixedPointPositions, FrameVector3D[] fixedPointVelocities,
           double[] intermediateTimes)
   {
      MathTools.checkEquals(rangeList.size(), 5);
      MathTools.checkEquals(fixedPointTimes.length, 4);
      MathTools.checkEquals(fixedPointPositions.length, 4);
      MathTools.checkEquals(fixedPointVelocities.length, 4);

      double[] allTimes = new double[]
      {
         fixedPointTimes[0], intermediateTimes[0], fixedPointTimes[1], fixedPointTimes[2], intermediateTimes[1], fixedPointTimes[3]
      };
      FramePoint3D[] allPositions = new FramePoint3D[]
      {
         fixedPointPositions[0], null, fixedPointPositions[1], fixedPointPositions[2], null, fixedPointPositions[3]
      };
      FrameVector3D[] allVelocities = new FrameVector3D[]
      {
         fixedPointVelocities[0], fixedPointVelocities[1], fixedPointVelocities[1], fixedPointVelocities[2], fixedPointVelocities[2], fixedPointVelocities[3]
      };
      int[] reorderedSplineIndices = new int[] {1, 3, 0, 4, 2};

      for (int i : reorderedSplineIndices)
      {
         YoSpline3D spline = splines.get(i);

         double t0 = allTimes[i];
         double tf = allTimes[i + 1];
         FramePoint3D p0 = allPositions[i];
         FramePoint3D pf = allPositions[i + 1];
         FrameVector3D pd0 = allVelocities[i];
         FrameVector3D pdf = allVelocities[i + 1];
         FrameVector3D zero = new FrameVector3D(referenceFrame, 0.0, 0.0, 0.0);

         if (i == 1)
         {
            spline.setLinearUsingFinalPositionAndVelocity(t0, tf, pf, pdf);
         }

         else if (i == 3)
         {
            spline.setLinearUsingInitialPositionAndVelocity(t0, tf, p0, pd0);
         }

         else if (i == 0)
         {
            splines.get(1).compute(tf);
            pf = splines.get(1).getPositionCopy();
            spline.setCubic(t0, tf, p0, pd0, pf, pdf);
         }

         else if (i == 4)
         {
            splines.get(3).compute(t0);
            p0 = splines.get(3).getPositionCopy();
            spline.setCubic(t0, tf, p0, pd0, pf, pdf);
         }

         else if (i == 2)
         {
            splines.get(1).compute(t0);
            FrameVector3D pdd0 = zero;
            splines.get(3).compute(tf);
            FrameVector3D pddf = zero;
            spline.setQuintic(t0, tf, p0, pd0, pdd0, pf, pdf, pddf);
         }

         rangeList.get(i).getLeft().set(t0);
         rangeList.get(i).getRight().set(tf);
      }

      setArcLength();
   }

   public void setQuarticLinearQuinticLinearQuartic(double[] fixedPointTimes, FramePoint3D[] fixedPointPositions, FrameVector3D[] fixedPointVelocities,
           double[] intermediateTimes)
   {
      MathTools.checkEquals(rangeList.size(), 5);
      MathTools.checkEquals(fixedPointTimes.length, 4);
      MathTools.checkEquals(fixedPointPositions.length, 4);
      MathTools.checkEquals(fixedPointVelocities.length, 4);

      double[] allTimes = new double[]
      {
         fixedPointTimes[0], intermediateTimes[0], fixedPointTimes[1], fixedPointTimes[2], intermediateTimes[1], fixedPointTimes[3]
      };
      FramePoint3D[] allPositions = new FramePoint3D[]
      {
         fixedPointPositions[0], null, fixedPointPositions[1], fixedPointPositions[2], null, fixedPointPositions[3]
      };
      FrameVector3D[] allVelocities = new FrameVector3D[]
      {
         fixedPointVelocities[0], fixedPointVelocities[1], fixedPointVelocities[1], fixedPointVelocities[2], fixedPointVelocities[2], fixedPointVelocities[3]
      };

      int[] reorderedSplineIndices = new int[] {1, 3, 0, 4, 2};

      for (int i : reorderedSplineIndices)
      {
         YoSpline3D spline = splines.get(i);

         double t0 = allTimes[i];
         double tf = allTimes[i + 1];
         FramePoint3D p0 = allPositions[i];
         FramePoint3D pf = allPositions[i + 1];
         FrameVector3D pd0 = allVelocities[i];
         FrameVector3D pdf = allVelocities[i + 1];
         FrameVector3D zero = new FrameVector3D(referenceFrame, 0.0, 0.0, 0.0);

         if (i == 1)
         {
            spline.setLinearUsingFinalPositionAndVelocity(t0, tf, pf, pdf);
         }

         else if (i == 3)
         {
            spline.setLinearUsingInitialPositionAndVelocity(t0, tf, p0, pd0);
         }

         else if (i == 0)
         {
            splines.get(1).compute(tf);
            pf = splines.get(1).getPositionCopy();
            FrameVector3D pddf = zero;
            spline.setQuarticUsingFinalAcceleration(t0, tf, p0, pd0, pf, pdf, pddf);
         }

         else if (i == 4)
         {
            splines.get(3).compute(t0);
            p0 = splines.get(3).getPositionCopy();
            FrameVector3D pdd0 = zero;
            spline.setQuarticUsingInitialAcceleration(t0, tf, p0, pd0, pdd0, pf, pdf);
         }

         else if (i == 2)
         {
            splines.get(1).compute(t0);
            FrameVector3D pdd0 = zero;
            splines.get(3).compute(tf);
            FrameVector3D pddf = zero;
            spline.setQuintic(t0, tf, p0, pd0, pdd0, pf, pdf, pddf);
         }

         rangeList.get(i).getLeft().set(t0);
         rangeList.get(i).getRight().set(tf);
      }

      setArcLength();
   }

   public void setCubicQuarticQuinticCubic(double[] times, FramePoint3D[] positions, FrameVector3D[] velocities)
   {
      MathTools.checkEquals(rangeList.size(), 4);
      MathTools.checkEquals(times.length, 5);
      MathTools.checkEquals(positions.length, 5);
      MathTools.checkEquals(velocities.length, 5);

      int[] reorderedSplineIndices = new int[] {0, 3, 1, 2};

      for (int i : reorderedSplineIndices)
      {
         YoSpline3D spline = splines.get(i);

         double t0 = times[i];
         double tf = times[i + 1];
         FramePoint3D p0 = positions[i];
         FramePoint3D pf = positions[i + 1];
         FrameVector3D pd0 = velocities[i];
         FrameVector3D pdf = velocities[i + 1];

         if ((i == 0) || (i == 3))
         {
            spline.setCubic(t0, tf, p0, pd0, pf, pdf);
         }

         else if (i == 1)
         {
            splines.get(0).compute(t0);
            FrameVector3D pdd0 = splines.get(0).getAccelerationCopy();
            spline.setQuarticUsingInitialAcceleration(t0, tf, p0, pd0, pdd0, pf, pdf);
         }

         else if (i == 2)
         {
            splines.get(1).compute(t0);
            FrameVector3D pdd0 = splines.get(1).getAccelerationCopy();
            splines.get(3).compute(tf);
            FrameVector3D pddf = splines.get(3).getAccelerationCopy();
            spline.setQuintic(t0, tf, p0, pd0, pdd0, pf, pdf, pddf);
         }

         rangeList.get(i).getLeft().set(t0);
         rangeList.get(i).getRight().set(tf);
      }

      setArcLength();
   }

   public void setCubicQuarticQuinticQuarticCubic(double[] times, FramePoint3D[] positions, FrameVector3D[] velocities)
   {
      MathTools.checkEquals(rangeList.size(), 5);
      MathTools.checkEquals(times.length, 6);
      MathTools.checkEquals(positions.length, 6);
      MathTools.checkEquals(velocities.length, 6);

      int[] reorderedSplineIndices = new int[] {0, 4, 1, 3, 2};

      for (int i : reorderedSplineIndices)
      {
         YoSpline3D spline = splines.get(i);

         double t0 = times[i];
         double tf = times[i + 1];
         FramePoint3D p0 = positions[i];
         FramePoint3D pf = positions[i + 1];
         FrameVector3D pd0 = velocities[i];
         FrameVector3D pdf = velocities[i + 1];

         if ((i == 0) || (i == 4))
         {
            spline.setCubic(t0, tf, p0, pd0, pf, pdf);
         }

         else if (i == 1)
         {
            splines.get(0).compute(t0);
            FrameVector3D pdd0 = splines.get(0).getAccelerationCopy();
            spline.setQuarticUsingInitialAcceleration(t0, tf, p0, pd0, pdd0, pf, pdf);
         }

         else if (i == 3)
         {
            splines.get(4).compute(tf);
            FrameVector3D pddf = splines.get(4).getAccelerationCopy();
            spline.setQuarticUsingFinalAcceleration(t0, tf, p0, pd0, pf, pdf, pddf);
         }

         else if (i == 2)
         {
            splines.get(1).compute(t0);
            FrameVector3D pdd0 = splines.get(1).getAccelerationCopy();
            splines.get(3).compute(tf);
            FrameVector3D pddf = splines.get(3).getAccelerationCopy();
            spline.setQuintic(t0, tf, p0, pd0, pdd0, pf, pdf, pddf);
         }

         rangeList.get(i).getLeft().set(t0);
         rangeList.get(i).getRight().set(tf);
      }

      setArcLength();
   }

   public void setCubicQuinticCubic(double[] times, FramePoint3D[] positions, FrameVector3D[] velocities)
   {
      MathTools.checkEquals(rangeList.size(), 3);
      MathTools.checkEquals(times.length, 4);
      MathTools.checkEquals(positions.length, 4);
      MathTools.checkEquals(velocities.length, 4);

      int[] reorderedSplineIndices = new int[] {0, 2, 1};

      for (int i : reorderedSplineIndices)
      {
         YoSpline3D spline = splines.get(i);

         double t0 = times[i];
         double tf = times[i + 1];
         FramePoint3D p0 = positions[i];
         FramePoint3D pf = positions[i + 1];
         FrameVector3D pd0 = velocities[i];
         FrameVector3D pdf = velocities[i + 1];

         if ((i == 0) || (i == 2))
         {
            spline.setCubic(t0, tf, p0, pd0, pf, pdf);
         }
         else if (i == 1)
         {
            splines.get(0).compute(t0);
            FrameVector3D pdd0 = splines.get(0).getAccelerationCopy();
            splines.get(2).compute(tf);
            FrameVector3D pddf = splines.get(2).getAccelerationCopy();
            spline.setQuintic(t0, tf, p0, pd0, pdd0, pf, pdf, pddf);
         }

         rangeList.get(i).getLeft().set(t0);
         rangeList.get(i).getRight().set(tf);
      }

      setArcLength();
   }

   public void setQuarticQuinticQuartic(double[] times, FramePoint3D[] positions, FrameVector3D[] velocities, double[] intermediateTimes,
           FrameVector3D[] intermediateVelocities)
   {
      MathTools.checkEquals(rangeList.size(), 3);
      MathTools.checkEquals(times.length, 4);
      MathTools.checkEquals(positions.length, 4);
      MathTools.checkEquals(velocities.length, 4);
      MathTools.checkEquals(intermediateTimes.length, 2);
      MathTools.checkEquals(intermediateVelocities.length, 2);

      int[] reorderedSplineIndices = new int[] {0, 2, 1};

      for (int i : reorderedSplineIndices)
      {
         YoSpline3D spline = splines.get(i);

         double t0 = times[i];
         double tf = times[i + 1];
         FramePoint3D p0 = positions[i];
         FramePoint3D pf = positions[i + 1];
         FrameVector3D pd0 = velocities[i];
         FrameVector3D pdf = velocities[i + 1];

         if ((i == 0) || (i == 2))
         {
            double t1 = intermediateTimes[i / 2];
            FrameVector3D pd1 = intermediateVelocities[i / 2];
            spline.setQuarticUsingIntermediateVelocity(t0, t1, tf, p0, pd0, pd1, pf, pdf);
         }
         else if (i == 1)
         {
            splines.get(0).compute(t0);
            FrameVector3D pdd0 = splines.get(0).getAccelerationCopy();
            splines.get(2).compute(tf);
            FrameVector3D pddf = splines.get(2).getAccelerationCopy();
            spline.setQuintic(t0, tf, p0, pd0, pdd0, pf, pdf, pddf);
         }

         rangeList.get(i).getLeft().set(t0);
         rangeList.get(i).getRight().set(tf);
      }

      setArcLength();
   }

   public void setQuinticsUsingIntermediateVeloctiesAndAccelerations(double[] times, FramePoint3D[] positions, FrameVector3D[] velocities,
           double[] intermediateTimes, FrameVector3D[] intermediateVelocities, FrameVector3D[] intermediateAccelerations)
   {
      MathTools.checkEquals(rangeList.size(), 3);
      MathTools.checkEquals(times.length, 4);
      MathTools.checkEquals(positions.length, 4);
      MathTools.checkEquals(velocities.length, 4);
      MathTools.checkEquals(intermediateTimes.length, 2);
      MathTools.checkEquals(intermediateVelocities.length, 2);
      MathTools.checkEquals(intermediateAccelerations.length, 2);

      int[] reorderedSplineIndices = new int[] {0, 2, 1};

      for (int i : reorderedSplineIndices)
      {
         YoSpline3D spline = splines.get(i);

         double t0 = times[i];
         double tf = times[i + 1];
         FramePoint3D p0 = positions[i];
         FramePoint3D pf = positions[i + 1];
         FrameVector3D pd0 = velocities[i];
         FrameVector3D pdf = velocities[i + 1];

         if ((i == 0) || (i == 2))
         {
            double t1 = intermediateTimes[i / 2];
            FrameVector3D pd1 = intermediateVelocities[i / 2];
            FrameVector3D pdd1 = intermediateAccelerations[i / 2];
            spline.setQuinticUsingIntermediateVelocityAndAcceleration(t0, t1, tf, p0, pd0, pd1, pdd1, pf, pdf);
         }
         else if (i == 1)
         {
            splines.get(0).compute(t0);
            FrameVector3D pdd0 = splines.get(0).getAccelerationCopy();
            splines.get(2).compute(tf);
            FrameVector3D pddf = splines.get(2).getAccelerationCopy();
            spline.setQuintic(t0, tf, p0, pd0, pdd0, pf, pdf, pddf);
         }

         rangeList.get(i).getLeft().set(t0);
         rangeList.get(i).getRight().set(tf);
      }

      setArcLength();
   }

   public void setQuinticSexticQuinticUsingWaypointAndIntermediateVeloctiesAndAccelerations(double[] times, FramePoint3D[] positions, FrameVector3D[] velocities,
           double ghostTime, FramePoint3D ghostWaypoint, double[] intermediateTimes, FrameVector3D[] intermediateVelocities,
           FrameVector3D[] intermediateAccelerations)
   {
      MathTools.checkEquals(rangeList.size(), 3);
      MathTools.checkEquals(times.length, 4);
      MathTools.checkEquals(positions.length, 4);
      MathTools.checkEquals(velocities.length, 4);
      MathTools.checkEquals(intermediateTimes.length, 2);
      MathTools.checkEquals(intermediateVelocities.length, 2);
      MathTools.checkEquals(intermediateAccelerations.length, 2);

      int[] reorderedSplineIndices = new int[] {0, 2, 1};

      for (int i : reorderedSplineIndices)
      {
         YoSpline3D spline = splines.get(i);

         double t0 = times[i];
         double tf = times[i + 1];
         FramePoint3D p0 = positions[i];
         FramePoint3D pf = positions[i + 1];
         FrameVector3D pd0 = velocities[i];
         FrameVector3D pdf = velocities[i + 1];

         if ((i == 0) || (i == 2))
         {
            double t1 = intermediateTimes[i / 2];
            FrameVector3D pd1 = intermediateVelocities[i / 2];
            FrameVector3D pdd1 = intermediateAccelerations[i / 2];
            spline.setQuinticUsingIntermediateVelocityAndAcceleration(t0, t1, tf, p0, pd0, pd1, pdd1, pf, pdf);
         }
         else if (i == 1)
         {
            splines.get(0).compute(t0);
            FrameVector3D pdd0 = splines.get(0).getAccelerationCopy();
            splines.get(2).compute(tf);
            FrameVector3D pddf = splines.get(2).getAccelerationCopy();
            spline.setSexticUsingWaypoint(t0, ghostTime, tf, p0, pd0, pdd0, ghostWaypoint, pf, pdf, pddf);
         }

         rangeList.get(i).getLeft().set(t0);
         rangeList.get(i).getRight().set(tf);
      }

      setArcLength();
   }

   public void setSexticQuinticSexticUsingIntermediateVelocitiesAndAccelerations(double[] times, FramePoint3D[] positions, FrameVector3D[] velocities,
           FrameVector3D[] waypointAccelerations, double[] intermediateTimes, FrameVector3D[] intermediateVelocities, FrameVector3D[] intermediateAccelerations)
   {
      MathTools.checkEquals(rangeList.size(), 3);
      MathTools.checkEquals(times.length, 4);
      MathTools.checkEquals(positions.length, 4);
      MathTools.checkEquals(velocities.length, 4);
      MathTools.checkEquals(waypointAccelerations.length, 2);
      MathTools.checkEquals(intermediateTimes.length, 2);
      MathTools.checkEquals(intermediateVelocities.length, 2);
      MathTools.checkEquals(intermediateAccelerations.length, 2);

      int[] reorderedSplineIndices = new int[] {0, 2, 1};

      for (int i : reorderedSplineIndices)
      {
         YoSpline3D spline = splines.get(i);

         double t0 = times[i];
         double t1 = 0.0;
         double tf = times[i + 1];
         FramePoint3D p0 = positions[i];
         FramePoint3D pf = positions[i + 1];
         FrameVector3D pd1 = null;
         FrameVector3D pdd1 = null;
         FrameVector3D pd0 = velocities[i];
         FrameVector3D pdf = velocities[i + 1];

         if ((i == 0) || (i == 2))
         {
            t1 = intermediateTimes[i / 2];
            pd1 = intermediateVelocities[i / 2];
            pdd1 = intermediateAccelerations[i / 2];
         }

         if (i == 0)
         {
            FrameVector3D pddf = waypointAccelerations[0];
            spline.setSexticUsingWaypointVelocityAndAccelerationAndFinalAcceleration(t0, t1, tf, p0, pd0, pd1, pdd1, pf, pdf, pddf);
         }
         else if (i == 2)
         {
            FrameVector3D pdd0 = waypointAccelerations[1];
            spline.setSexticUsingWaypointVelocityAndAccelerationAndInitialAcceleration(t0, t1, tf, p0, pd0, pdd0, pd1, pdd1, pf, pdf);
         }

         else if (i == 1)
         {
            splines.get(0).compute(t0);
            FrameVector3D pdd0 = splines.get(0).getAccelerationCopy();
            splines.get(2).compute(tf);
            FrameVector3D pddf = splines.get(2).getAccelerationCopy();
            spline.setQuintic(t0, tf, p0, pd0, pdd0, pf, pdf, pddf);
         }

         rangeList.get(i).getLeft().set(t0);
         rangeList.get(i).getRight().set(tf);
      }

      setArcLength();
   }

   public void setQuintics(double[] times, FramePoint3D[] positions, FrameVector3D[] velocities, FrameVector3D[] accelerations)
   {
      MathTools.checkEquals(times.length, rangeList.size() + 1);
      MathTools.checkEquals(positions.length, times.length);
      MathTools.checkEquals(velocities.length, times.length);
      MathTools.checkEquals(accelerations.length, times.length);

      for (int i = 0; i < times.length - 1; i++)
      {
         YoSpline3D spline = splines.get(i);

         double t0 = times[i];
         double tf = times[i + 1];
         FramePoint3D p0 = positions[i];
         FramePoint3D pf = positions[i + 1];
         FrameVector3D pd0 = velocities[i];
         FrameVector3D pdf = velocities[i + 1];
         FrameVector3D pdd0 = accelerations[i];
         FrameVector3D pddf = accelerations[i + 1];

         spline.setQuintic(t0, tf, p0, pd0, pdd0, pf, pdf, pddf);

         rangeList.get(i).getLeft().set(t0);
         rangeList.get(i).getRight().set(tf);
      }

      setArcLength();
   }

   public void setQuintics(YoConcatenatedSplines oldSplines, double[] oldTimes, double[] newTimes)
   {
      MathTools.checkEquals(oldTimes.length, newTimes.length);
      MathTools.checkEquals(oldTimes.length, rangeList.size() + 1);

      for (int i = 0; i < oldTimes.length - 1; i++)
      {
         YoSpline3D spline = splines.get(i);

         oldSplines.compute(oldTimes[i]);
         double t0 = newTimes[i];
         FramePoint3D p0 = oldSplines.getPosition();
         FrameVector3D pd0 = oldSplines.getVelocity();
         FrameVector3D pdd0 = oldSplines.getAcceleration();

         oldSplines.compute(oldTimes[i + 1]);
         double tf = newTimes[i + 1];
         FramePoint3D pf = oldSplines.getPosition();
         FrameVector3D pdf = oldSplines.getVelocity();
         FrameVector3D pddf = oldSplines.getAcceleration();

         spline.setQuintic(t0, tf, p0, pd0, pdd0, pf, pdf, pddf);

         rangeList.get(i).getLeft().set(t0);
         rangeList.get(i).getRight().set(tf);
      }

      setArcLength();
   }

   private void setArcLength()
   {
      double arcLength = 0.0;
      for (YoSpline3D spline : splines)
      {
         arcLength += spline.getArcLength();
      }

      this.arcLength.set(arcLength);
   }

   public double approximateTimeFromArcLength(double arcLength)
   {
      if (arcLength > this.arcLength.getDoubleValue())
      {
         return getTf() - getT0();
      }

      int splineIndex = 0;
      double runningArcLengthCounter = arcLength;
      double runningTimeCounter = getT0();
      double deltaArcLength = splines.get(splineIndex).getArcLength();
      while (runningArcLengthCounter - deltaArcLength > EPSILON)
      {
         runningArcLengthCounter -= deltaArcLength;
         runningTimeCounter += splines.get(splineIndex).getTotalTime();
         splineIndex++;
         deltaArcLength = splines.get(splineIndex).getArcLength();
      }

      runningTimeCounter += splines.get(splineIndex).getApproximateTimeForArcLength(runningArcLengthCounter);

      return runningTimeCounter;
   }

   public void compute(double t)
   {
      currentSplineIndex.set(getSplineIndex(t));
      YoSpline3D desiredSpline = splines.get(currentSplineIndex.getIntegerValue());

      desiredSpline.compute(t);
      desiredSpline.getPosition(position);
      desiredSpline.getVelocity(velocity);
      desiredSpline.getAcceleration(acceleration);
   }

   public FramePoint3D getPosition()
   {
      return new FramePoint3D(position);
   }

   public FrameVector3D getVelocity()
   {
      return new FrameVector3D(velocity);
   }

   public FrameVector3D getAcceleration()
   {
      return new FrameVector3D(acceleration);
   }

   public void getPosition(YoFramePoint3D positionToPack)
   {
      positionToPack.set(position);
   }

   public void getVelocity(YoFrameVector3D velocityToPack)
   {
      velocityToPack.set(velocity);
   }

   public void getAcceleration(YoFrameVector3D accelerationToPack)
   {
      accelerationToPack.set(acceleration);
   }

   public double getT0()
   {
      return rangeList.get(0).getLeft().getDoubleValue();
   }

   public double getTf()
   {
      return rangeList.get(rangeList.size() - 1).getRight().getDoubleValue();
   }

   public double getArcLength()
   {
      return arcLength.getDoubleValue();
   }

   private int getSplineIndex(double t)
   {
      t = MathTools.clamp(t, rangeList.get(0).getLeft().getDoubleValue(), rangeList.get(rangeList.size() - 1).getRight().getDoubleValue());
      int index;
      int lowerBound = 0;
      int upperBound = rangeList.size() - 1;
      while (true)
      {
         index = (lowerBound + upperBound) / 2;

         if ((rangeList.get(index).getLeft().getDoubleValue() <= t) && (t <= rangeList.get(index).getRight().getDoubleValue()))
         {
            return index;
         }

         if (t <= rangeList.get(index).getLeft().getDoubleValue())
         {
            upperBound = index - 1;
         }

         else if (t >= rangeList.get(index).getRight().getDoubleValue())
         {
            lowerBound = index + 1;
         }
      }
   }

   public int getNumberOfSplines()
   {
      return splines.size();
   }

   public YoSpline3D getSplineByIndex(int i)
   {
      return splines.get(i);
   }
}
