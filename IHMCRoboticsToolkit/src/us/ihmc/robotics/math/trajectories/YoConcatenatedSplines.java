package us.ihmc.robotics.math.trajectories;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;


public class YoConcatenatedSplines
{
   private static final double EPSILON = 1e-6;

   private final YoVariableRegistry registry;
   
   private final List<YoSpline3D> splines;
   private final List<ImmutablePair<DoubleYoVariable, DoubleYoVariable>> rangeList;
   private final DoubleYoVariable arcLength;
   
   private final YoFramePoint position;
   private final YoFrameVector velocity;
   private final YoFrameVector acceleration;
   private final ReferenceFrame referenceFrame;
   
   private final IntegerYoVariable currentSplineIndex;

   public YoConcatenatedSplines(int[] numberOfCoefficientsPerPolynomial, ReferenceFrame referenceFrame, int arcLengthCalculatorDivisionsPerPolynomial,
                                YoVariableRegistry parentRegistry, String namePrefix)
   {
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);

      this.referenceFrame = referenceFrame;

      this.splines = new ArrayList<YoSpline3D>();
      rangeList = new ArrayList<ImmutablePair<DoubleYoVariable, DoubleYoVariable>>();

      for (int i = 0; i < numberOfCoefficientsPerPolynomial.length; i++)
      {
         splines.add(new YoSpline3D(numberOfCoefficientsPerPolynomial[i], arcLengthCalculatorDivisionsPerPolynomial, referenceFrame, registry,
                                    namePrefix + "Spline" + i));
         rangeList.add(new ImmutablePair<DoubleYoVariable, DoubleYoVariable>(new DoubleYoVariable(namePrefix + "Range" + i + "First", registry),
                                new DoubleYoVariable(namePrefix + "Range" + i + "Second", registry)));
      }

      position = new YoFramePoint(namePrefix + "Position", referenceFrame, registry);
      velocity = new YoFrameVector(namePrefix + "Velocity", referenceFrame, registry);
      acceleration = new YoFrameVector(namePrefix + "Acceleration", referenceFrame, registry);
      arcLength = new DoubleYoVariable(namePrefix + "ArcLength", registry);
      
      currentSplineIndex = new IntegerYoVariable(namePrefix + "CurrentSplineIndex", registry);
   }

   public void setQuadraticQuinticQuadratic(double[] times, FramePoint[] positions, FrameVector initialVelocity, FrameVector finalVelocity)
   {
      MathTools.checkIfEqual(times.length, 4);
      MathTools.checkIfEqual(positions.length, 4);

      int[] reorderedSplineIndices = new int[] {0, 2, 1};

      for (int i : reorderedSplineIndices)
      {
         YoSpline3D spline = splines.get(i);

         double t0 = times[i];
         double tf = times[i + 1];
         FramePoint p0 = positions[i];
         FramePoint pf = positions[i + 1];

         if (i == 0)
         {
            FrameVector pd0 = initialVelocity;
            spline.setQuadraticUsingInitialVelocity(t0, tf, p0, pd0, pf);
         }
         else if (i == 2)
         {
            FrameVector pdf = finalVelocity;
            spline.setQuadraticUsingFinalVelocity(t0, tf, p0, pf, pdf);
         }
         else if (i == 1)
         {
            splines.get(0).compute(times[i]);
            FrameVector pd0 = splines.get(0).getVelocityCopy();
            FrameVector pdd0 = splines.get(0).getAccelerationCopy();
            splines.get(2).compute(times[i + 1]);
            FrameVector pdf = splines.get(2).getVelocityCopy();
            FrameVector pddf = splines.get(2).getAccelerationCopy();
            spline.setQuintic(t0, tf, p0, pd0, pdd0, pf, pdf, pddf);
            splines.get(1).compute(splines.get(1).getT0());
         }

         rangeList.get(i).getLeft().set(t0);
         rangeList.get(i).getRight().set(tf);
      }

      setArcLength();
   }

   public void setCubicLinearQuinticLinearCubic(double[] fixedPointTimes, FramePoint[] fixedPointPositions, FrameVector[] fixedPointVelocities,
           double[] intermediateTimes)
   {
      MathTools.checkIfEqual(rangeList.size(), 5);
      MathTools.checkIfEqual(fixedPointTimes.length, 4);
      MathTools.checkIfEqual(fixedPointPositions.length, 4);
      MathTools.checkIfEqual(fixedPointVelocities.length, 4);

      double[] allTimes = new double[]
      {
         fixedPointTimes[0], intermediateTimes[0], fixedPointTimes[1], fixedPointTimes[2], intermediateTimes[1], fixedPointTimes[3]
      };
      FramePoint[] allPositions = new FramePoint[]
      {
         fixedPointPositions[0], null, fixedPointPositions[1], fixedPointPositions[2], null, fixedPointPositions[3]
      };
      FrameVector[] allVelocities = new FrameVector[]
      {
         fixedPointVelocities[0], fixedPointVelocities[1], fixedPointVelocities[1], fixedPointVelocities[2], fixedPointVelocities[2], fixedPointVelocities[3]
      };
      int[] reorderedSplineIndices = new int[] {1, 3, 0, 4, 2};

      for (int i : reorderedSplineIndices)
      {
         YoSpline3D spline = splines.get(i);

         double t0 = allTimes[i];
         double tf = allTimes[i + 1];
         FramePoint p0 = allPositions[i];
         FramePoint pf = allPositions[i + 1];
         FrameVector pd0 = allVelocities[i];
         FrameVector pdf = allVelocities[i + 1];
         FrameVector zero = new FrameVector(referenceFrame, 0.0, 0.0, 0.0);

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
            FrameVector pdd0 = zero;
            splines.get(3).compute(tf);
            FrameVector pddf = zero;
            spline.setQuintic(t0, tf, p0, pd0, pdd0, pf, pdf, pddf);
         }

         rangeList.get(i).getLeft().set(t0);
         rangeList.get(i).getRight().set(tf);
      }

      setArcLength();
   }

   public void setQuarticLinearQuinticLinearQuartic(double[] fixedPointTimes, FramePoint[] fixedPointPositions, FrameVector[] fixedPointVelocities,
           double[] intermediateTimes)
   {
      MathTools.checkIfEqual(rangeList.size(), 5);
      MathTools.checkIfEqual(fixedPointTimes.length, 4);
      MathTools.checkIfEqual(fixedPointPositions.length, 4);
      MathTools.checkIfEqual(fixedPointVelocities.length, 4);

      double[] allTimes = new double[]
      {
         fixedPointTimes[0], intermediateTimes[0], fixedPointTimes[1], fixedPointTimes[2], intermediateTimes[1], fixedPointTimes[3]
      };
      FramePoint[] allPositions = new FramePoint[]
      {
         fixedPointPositions[0], null, fixedPointPositions[1], fixedPointPositions[2], null, fixedPointPositions[3]
      };
      FrameVector[] allVelocities = new FrameVector[]
      {
         fixedPointVelocities[0], fixedPointVelocities[1], fixedPointVelocities[1], fixedPointVelocities[2], fixedPointVelocities[2], fixedPointVelocities[3]
      };

      int[] reorderedSplineIndices = new int[] {1, 3, 0, 4, 2};

      for (int i : reorderedSplineIndices)
      {
         YoSpline3D spline = splines.get(i);

         double t0 = allTimes[i];
         double tf = allTimes[i + 1];
         FramePoint p0 = allPositions[i];
         FramePoint pf = allPositions[i + 1];
         FrameVector pd0 = allVelocities[i];
         FrameVector pdf = allVelocities[i + 1];
         FrameVector zero = new FrameVector(referenceFrame, 0.0, 0.0, 0.0);

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
            FrameVector pddf = zero;
            spline.setQuarticUsingFinalAcceleration(t0, tf, p0, pd0, pf, pdf, pddf);
         }

         else if (i == 4)
         {
            splines.get(3).compute(t0);
            p0 = splines.get(3).getPositionCopy();
            FrameVector pdd0 = zero;
            spline.setQuarticUsingInitialAcceleration(t0, tf, p0, pd0, pdd0, pf, pdf);
         }

         else if (i == 2)
         {
            splines.get(1).compute(t0);
            FrameVector pdd0 = zero;
            splines.get(3).compute(tf);
            FrameVector pddf = zero;
            spline.setQuintic(t0, tf, p0, pd0, pdd0, pf, pdf, pddf);
         }

         rangeList.get(i).getLeft().set(t0);
         rangeList.get(i).getRight().set(tf);
      }

      setArcLength();
   }

   public void setCubicQuarticQuinticCubic(double[] times, FramePoint[] positions, FrameVector[] velocities)
   {
      MathTools.checkIfEqual(rangeList.size(), 4);
      MathTools.checkIfEqual(times.length, 5);
      MathTools.checkIfEqual(positions.length, 5);
      MathTools.checkIfEqual(velocities.length, 5);

      int[] reorderedSplineIndices = new int[] {0, 3, 1, 2};

      for (int i : reorderedSplineIndices)
      {
         YoSpline3D spline = splines.get(i);

         double t0 = times[i];
         double tf = times[i + 1];
         FramePoint p0 = positions[i];
         FramePoint pf = positions[i + 1];
         FrameVector pd0 = velocities[i];
         FrameVector pdf = velocities[i + 1];

         if ((i == 0) || (i == 3))
         {
            spline.setCubic(t0, tf, p0, pd0, pf, pdf);
         }

         else if (i == 1)
         {
            splines.get(0).compute(t0);
            FrameVector pdd0 = splines.get(0).getAccelerationCopy();
            spline.setQuarticUsingInitialAcceleration(t0, tf, p0, pd0, pdd0, pf, pdf);
         }

         else if (i == 2)
         {
            splines.get(1).compute(t0);
            FrameVector pdd0 = splines.get(1).getAccelerationCopy();
            splines.get(3).compute(tf);
            FrameVector pddf = splines.get(3).getAccelerationCopy();
            spline.setQuintic(t0, tf, p0, pd0, pdd0, pf, pdf, pddf);
         }

         rangeList.get(i).getLeft().set(t0);
         rangeList.get(i).getRight().set(tf);
      }

      setArcLength();
   }

   public void setCubicQuarticQuinticQuarticCubic(double[] times, FramePoint[] positions, FrameVector[] velocities)
   {
      MathTools.checkIfEqual(rangeList.size(), 5);
      MathTools.checkIfEqual(times.length, 6);
      MathTools.checkIfEqual(positions.length, 6);
      MathTools.checkIfEqual(velocities.length, 6);

      int[] reorderedSplineIndices = new int[] {0, 4, 1, 3, 2};

      for (int i : reorderedSplineIndices)
      {
         YoSpline3D spline = splines.get(i);

         double t0 = times[i];
         double tf = times[i + 1];
         FramePoint p0 = positions[i];
         FramePoint pf = positions[i + 1];
         FrameVector pd0 = velocities[i];
         FrameVector pdf = velocities[i + 1];

         if ((i == 0) || (i == 4))
         {
            spline.setCubic(t0, tf, p0, pd0, pf, pdf);
         }

         else if (i == 1)
         {
            splines.get(0).compute(t0);
            FrameVector pdd0 = splines.get(0).getAccelerationCopy();
            spline.setQuarticUsingInitialAcceleration(t0, tf, p0, pd0, pdd0, pf, pdf);
         }

         else if (i == 3)
         {
            splines.get(4).compute(tf);
            FrameVector pddf = splines.get(4).getAccelerationCopy();
            spline.setQuarticUsingFinalAcceleration(t0, tf, p0, pd0, pf, pdf, pddf);
         }

         else if (i == 2)
         {
            splines.get(1).compute(t0);
            FrameVector pdd0 = splines.get(1).getAccelerationCopy();
            splines.get(3).compute(tf);
            FrameVector pddf = splines.get(3).getAccelerationCopy();
            spline.setQuintic(t0, tf, p0, pd0, pdd0, pf, pdf, pddf);
         }

         rangeList.get(i).getLeft().set(t0);
         rangeList.get(i).getRight().set(tf);
      }

      setArcLength();
   }

   public void setCubicQuinticCubic(double[] times, FramePoint[] positions, FrameVector[] velocities)
   {
      MathTools.checkIfEqual(rangeList.size(), 3);
      MathTools.checkIfEqual(times.length, 4);
      MathTools.checkIfEqual(positions.length, 4);
      MathTools.checkIfEqual(velocities.length, 4);

      int[] reorderedSplineIndices = new int[] {0, 2, 1};

      for (int i : reorderedSplineIndices)
      {
         YoSpline3D spline = splines.get(i);

         double t0 = times[i];
         double tf = times[i + 1];
         FramePoint p0 = positions[i];
         FramePoint pf = positions[i + 1];
         FrameVector pd0 = velocities[i];
         FrameVector pdf = velocities[i + 1];

         if ((i == 0) || (i == 2))
         {
            spline.setCubic(t0, tf, p0, pd0, pf, pdf);
         }
         else if (i == 1)
         {
            splines.get(0).compute(t0);
            FrameVector pdd0 = splines.get(0).getAccelerationCopy();
            splines.get(2).compute(tf);
            FrameVector pddf = splines.get(2).getAccelerationCopy();
            spline.setQuintic(t0, tf, p0, pd0, pdd0, pf, pdf, pddf);
         }

         rangeList.get(i).getLeft().set(t0);
         rangeList.get(i).getRight().set(tf);
      }

      setArcLength();
   }

   public void setQuarticQuinticQuartic(double[] times, FramePoint[] positions, FrameVector[] velocities, double[] intermediateTimes,
           FrameVector[] intermediateVelocities)
   {
      MathTools.checkIfEqual(rangeList.size(), 3);
      MathTools.checkIfEqual(times.length, 4);
      MathTools.checkIfEqual(positions.length, 4);
      MathTools.checkIfEqual(velocities.length, 4);
      MathTools.checkIfEqual(intermediateTimes.length, 2);
      MathTools.checkIfEqual(intermediateVelocities.length, 2);

      int[] reorderedSplineIndices = new int[] {0, 2, 1};

      for (int i : reorderedSplineIndices)
      {
         YoSpline3D spline = splines.get(i);

         double t0 = times[i];
         double tf = times[i + 1];
         FramePoint p0 = positions[i];
         FramePoint pf = positions[i + 1];
         FrameVector pd0 = velocities[i];
         FrameVector pdf = velocities[i + 1];

         if ((i == 0) || (i == 2))
         {
            double t1 = intermediateTimes[i / 2];
            FrameVector pd1 = intermediateVelocities[i / 2];
            spline.setQuarticUsingIntermediateVelocity(t0, t1, tf, p0, pd0, pd1, pf, pdf);
         }
         else if (i == 1)
         {
            splines.get(0).compute(t0);
            FrameVector pdd0 = splines.get(0).getAccelerationCopy();
            splines.get(2).compute(tf);
            FrameVector pddf = splines.get(2).getAccelerationCopy();
            spline.setQuintic(t0, tf, p0, pd0, pdd0, pf, pdf, pddf);
         }

         rangeList.get(i).getLeft().set(t0);
         rangeList.get(i).getRight().set(tf);
      }

      setArcLength();
   }

   public void setQuinticsUsingIntermediateVeloctiesAndAccelerations(double[] times, FramePoint[] positions, FrameVector[] velocities,
           double[] intermediateTimes, FrameVector[] intermediateVelocities, FrameVector[] intermediateAccelerations)
   {
      MathTools.checkIfEqual(rangeList.size(), 3);
      MathTools.checkIfEqual(times.length, 4);
      MathTools.checkIfEqual(positions.length, 4);
      MathTools.checkIfEqual(velocities.length, 4);
      MathTools.checkIfEqual(intermediateTimes.length, 2);
      MathTools.checkIfEqual(intermediateVelocities.length, 2);
      MathTools.checkIfEqual(intermediateAccelerations.length, 2);

      int[] reorderedSplineIndices = new int[] {0, 2, 1};

      for (int i : reorderedSplineIndices)
      {
         YoSpline3D spline = splines.get(i);

         double t0 = times[i];
         double tf = times[i + 1];
         FramePoint p0 = positions[i];
         FramePoint pf = positions[i + 1];
         FrameVector pd0 = velocities[i];
         FrameVector pdf = velocities[i + 1];

         if ((i == 0) || (i == 2))
         {
            double t1 = intermediateTimes[i / 2];
            FrameVector pd1 = intermediateVelocities[i / 2];
            FrameVector pdd1 = intermediateAccelerations[i / 2];
            spline.setQuinticUsingIntermediateVelocityAndAcceleration(t0, t1, tf, p0, pd0, pd1, pdd1, pf, pdf);
         }
         else if (i == 1)
         {
            splines.get(0).compute(t0);
            FrameVector pdd0 = splines.get(0).getAccelerationCopy();
            splines.get(2).compute(tf);
            FrameVector pddf = splines.get(2).getAccelerationCopy();
            spline.setQuintic(t0, tf, p0, pd0, pdd0, pf, pdf, pddf);
         }

         rangeList.get(i).getLeft().set(t0);
         rangeList.get(i).getRight().set(tf);
      }

      setArcLength();
   }

   public void setQuinticSexticQuinticUsingWaypointAndIntermediateVeloctiesAndAccelerations(double[] times, FramePoint[] positions, FrameVector[] velocities,
           double ghostTime, FramePoint ghostWaypoint, double[] intermediateTimes, FrameVector[] intermediateVelocities,
           FrameVector[] intermediateAccelerations)
   {
      MathTools.checkIfEqual(rangeList.size(), 3);
      MathTools.checkIfEqual(times.length, 4);
      MathTools.checkIfEqual(positions.length, 4);
      MathTools.checkIfEqual(velocities.length, 4);
      MathTools.checkIfEqual(intermediateTimes.length, 2);
      MathTools.checkIfEqual(intermediateVelocities.length, 2);
      MathTools.checkIfEqual(intermediateAccelerations.length, 2);

      int[] reorderedSplineIndices = new int[] {0, 2, 1};

      for (int i : reorderedSplineIndices)
      {
         YoSpline3D spline = splines.get(i);

         double t0 = times[i];
         double tf = times[i + 1];
         FramePoint p0 = positions[i];
         FramePoint pf = positions[i + 1];
         FrameVector pd0 = velocities[i];
         FrameVector pdf = velocities[i + 1];

         if ((i == 0) || (i == 2))
         {
            double t1 = intermediateTimes[i / 2];
            FrameVector pd1 = intermediateVelocities[i / 2];
            FrameVector pdd1 = intermediateAccelerations[i / 2];
            spline.setQuinticUsingIntermediateVelocityAndAcceleration(t0, t1, tf, p0, pd0, pd1, pdd1, pf, pdf);
         }
         else if (i == 1)
         {
            splines.get(0).compute(t0);
            FrameVector pdd0 = splines.get(0).getAccelerationCopy();
            splines.get(2).compute(tf);
            FrameVector pddf = splines.get(2).getAccelerationCopy();
            spline.setSexticUsingWaypoint(t0, ghostTime, tf, p0, pd0, pdd0, ghostWaypoint, pf, pdf, pddf);
         }

         rangeList.get(i).getLeft().set(t0);
         rangeList.get(i).getRight().set(tf);
      }

      setArcLength();
   }

   public void setSexticQuinticSexticUsingIntermediateVelocitiesAndAccelerations(double[] times, FramePoint[] positions, FrameVector[] velocities,
           FrameVector[] waypointAccelerations, double[] intermediateTimes, FrameVector[] intermediateVelocities, FrameVector[] intermediateAccelerations)
   {
      MathTools.checkIfEqual(rangeList.size(), 3);
      MathTools.checkIfEqual(times.length, 4);
      MathTools.checkIfEqual(positions.length, 4);
      MathTools.checkIfEqual(velocities.length, 4);
      MathTools.checkIfEqual(waypointAccelerations.length, 2);
      MathTools.checkIfEqual(intermediateTimes.length, 2);
      MathTools.checkIfEqual(intermediateVelocities.length, 2);
      MathTools.checkIfEqual(intermediateAccelerations.length, 2);

      int[] reorderedSplineIndices = new int[] {0, 2, 1};

      for (int i : reorderedSplineIndices)
      {
         YoSpline3D spline = splines.get(i);

         double t0 = times[i];
         double t1 = 0.0;
         double tf = times[i + 1];
         FramePoint p0 = positions[i];
         FramePoint pf = positions[i + 1];
         FrameVector pd1 = null;
         FrameVector pdd1 = null;
         FrameVector pd0 = velocities[i];
         FrameVector pdf = velocities[i + 1];

         if ((i == 0) || (i == 2))
         {
            t1 = intermediateTimes[i / 2];
            pd1 = intermediateVelocities[i / 2];
            pdd1 = intermediateAccelerations[i / 2];
         }

         if (i == 0)
         {
            FrameVector pddf = waypointAccelerations[0];
            spline.setSexticUsingWaypointVelocityAndAccelerationAndFinalAcceleration(t0, t1, tf, p0, pd0, pd1, pdd1, pf, pdf, pddf);
         }
         else if (i == 2)
         {
            FrameVector pdd0 = waypointAccelerations[1];
            spline.setSexticUsingWaypointVelocityAndAccelerationAndInitialAcceleration(t0, t1, tf, p0, pd0, pdd0, pd1, pdd1, pf, pdf);
         }

         else if (i == 1)
         {
            splines.get(0).compute(t0);
            FrameVector pdd0 = splines.get(0).getAccelerationCopy();
            splines.get(2).compute(tf);
            FrameVector pddf = splines.get(2).getAccelerationCopy();
            spline.setQuintic(t0, tf, p0, pd0, pdd0, pf, pdf, pddf);
         }

         rangeList.get(i).getLeft().set(t0);
         rangeList.get(i).getRight().set(tf);
      }

      setArcLength();
   }

   public void setQuintics(double[] times, FramePoint[] positions, FrameVector[] velocities, FrameVector[] accelerations)
   {
      MathTools.checkIfEqual(times.length, rangeList.size() + 1);
      MathTools.checkIfEqual(positions.length, times.length);
      MathTools.checkIfEqual(velocities.length, times.length);
      MathTools.checkIfEqual(accelerations.length, times.length);

      for (int i = 0; i < times.length - 1; i++)
      {
         YoSpline3D spline = splines.get(i);

         double t0 = times[i];
         double tf = times[i + 1];
         FramePoint p0 = positions[i];
         FramePoint pf = positions[i + 1];
         FrameVector pd0 = velocities[i];
         FrameVector pdf = velocities[i + 1];
         FrameVector pdd0 = accelerations[i];
         FrameVector pddf = accelerations[i + 1];

         spline.setQuintic(t0, tf, p0, pd0, pdd0, pf, pdf, pddf);

         rangeList.get(i).getLeft().set(t0);
         rangeList.get(i).getRight().set(tf);
      }

      setArcLength();
   }

   public void setQuintics(YoConcatenatedSplines oldSplines, double[] oldTimes, double[] newTimes)
   {
      MathTools.checkIfEqual(oldTimes.length, newTimes.length);
      MathTools.checkIfEqual(oldTimes.length, rangeList.size() + 1);

      for (int i = 0; i < oldTimes.length - 1; i++)
      {
         YoSpline3D spline = splines.get(i);

         oldSplines.compute(oldTimes[i]);
         double t0 = newTimes[i];
         FramePoint p0 = oldSplines.getPosition();
         FrameVector pd0 = oldSplines.getVelocity();
         FrameVector pdd0 = oldSplines.getAcceleration();

         oldSplines.compute(oldTimes[i + 1]);
         double tf = newTimes[i + 1];
         FramePoint pf = oldSplines.getPosition();
         FrameVector pdf = oldSplines.getVelocity();
         FrameVector pddf = oldSplines.getAcceleration();

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

   public FramePoint getPosition()
   {
      return position.getFramePointCopy();
   }

   public FrameVector getVelocity()
   {
      return velocity.getFrameVectorCopy();
   }

   public FrameVector getAcceleration()
   {
      return acceleration.getFrameVectorCopy();
   }

   public void getPosition(YoFramePoint positionToPack)
   {
      positionToPack.set(position);
   }

   public void getVelocity(YoFrameVector velocityToPack)
   {
      velocityToPack.set(velocity);
   }

   public void getAcceleration(YoFrameVector accelerationToPack)
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
      t = MathTools.clipToMinMax(t, rangeList.get(0).getLeft().getDoubleValue(), rangeList.get(rangeList.size() - 1).getRight().getDoubleValue());
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
