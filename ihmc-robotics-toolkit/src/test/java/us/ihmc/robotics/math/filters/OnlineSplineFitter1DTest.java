package us.ihmc.robotics.math.filters;

import org.apache.commons.math3.fitting.PolynomialCurveFitter;
import org.apache.commons.math3.fitting.WeightedObservedPoint;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.robotics.math.filters.OnlineSplineFitter1D.DataPoint1D;
import us.ihmc.robotics.math.filters.OnlineSplineFitter1D.SplineFitter1D;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.DoubleUnaryOperator;
import java.util.stream.Collectors;

import static org.junit.jupiter.api.Assertions.*;

public class OnlineSplineFitter1DTest
{
   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   static
   {
      simulationTestingParameters.setCreateGUI(false);
      simulationTestingParameters.setDataBufferSize(1 << 16);
   }

   protected static final boolean visualize = simulationTestingParameters.getCreateGUI();

   private SimulationConstructionSet scs;

   @AfterEach
   public void teardown()
   {
      if (visualize)
         ThreadTools.sleepForever();

      if (scs != null)
      {
         scs.closeAndDispose();
         scs = null;
      }
   }

   @Test
   void testEstimateNoiseFreeSineWave()
   {
      double dt = 0.005;
      YoRegistry mainRegistry = new YoRegistry("main");

      if (visualize)
      {
         scs = new SimulationConstructionSet(new Robot("Dummy"), simulationTestingParameters);
         scs.addYoRegistry(mainRegistry);
      }

      YoDouble time = new YoDouble("time", mainRegistry);
      YoDouble originalSignal = new YoDouble("originalSignal", mainRegistry);
      YoDouble originalSignalRate = new YoDouble("originalSignalRate", mainRegistry);
      YoDouble originalSignalAcceleration = new YoDouble("originalSignalAcceleration", mainRegistry);
      YoDouble downsampledSignal = new YoDouble("downsampledSignal", mainRegistry);
      YoBoolean signalUpdated = new YoBoolean("signalUpdated", mainRegistry);

      YoDouble estimatedSignal = new YoDouble("estimatedSignal", mainRegistry);
      YoDouble estimatedSignalRate = new YoDouble("estimatedSignalRate", mainRegistry);
      YoDouble estimatedSignalAcceleration = new YoDouble("estimatedSignalAcceleration", mainRegistry);

      YoDouble correctionGainRate = new YoDouble("correctionGainRate", mainRegistry);
      correctionGainRate.set(1.0);
      OnlineSplineFitter1D polynomialFitter1D = new OnlineSplineFitter1D(5, 20, 0.1);

      double correctionGainRateBackup = correctionGainRate.getValue();
      double offset = 0.0;
      double amplitude = 1.0;
      double frequency = 1.0;
      DoubleUnaryOperator sineWaveGenerator = sineWaveGenerator(offset, amplitude, frequency);
      DoubleUnaryOperator sineWaveRateGenerator = sineWaveRateGenerator(offset, amplitude, frequency);
      DoubleUnaryOperator sineWaveAccelerationGenerator = sineWaveAccelerationGenerator(offset, amplitude, frequency);

      Random random = new Random(4638734);
      double minSamplePeriod = dt;
      double maxSamplePeriod = 10.0 * dt;
      double nextSampleTime = 0.0;

      for (double t = 0.0; t < 10.0; t += dt)
      {
         correctionGainRate.set(correctionGainRateBackup);

         time.set(t);
         originalSignal.set(sineWaveGenerator.applyAsDouble(t));
         originalSignalRate.set(sineWaveRateGenerator.applyAsDouble(t));
         originalSignalAcceleration.set(sineWaveAccelerationGenerator.applyAsDouble(t));

         if (nextSampleTime <= t)
         {
            downsampledSignal.set(originalSignal.getValue());
            nextSampleTime = t + EuclidCoreRandomTools.nextDouble(random, minSamplePeriod, maxSamplePeriod);
            signalUpdated.set(true);
         }
         else
         {
            signalUpdated.set(false);
         }

         if (signalUpdated.getValue())
            polynomialFitter1D.recordNewPoint(t, downsampledSignal.getValue());

         estimatedSignal.set(polynomialFitter1D.evaluateValueAt(t));
         estimatedSignalRate.set(polynomialFitter1D.evaluateRateAt(t));
         estimatedSignalAcceleration.set(polynomialFitter1D.evaluateAccelerationAt(t));

         if (polynomialFitter1D.isSplineInitialized())
         {
            double epsilonValue = 5.0e-3;
            double epsilonRate = 7.0e-2;

            assertEquals(originalSignal.getValue(),
                         estimatedSignal.getValue(),
                         epsilonValue,
                         "Error: " + Math.abs(originalSignal.getValue() - estimatedSignal.getValue()));
            assertEquals(originalSignalRate.getValue(),
                         estimatedSignalRate.getValue(),
                         epsilonRate,
                         "Error: " + Math.abs(originalSignalRate.getValue() - estimatedSignalRate.getValue()));

            if (signalUpdated.getValue())
            {
               assertEquals(t, polynomialFitter1D.getNewestPointTime());
            }
         }

         if (visualize)
            scs.tickAndUpdate();
      }

      if (visualize)
      {
         scs.hideViewport();
         scs.startOnAThread();
      }
   }

   @Test
   void testEstimateGaussianNoiseSineWave()
   {
      double dt = 0.005;
      YoRegistry mainRegistry = new YoRegistry("main");

      if (visualize)
      {
         scs = new SimulationConstructionSet(new Robot("Dummy"), simulationTestingParameters);
         scs.addYoRegistry(mainRegistry);
      }

      YoDouble time = new YoDouble("time", mainRegistry);
      YoDouble originalSignal = new YoDouble("originalSignal", mainRegistry);
      YoDouble originalSignalWithNoise = new YoDouble("originalSignalWithNoise", mainRegistry);
      YoDouble originalSignalRate = new YoDouble("originalSignalRate", mainRegistry);
      YoDouble originalSignalAcceleration = new YoDouble("originalSignalAcceleration", mainRegistry);
      YoDouble downsampledSignal = new YoDouble("downsampledSignal", mainRegistry);
      YoDouble filterAlpha = new YoDouble("fiterAlpha", mainRegistry);
      YoDouble downsampledSignalFiltered = new YoDouble("downsampledSignalFiltered", mainRegistry);
      YoDouble durationSinceLastUpdate = new YoDouble("durationSinceLastUpdate", mainRegistry);
      YoBoolean signalUpdated = new YoBoolean("signalUpdated", mainRegistry);

      YoDouble estimatedSignal = new YoDouble("estimatedSignal", mainRegistry);
      YoDouble estimatedSignalRate = new YoDouble("estimatedSignalRate", mainRegistry);
      YoDouble estimatedSignalAcceleration = new YoDouble("estimatedSignalAcceleration", mainRegistry);

      OnlineSplineFitter1D polynomialFitter1D = new OnlineSplineFitter1D(4, 20, 0.5);

      double filterFreq = 2.0;

      Random random = new Random(4638734);
      double offset = 0.0;
      double amplitude = 1.0;
      double frequency = 1.0;
      double noiseAmplitude = 1.0e-2;
      DoubleUnaryOperator rawSineWaveGenerator = sineWaveGenerator(offset, amplitude, frequency);
      DoubleUnaryOperator noisySineWaveGenerator = addGaussianNoise(random, noiseAmplitude, rawSineWaveGenerator);
      DoubleUnaryOperator sineWaveRateGenerator = sineWaveRateGenerator(offset, amplitude, frequency);
      DoubleUnaryOperator sineWaveAccelerationGenerator = sineWaveAccelerationGenerator(offset, amplitude, frequency);

      double minSamplePeriod = dt;
      double maxSamplePeriod = 10.0 * dt;
      double nextSampleTime = 0.0;

      for (double t = 0.0; t < 10.0; t += dt)
      {
         time.set(t);
         originalSignal.set(rawSineWaveGenerator.applyAsDouble(t));
         originalSignalWithNoise.set(noisySineWaveGenerator.applyAsDouble(t));
         originalSignalRate.set(sineWaveRateGenerator.applyAsDouble(t));
         originalSignalAcceleration.set(sineWaveAccelerationGenerator.applyAsDouble(t));

         if (nextSampleTime <= t)
         {
            downsampledSignal.set(originalSignalWithNoise.getValue());
            nextSampleTime = t + EuclidCoreRandomTools.nextDouble(random, minSamplePeriod, maxSamplePeriod);
            signalUpdated.set(true);
         }
         else
         {
            signalUpdated.set(false);
         }

         //         if (signalUpdated.getValue())
         {
            filterAlpha.set(computeAlphaGivenBreakFrequencyProperly(filterFreq, dt));
            downsampledSignalFiltered.set(EuclidCoreTools.interpolate(downsampledSignal.getValue(),
                                                                      downsampledSignalFiltered.getValue(),
                                                                      filterAlpha.getValue()));
         }

         if (signalUpdated.getValue())
            durationSinceLastUpdate.set(0.0);
         else
            durationSinceLastUpdate.add(dt);

         if (signalUpdated.getValue())
            polynomialFitter1D.recordNewPoint(t, downsampledSignal.getValue());

         estimatedSignal.set(polynomialFitter1D.evaluateValueAt(t));
         estimatedSignalRate.set(polynomialFitter1D.evaluateRateAt(t));
         estimatedSignalAcceleration.set(polynomialFitter1D.evaluateAccelerationAt(t));

         if (visualize)
            scs.tickAndUpdate();
      }

      if (visualize)
      {
         scs.hideViewport();
         scs.startOnAThread();
      }
   }

   public static double computeAlphaGivenBreakFrequencyProperly(double breakFrequencyInHertz, double dt)
   {
      if (Double.isInfinite(breakFrequencyInHertz))
         return 0.0;

      double omega = 2.0 * Math.PI * breakFrequencyInHertz;
      double alpha = (1.0 - omega * dt / 2.0) / (1.0 + omega * dt / 2.0);
      alpha = MathTools.clamp(alpha, 0.0, 1.0);
      return alpha;
   }

   private DoubleUnaryOperator addGaussianNoise(Random random, double noiseAmplitude, DoubleUnaryOperator rawSignal)
   {
      return new DoubleUnaryOperator()
      {
         @Override
         public double applyAsDouble(double time)
         {
            double noise = noiseAmplitude * (2.0 * random.nextGaussian() - 1.0);
            return rawSignal.applyAsDouble(time) + noise;
         }
      };
   }

   private DoubleUnaryOperator sineWaveGenerator(double offset, double amplitude, double frequency)
   {
      return time -> offset + amplitude * Math.sin(2.0 * Math.PI * frequency * time);
   }

   private DoubleUnaryOperator sineWaveRateGenerator(double offset, double amplitude, double frequency)
   {
      return time -> 2.0 * Math.PI * amplitude * Math.cos(2.0 * Math.PI * frequency * time);
   }

   private DoubleUnaryOperator sineWaveAccelerationGenerator(double offset, double amplitude, double frequency)
   {
      return time -> -MathTools.square(2.0 * Math.PI) * amplitude * Math.sin(2.0 * Math.PI * frequency * time);
   }

   /////////////////////////////////////////////////////////////////////////////////////////////
   //////////////////////////////// Test for the spline fitting  ///////////////////////////////
   /////////////////////////////////////////////////////////////////////////////////////////////

   @Test
   void testCompareAgainstApache()
   {
      Random random = new Random(345346);

      int order = 5;
      PolynomialCurveFitter polynomialCurveFitter = PolynomialCurveFitter.create(order);
      SplineFitter1D splineFitter = new SplineFitter1D();
      splineFitter.setOrder(order);
      splineFitter.setRegularizationWeight(0.0);

      { // Simple constant function
         List<DataPoint1D> points = new ArrayList<>();
         int numberOfPoints = 10;
         double constant = random.nextDouble();

         for (int i = 0; i < numberOfPoints; i++)
         {
            DataPoint1D point = new DataPoint1D();
            point.setTime(i);
            point.setValue(constant);
            points.add(point);
         }

         splineFitter.clear();
         splineFitter.addPoints(points);
         splineFitter.fit();
         double[] actualCoefficients = splineFitter.getCoefficients();
         double[] expectedCoefficients = polynomialCurveFitter.fit(normalizePoints(toWeightedObservedPoints(points)));
         assertArrayEquals(expectedCoefficients, actualCoefficients, 1.0e-5);
      }

      { // Simple affine function
         List<DataPoint1D> points = new ArrayList<>();
         int numberOfPoints = 10;

         double offset = random.nextDouble();
         double factor = random.nextDouble();

         for (int i = 0; i < numberOfPoints; i++)
         {
            DataPoint1D point = new DataPoint1D();
            point.setTime(i);
            point.setValue(factor * i + offset);
            points.add(point);
         }

         splineFitter.clear();
         splineFitter.addPoints(points);
         splineFitter.fit();
         double[] actualCoefficients = splineFitter.getCoefficients();
         double[] expectedCoefficients = polynomialCurveFitter.fit(normalizePoints(toWeightedObservedPoints(points)));
         assertArrayEquals(expectedCoefficients, actualCoefficients, 1.0e-5);
      }

      { // Random points
         List<DataPoint1D> points = new ArrayList<>();
         int numberOfPoints = 10;

         for (int i = 0; i < numberOfPoints; i++)
         {
            DataPoint1D point = new DataPoint1D();
            point.setTime(i);
            point.setValue(random.nextDouble());
            points.add(point);
         }

         splineFitter.clear();
         splineFitter.addPoints(points);
         splineFitter.fit();
         double[] actualCoefficients = splineFitter.getCoefficients();
         double[] expectedCoefficients = polynomialCurveFitter.fit(normalizePoints(toWeightedObservedPoints(points)));
         assertArrayEquals(expectedCoefficients, actualCoefficients, 1.0e-5);
      }

      { // Sine points
         List<DataPoint1D> points = new ArrayList<>();
         int numberOfPoints = 10;
         double offset = 0.0;
         double amplitude = 1.0;
         double frequency = 1.0;
         double dt = 0.005;

         for (int i = 0; i < numberOfPoints; i++)
         {
            double time = i * dt;
            DataPoint1D point = new DataPoint1D();
            point.setTime(time);
            point.setValue(offset + amplitude * Math.sin(2.0 * Math.PI * frequency * time));
            points.add(point);
         }

         splineFitter.clear();
         splineFitter.addPoints(points);
         splineFitter.fit();
         double[] actualCoefficients = splineFitter.getCoefficients();
         double[] expectedCoefficients = polynomialCurveFitter.fit(normalizePoints(toWeightedObservedPoints(points)));
         for (int i = 0; i < expectedCoefficients.length; i++)
         {
            assertEquals(expectedCoefficients[i], actualCoefficients[i], Math.max(1.0, expectedCoefficients[i]) * 1.0e-5);
         }
      }
   }

   @Test
   public void testAgainstPolynomial()
   {
      Random random = new Random(345346);

      int order = 5;
      SplineFitter1D splineFitter = new SplineFitter1D();
      splineFitter.setOrder(order);
      splineFitter.setRegularizationWeight(0.0);

      { // Simple constant function
         List<DataPoint1D> points = new ArrayList<>();
         int numberOfPoints = 10;
         double constant = random.nextDouble();

         for (int i = 0; i < numberOfPoints; i++)
         {
            DataPoint1D point = new DataPoint1D();
            point.setTime(i);
            point.setValue(constant);
            points.add(point);
         }

         splineFitter.clear();
         splineFitter.addPoints(points);
         splineFitter.fit();
         Polynomial polynomial = constructPolynomialWithApacheFitter(order, points);
         double tEnd = points.get(numberOfPoints - 1).getTime();
         double dt = tEnd / 100.0;
         assertPolynomialSimilar(0.0, tEnd, dt, polynomial, splineFitter, 1.0e-5, 1.0e-5, 1.0e-5);
      }

      { // Simple affine function
         List<DataPoint1D> points = new ArrayList<>();
         int numberOfPoints = 10;

         double offset = random.nextDouble();
         double factor = random.nextDouble();

         for (int i = 0; i < numberOfPoints; i++)
         {
            DataPoint1D point = new DataPoint1D();
            point.setTime(i);
            point.setValue(factor * i + offset);
            points.add(point);
         }

         splineFitter.clear();
         splineFitter.addPoints(points);
         splineFitter.fit();
         Polynomial polynomial = constructPolynomialWithApacheFitter(order, points);
         double tEnd = points.get(numberOfPoints - 1).getTime();
         double dt = tEnd / 100.0;
         assertPolynomialSimilar(0.0, tEnd, dt, polynomial, splineFitter, 1.0e-5, 1.0e-5, 1.0e-5);
      }

      { // Random points
         List<DataPoint1D> points = new ArrayList<>();
         int numberOfPoints = 10;

         for (int i = 0; i < numberOfPoints; i++)
         {
            DataPoint1D point = new DataPoint1D();
            point.setTime(i);
            point.setValue(random.nextDouble());
            points.add(point);
         }

         splineFitter.clear();
         splineFitter.addPoints(points);
         splineFitter.fit();
         Polynomial polynomial = constructPolynomialWithApacheFitter(order, points);
         double tEnd = points.get(numberOfPoints - 1).getTime();
         double dt = tEnd / 100.0;
         assertPolynomialSimilar(0.0, tEnd, dt, polynomial, splineFitter, 1.0e-5, 1.0e-5, 1.0e-5);
      }

      { // Sine points
         List<DataPoint1D> points = new ArrayList<>();
         int numberOfPoints = 10;
         double offset = 0.0;
         double amplitude = 1.0;
         double frequency = 1.0;
         double dt = 0.005;

         for (int i = 0; i < numberOfPoints; i++)
         {
            double time = i * dt;
            DataPoint1D point = new DataPoint1D();
            point.setTime(time);
            point.setValue(offset + amplitude * Math.sin(2.0 * Math.PI * frequency * time));
            points.add(point);
         }

         splineFitter.clear();
         splineFitter.addPoints(points);
         splineFitter.fit();
         Polynomial polynomial = constructPolynomialWithApacheFitter(order, points);
         double tEnd = points.get(numberOfPoints - 1).getTime();
         assertPolynomialSimilar(0.0, tEnd, tEnd / 100.0, polynomial, splineFitter, 1.0e-5, 1.0e-5, 1.0e-5);
      }
   }

   private static Polynomial constructPolynomialWithApacheFitter(int degree, List<DataPoint1D> pointsToFit)
   {
      double[] coefficients = PolynomialCurveFitter.create(degree).fit(toWeightedObservedPoints(pointsToFit));
      reverse(coefficients);
      return new Polynomial(coefficients);
   }

   public static void reverse(double[] array)
   {
      for (int i = 0, mid = array.length >> 1, j = array.length - 1; i < mid; i++, j--)
      {
         double oldCoefficient_i = array[i];
         array[i] = array[j];
         array[j] = oldCoefficient_i;
      }
   }

   private static void assertPolynomialSimilar(double tStart,
                                               double tEnd,
                                               double dt,
                                               Polynomial polynomial,
                                               SplineFitter1D splineFitter,
                                               double epsilonValue,
                                               double epsilonRate,
                                               double epsilonAcceleration)
   {
      for (double t = tStart; t <= tEnd; t += dt)
      {
         assertEquals(polynomial.evaluate(t), splineFitter.evaluateValueAt(t), epsilonValue);
         assertEquals(polynomial.evaluateDerivative(t), splineFitter.evaluateRateAt(t), epsilonRate);
         assertEquals(polynomial.evaluateDoubleDerivative(t), splineFitter.evaluateAccelerationAt(t), epsilonAcceleration);
      }
   }

   private static List<WeightedObservedPoint> normalizePoints(List<WeightedObservedPoint> unnormalizedPoints)
   {
      double minX = unnormalizedPoints.stream().mapToDouble(WeightedObservedPoint::getX).min().getAsDouble();
      double maxX = unnormalizedPoints.stream().mapToDouble(WeightedObservedPoint::getX).max().getAsDouble();

      return unnormalizedPoints.stream().map(unnormalizedPoint ->
                                             {
                                                double weight = unnormalizedPoint.getWeight();
                                                double x = (unnormalizedPoint.getX() - minX) / (maxX - minX);
                                                double y = unnormalizedPoint.getY();
                                                return new WeightedObservedPoint(weight, x, y);
                                             }).collect(Collectors.toList());
   }

   private static List<WeightedObservedPoint> toWeightedObservedPoints(List<DataPoint1D> points)
   {
      return points.stream().map(OnlineSplineFitter1DTest::toWeightedObservedPoint).collect(Collectors.toList());
   }

   private static WeightedObservedPoint toWeightedObservedPoint(DataPoint1D point)
   {
      return new WeightedObservedPoint(point.getWeight(), point.getTime(), point.getValue());
   }

   private static class Polynomial
   {
      double[] a;

      public Polynomial(double[] coefficients)
      {
         a = coefficients;
      }

      public double evaluateDoubleDerivative(double t)
      {
         double result = 0.0;

         for (int i = 2; i < a.length; i++)
         {
            result += i * (i - 1.0) * a[a.length - 1 - i] * Math.pow(t, i - 2);
         }
         return result;
      }

      public double evaluateDerivative(double t)
      {
         double result = 0.0;

         for (int i = 1; i < a.length; i++)
         {
            result += i * a[a.length - 1 - i] * Math.pow(t, i - 1);
         }
         return result;
      }

      public double evaluate(double t)
      {
         double result = 0.0;

         for (int i = 0; i < a.length; i++)
         {
            result += a[a.length - 1 - i] * Math.pow(t, i);
         }
         return result;
      }
   }
}