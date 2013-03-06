package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.BagOfBalls;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import com.yobotics.simulationconstructionset.util.trajectory.ConstantPositionProvider;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleProvider;
import com.yobotics.simulationconstructionset.util.trajectory.PositionProvider;
import com.yobotics.simulationconstructionset.util.trajectory.PositionTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.VectorProvider;

public class TwoWaypointPositionTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private final String namePostFix = getClass().getSimpleName();
   private final YoVariableRegistry registry;

   private boolean VISUALIZE = true;
   private final int numberOfVisualizationMarkers = 50;

   private final BagOfBalls trajectoryBagOfBalls;
   private final BagOfBalls waypointBagOfBalls;

   private final DoubleProvider stepTimeProvider;
   private final PositionProvider[] positionSource = new PositionProvider[4];
   private final VectorProvider[] velocitySource = new VectorProvider[4];

   private final DoubleYoVariable stepTime;
   private final DoubleYoVariable timeIntoStep;

   private final YoFramePoint desiredPosition;
   private final YoFrameVector desiredVelocity;
   private final YoFrameVector desiredAcceleration;
   private final ReferenceFrame referenceFrame;

   private final ConcatenatedSplines origianlConcatenatedSplines;
   private final ConcatenatedSplines respacedConcatenatedSplines;

   private final int desiredNumberOfSplines;

   public TwoWaypointPositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, DoubleProvider stepTimeProvider,
           PositionProvider initialPositionProvider, VectorProvider initalVelocityProvider, PositionProvider finalPositionProvider,
           VectorProvider finalDesiredVelocityProvider, YoVariableRegistry parentRegistry, PositionProvider firstIntermediatePositionProvider,
           PositionProvider secondIntermediatePositionProvider, int desiredNumberOfSplines, int arcLengthCalculatorDivisions,
           DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + namePostFix);
      parentRegistry.addChild(registry);
      trajectoryBagOfBalls = new BagOfBalls(numberOfVisualizationMarkers, 0.01, namePrefix + "TrajectoryBagOfBalls", registry, dynamicGraphicObjectsListRegistry);
      waypointBagOfBalls = new BagOfBalls(4, 0.02, namePrefix + "WaypointBagOfBalls", registry, dynamicGraphicObjectsListRegistry);

      this.stepTimeProvider = stepTimeProvider;

      this.referenceFrame = referenceFrame;

      this.positionSource[0] = initialPositionProvider;
      this.positionSource[1] = firstIntermediatePositionProvider;
      this.positionSource[2] = secondIntermediatePositionProvider;
      this.positionSource[3] = finalPositionProvider;

      this.velocitySource[0] = initalVelocityProvider;
      this.velocitySource[3] = finalDesiredVelocityProvider;
      this.velocitySource[1] = getFirstIntermediateVelocityProvider();
      this.velocitySource[2] = getSecondIntermediateVelocityProvider();

      this.stepTime = new DoubleYoVariable("stepTime", registry);
      this.timeIntoStep = new DoubleYoVariable("timeIntoStep", registry);

      this.desiredPosition = new YoFramePoint("desiredPosition", referenceFrame, registry);
      this.desiredVelocity = new YoFrameVector("desiredVelocity", referenceFrame, registry);
      this.desiredAcceleration = new YoFrameVector("desiredAcceleration", referenceFrame, registry);

      this.origianlConcatenatedSplines = new ConcatenatedSplines(new int[] {4, 6, 4}, referenceFrame, arcLengthCalculatorDivisions);

      int[] reparameterizedNumberOfCoefficientsPerPolynomial = new int[desiredNumberOfSplines];
      for (int i = 0; i < reparameterizedNumberOfCoefficientsPerPolynomial.length; i++)
      {
         reparameterizedNumberOfCoefficientsPerPolynomial[i] = 6;
      }

      this.respacedConcatenatedSplines = new ConcatenatedSplines(reparameterizedNumberOfCoefficientsPerPolynomial, referenceFrame, 2);

      this.desiredNumberOfSplines = desiredNumberOfSplines;
   }

   private VectorProvider getFirstIntermediateVelocityProvider()
   {
      // TODO hack
      FramePoint initial = new FramePoint(referenceFrame);
      FramePoint finall = new FramePoint(referenceFrame);
      FramePoint way1 = new FramePoint(referenceFrame);
      positionSource[0].get(initial);
      initial.changeFrame(referenceFrame);
      positionSource[3].get(finall);
      way1.set(finall);
      way1.sub(initial);
      way1.scale(1.0 / 3.0);
      way1.add(initial);
      way1.setZ(way1.getZ() + 0.3);

      FramePoint initialPosition = new FramePoint(referenceFrame);
      FramePoint firstWayPoint = new FramePoint(referenceFrame);
      FrameVector initialVelocity = new FrameVector(referenceFrame);
      positionSource[0].get(initialPosition);
//      positionSource[1].get(way1);
      velocitySource[0].get(initialVelocity);
      initialPosition.changeFrame(referenceFrame);
      way1.changeFrame(referenceFrame);
      initialVelocity.changeFrame(referenceFrame);

      FrameVector firstWayPointVelocity = new FrameVector(way1);
      firstWayPointVelocity.sub(initialPosition);
      firstWayPointVelocity.normalize();
      firstWayPointVelocity.scale(initialVelocity.length());
      
      return new ConstantVectorProvider(firstWayPointVelocity);
   }

   private VectorProvider getSecondIntermediateVelocityProvider()
   {
      // TODO hack
      FramePoint initial = new FramePoint(referenceFrame);
      FramePoint finall = new FramePoint(referenceFrame);
      FramePoint way2 = new FramePoint(referenceFrame);
      positionSource[0].get(initial);
      initial.changeFrame(referenceFrame);
      positionSource[3].get(finall);
      way2.set(finall);
      way2.sub(initial);
      way2.scale(2.0 / 3.0);
      way2.add(initial);
      way2.setZ(way2.getZ() + 0.3);

      FramePoint secondWayPoint = new FramePoint(referenceFrame);
      FramePoint finalPosition = new FramePoint(referenceFrame);
      FrameVector finalVelocity = new FrameVector(referenceFrame);
//      positionSource[2].get(way2);
      positionSource[3].get(finalPosition);
      velocitySource[3].get(finalVelocity);
      way2.changeFrame(referenceFrame);
      finalPosition.changeFrame(referenceFrame);
      finalVelocity.changeFrame(referenceFrame);

      FrameVector secondWayPointVelocity = new FrameVector(finalPosition);
      secondWayPointVelocity.sub(way2);
      secondWayPointVelocity.normalize();
      secondWayPointVelocity.scale(finalVelocity.length());
      
      return new ConstantVectorProvider(secondWayPointVelocity);
   }

   public void compute(double time)
   {
      timeIntoStep.set(time);

      double totalTime = stepTime.getDoubleValue();
      if (time > totalTime)
         time = totalTime;

      respacedConcatenatedSplines.compute(time);
      desiredPosition.set(respacedConcatenatedSplines.getPosition());
      desiredVelocity.set(respacedConcatenatedSplines.getVelocity());
      desiredAcceleration.set(respacedConcatenatedSplines.getAcceleration());
   }

   public void get(FramePoint positionToPack)
   {
      desiredPosition.getFramePointAndChangeFrameOfPackedPoint(positionToPack);
   }

   public void packVelocity(FrameVector velocityToPack)
   {
      desiredVelocity.getFrameVectorAndChangeFrameOfPackedVector(velocityToPack);
   }

   public void packAcceleration(FrameVector accelerationToPack)
   {
      desiredAcceleration.getFrameVectorAndChangeFrameOfPackedVector(accelerationToPack);
   }

   public void initialize()
   {
      System.out.println("A");
      this.velocitySource[1] = getFirstIntermediateVelocityProvider();
      this.velocitySource[2] = getSecondIntermediateVelocityProvider();
      double stepTime = stepTimeProvider.getValue();
      MathTools.checkIfInRange(stepTime, 0.0, Double.POSITIVE_INFINITY);
      this.stepTime.set(stepTime);
      timeIntoStep.set(0.0);

      setOriginalConcatenatedSplines();

      if (desiredNumberOfSplines == 3)
      {
         respaceSplineRangesProportionalToCurrentArcLengths(origianlConcatenatedSplines, respacedConcatenatedSplines);
      }
      else
      {
         respaceSplineRangesWithEqualArcLengths(origianlConcatenatedSplines, respacedConcatenatedSplines);
      }

      if (VISUALIZE)
      {
         visualizeSpline();
      }
      System.out.println("B");
   }

   private void visualizeSpline()
   {
      for (int i = 0; i < numberOfVisualizationMarkers; i++)
      {
         double t0 = respacedConcatenatedSplines.getT0();
         double tf = respacedConcatenatedSplines.getTf();
         double t = t0 + (double) i / (double) (numberOfVisualizationMarkers) * (tf - t0);
         compute(t);
         trajectoryBagOfBalls.setBall(desiredPosition.getFramePointCopy(), i);
      }
      FramePoint[] temp = new FramePoint[positionSource.length];
      for (int i = 0; i < positionSource.length; i++)
      {
         temp[i] = new FramePoint(referenceFrame);
         positionSource[i].get(temp[i]);
         temp[i].changeFrame(referenceFrame);
         waypointBagOfBalls.setBall(temp[i], YoAppearance.AliceBlue(), i);
      }
   }

   private void setOriginalConcatenatedSplines()
   {
      double[] times = new double[4];
      FramePoint[] positions = new FramePoint[4];
      FrameVector[] velocities = new FrameVector[4];

      for (int i = 0; i < 4; i++)
      {
         positions[i] = new FramePoint(referenceFrame);
         positionSource[i].get(positions[i]);
         positions[i].changeFrame(referenceFrame);

         velocities[i] = new FrameVector(referenceFrame);
         velocitySource[i].get(velocities[i]);
         velocities[i].changeFrame(referenceFrame);
      }

      // TODO hack
      positions[0].changeFrame(referenceFrame);
      FramePoint way1 = new FramePoint(referenceFrame);
      FramePoint way2 = new FramePoint(referenceFrame);
      way1.set(positions[3]);
      way1.sub(positions[0]);
      way1.scale(1.0 / 3.0);
      way2.set(way1);
      way2.scale(2.0);
      way1.add(positions[0]);
      way2.add(positions[0]);
      way1.setZ(way1.getZ() + 0.3);
      way2.setZ(way2.getZ() + 0.3);
      positions[1] = way1;
      positions[2] = way2;
      positionSource[1] = new ConstantPositionProvider(way1);
      positionSource[2] = new ConstantPositionProvider(way2);

      double[] distances = new double[4];
      double deltaDistance;

      distances[0] = 0.0;

      for (int i = 1; i < 4; i++)
      {
         deltaDistance = positions[i - 1].distance(positions[i]);
         distances[i] = distances[i - 1] + deltaDistance;
      }

      double totalDistance = distances[3];

      for (int i = 0; i < 4; i++)
      {
         times[i] = distances[i] * stepTime.getDoubleValue() / totalDistance;
      }

      origianlConcatenatedSplines.setCubicQuinticCubic(times, positions, velocities);
   }

   public void respaceSplineRangesProportionalToCurrentArcLengths(ConcatenatedSplines oldSplines, ConcatenatedSplines newSplines)
   {
      double[] oldTimes = new double[desiredNumberOfSplines + 1];
      double[] newTimes = new double[desiredNumberOfSplines + 1];

      double t0 = oldSplines.getT0();
      double tf = oldSplines.getTf();
      double totalTime = tf - t0;

      double totalArcLength = oldSplines.getArcLength();
      double cumulativeArcLength = 0.0;
      Spline3D oldSpline;

      oldTimes[0] = t0;
      newTimes[0] = t0;

      for (int i = 1; i < oldTimes.length - 1; i++)
      {
         oldSpline = oldSplines.getSplineByIndex(i - 1);
         oldTimes[i] = oldSpline.getTf();
         cumulativeArcLength += oldSpline.getArcLength();
         newTimes[i] = t0 + totalTime * cumulativeArcLength / totalArcLength;
      }

      oldTimes[oldTimes.length - 1] = tf;
      newTimes[newTimes.length - 1] = tf;
      
      newSplines.setQuintics(oldSplines, oldTimes, newTimes);
   }

   public void respaceSplineRangesWithEqualArcLengths(ConcatenatedSplines oldSplines, ConcatenatedSplines newSplines)
   {
      int desiredNumberOfSplines = newSplines.getNumberOfSplines();

      double[] oldTimes = new double[desiredNumberOfSplines + 1];
      double[] newTimes = new double[desiredNumberOfSplines + 1];

      double t0 = oldSplines.getT0();
      double tf = oldSplines.getTf();
      double totalTime = tf - t0;

      double totalArcLength = oldSplines.getArcLength();
      double desiredIndividualArcLength = totalArcLength / (double) desiredNumberOfSplines;

      oldTimes[0] = t0;
      newTimes[0] = t0;

      for (int i = 1; i < oldTimes.length - 1; i++)
      {
         oldTimes[i] = oldSplines.approximateTimeFromArcLength((double) i * desiredIndividualArcLength);
         newTimes[i] = t0 + totalTime * ((double) i) / ((double) desiredNumberOfSplines);
      }

      oldTimes[oldTimes.length - 1] = tf;
      newTimes[newTimes.length - 1] = tf;

      newSplines.setQuintics(oldSplines, oldTimes, newTimes);
   }

   public boolean isDone()
   {
      return timeIntoStep.getDoubleValue() >= stepTime.getDoubleValue();
   }
}
