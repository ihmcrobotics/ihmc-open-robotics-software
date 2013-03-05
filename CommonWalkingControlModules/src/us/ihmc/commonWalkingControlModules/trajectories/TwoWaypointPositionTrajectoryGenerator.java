package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleProvider;
import com.yobotics.simulationconstructionset.util.trajectory.PositionProvider;
import com.yobotics.simulationconstructionset.util.trajectory.PositionTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.VectorProvider;

public class TwoWaypointPositionTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private final String namePostFix = getClass().getSimpleName();
   private final YoVariableRegistry registry;

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
           PositionProvider secondIntermediatePositionProvider, int desiredNumberOfSplines, int arcLengthCalculatorDivisions)
   {
      this.registry = new YoVariableRegistry(namePrefix + namePostFix);
      parentRegistry.addChild(registry);

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

      this.respacedConcatenatedSplines = new ConcatenatedSplines(reparameterizedNumberOfCoefficientsPerPolynomial, referenceFrame,
              arcLengthCalculatorDivisions);

      this.desiredNumberOfSplines = desiredNumberOfSplines;
   }

   // TODO smarter method
   private VectorProvider getFirstIntermediateVelocityProvider()
   {
      FramePoint initialPosition = new FramePoint(referenceFrame);
      FramePoint firstWayPoint = new FramePoint(referenceFrame);
      FrameVector initialVelocity = new FrameVector(referenceFrame);
      positionSource[0].get(initialPosition);
      positionSource[1].get(firstWayPoint);
      velocitySource[0].get(initialVelocity);

      FrameVector firstWayPointVelocity = new FrameVector(firstWayPoint);
      firstWayPointVelocity.sub(initialPosition);
      firstWayPointVelocity.normalize();
      firstWayPointVelocity.scale(initialVelocity.length());

      return new ConstantVectorProvider(firstWayPointVelocity);
   }

   // TODO smarter method
   private VectorProvider getSecondIntermediateVelocityProvider()
   {
      FramePoint secondWayPoint = new FramePoint(referenceFrame);
      FramePoint finalPosition = new FramePoint(referenceFrame);
      FrameVector finalVelocity = new FrameVector(referenceFrame);
      positionSource[2].get(secondWayPoint);
      positionSource[3].get(finalPosition);
      velocitySource[3].get(finalVelocity);

      FrameVector secondWayPointVelocity = new FrameVector(finalPosition);
      secondWayPointVelocity.sub(secondWayPoint);
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
      double stepTime = stepTimeProvider.getValue();
      MathTools.checkIfInRange(stepTime, 0.0, Double.POSITIVE_INFINITY);
      this.stepTime.set(stepTime);
      timeIntoStep.set(0.0);

      setOriginalConcatenatedSplines();

      if (desiredNumberOfSplines == 3)
      {
         respaceSplineRangesProportionalToCurrentArcLengths();
      }
      else
      {
         respaceSplineRangesWithEqualArcLengths();
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

   public void respaceSplineRangesProportionalToCurrentArcLengths()
   {
      double[] oldTimes = new double[desiredNumberOfSplines + 1];
      double[] newTimes = new double[desiredNumberOfSplines + 1];

      double t0 = origianlConcatenatedSplines.getT0();
      double tf = origianlConcatenatedSplines.getTf();
      double totalTime = tf - t0;

      double totalArcLength = origianlConcatenatedSplines.getArcLength();
      double cumulativeArcLength = 0.0;
      Spline3D oldSpline;

      oldTimes[0] = t0;
      newTimes[0] = t0;

      for (int i = 1; i < oldTimes.length - 1; i++)
      {
         oldSpline = origianlConcatenatedSplines.getSplineByIndex(i);
         oldTimes[i] = oldSpline.getT0();
         newTimes[i] = t0 + totalTime * cumulativeArcLength / totalArcLength;
         cumulativeArcLength += oldSpline.getArcLength();
      }

      oldTimes[oldTimes.length - 1] = tf;
      newTimes[newTimes.length - 1] = tf;

      respacedConcatenatedSplines.setQuintics(origianlConcatenatedSplines, oldTimes, newTimes);
   }

   public void respaceSplineRangesWithEqualArcLengths()
   {
      int desiredNumberOfSplines = respacedConcatenatedSplines.getNumberOfSplines();

      double[] oldTimes = new double[desiredNumberOfSplines + 1];
      double[] newTimes = new double[desiredNumberOfSplines + 1];

      double t0 = origianlConcatenatedSplines.getT0();
      double tf = origianlConcatenatedSplines.getTf();
      double totalTime = tf - t0;

      double totalArcLength = origianlConcatenatedSplines.getArcLength();
      double desiredIndividualArcLength = totalArcLength / (double) desiredNumberOfSplines;

      oldTimes[0] = t0;
      newTimes[0] = t0;

      for (int i = 1; i < oldTimes.length - 1; i++)
      {
         oldTimes[i] = origianlConcatenatedSplines.approximateTimeFromArcLength((double) i * desiredIndividualArcLength);
         newTimes[i] = t0 + totalTime * ((double) i) / ((double) desiredNumberOfSplines);
      }

      oldTimes[oldTimes.length - 1] = tf;
      newTimes[newTimes.length - 1] = tf;

      respacedConcatenatedSplines.setQuintics(origianlConcatenatedSplines, null, null);
   }

   public boolean isDone()
   {
      return timeIntoStep.getDoubleValue() >= stepTime.getDoubleValue();
   }
}
