package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.TreeMap;

import us.ihmc.utilities.Pair;
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

public class ThirdOrderWaypointPositionTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private final String namePostFix = getClass().getSimpleName();
   private final YoVariableRegistry registry;
   private final Spline3D[] splines = new Spline3D[3];

   private final DoubleProvider stepTimeProvider;
   private final PositionProvider[] positionSource = new PositionProvider[4];
   private final VectorProvider[] velocitySource = new VectorProvider[4];

   private final DoubleYoVariable stepTime;
   private final DoubleYoVariable timeIntoStep;

   private final YoFramePoint desiredPosition;
   private final YoFrameVector desiredVelocity;
   private final YoFrameVector desiredAcceleration;
   private final ReferenceFrame referenceFrame;

   private final int desiredNumberOfSplines;
   private final int arcLengthCalculatorDivisions;

   private ConcatenatedSplines concatenatedSplines;

   public ThirdOrderWaypointPositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, DoubleProvider stepTimeProvider,
           PositionProvider initialPositionProvider, VectorProvider initalVelocityProvider, PositionProvider finalPositionProvider,
           VectorProvider finalDesiredVelocityProvider, YoVariableRegistry parentRegistry, PositionProvider firstIntermediatePosition,
           PositionProvider secondIntermediatePosition, int desiredNumberOfSplines, int arcLengthCalculatorDivisions)
   {
      this.registry = new YoVariableRegistry(namePrefix + namePostFix);
      parentRegistry.addChild(registry);

      int[] numberOfCoefficientsForSplines = new int[] {4, 6, 4};

      for (int i = 0; i < 3; i++)
      {
         splines[i] = new Spline3D(numberOfCoefficientsForSplines[i], arcLengthCalculatorDivisions, referenceFrame);
      }

      this.stepTimeProvider = stepTimeProvider;

      this.positionSource[0] = initialPositionProvider;
      this.positionSource[1] = firstIntermediatePosition;
      this.positionSource[2] = secondIntermediatePosition;
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

      this.referenceFrame = referenceFrame;

      this.desiredNumberOfSplines = desiredNumberOfSplines;
      this.arcLengthCalculatorDivisions = arcLengthCalculatorDivisions;
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

      concatenatedSplines.compute(time);
      desiredPosition.set(concatenatedSplines.getPosition());
      desiredVelocity.set(concatenatedSplines.getVelocity());
      desiredAcceleration.set(concatenatedSplines.getAcceleration());
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

      timeIntoStep.set(0.0);

      double[] distances = new double[4];
      double[] times = new double[4];
      double totalDist = 0;

      distances[0] = 0.0;

      for (int i = 1; i < 4; i++)
      {
         double distInc = positions[i - 1].distance(positions[i]);
         totalDist += distInc;
         distances[i] = totalDist;
      }

      for (int i = 0; i < 4; i++)
      {
         times[i] = distances[i] * stepTime / totalDist;
      }

      int[] cubicSplines = new int[] {0, 2, 1};
      for (int i : cubicSplines)
      {
         double t0 = times[i];
         double tf = times[i + 1];
         FramePoint z0 = positions[i];
         FrameVector zd0 = velocities[i];
         FramePoint zf = positions[i + 1];
         FrameVector zdf = velocities[i + 1];
         Spline3D spline = splines[i];
         if ((i == 0) || (i == 2))
         {
            spline.setCubic(t0, tf, z0, zd0, zf, zdf);
         }
         else if (i == 1)
         {
            FrameVector zdd0 = splines[0].getAcceleration(t0);
            FrameVector zddf = splines[2].getAcceleration(tf);

            spline.setQuintic(t0, tf, z0, zd0, zdd0, zf, zdf, zddf);
         }
      }

      TreeMap<Pair<Double, Double>, Spline3D> splineMap = new TreeMap<Pair<Double, Double>, Spline3D>(ConcatenatedSplines.getComparatorForTreeMap());
      for (int i = 0; i < 3; i++)
      {
         splineMap.put(new Pair<Double, Double>(times[i], times[i + 1]), splines[i]);
      }

      concatenatedSplines = new ConcatenatedSplines(splineMap, referenceFrame, arcLengthCalculatorDivisions);
      concatenatedSplines = new ConcatenatedSplines(concatenatedSplines, desiredNumberOfSplines, arcLengthCalculatorDivisions);
   }

   public boolean isDone()
   {
      return timeIntoStep.getDoubleValue() >= stepTime.getDoubleValue();
   }
}
