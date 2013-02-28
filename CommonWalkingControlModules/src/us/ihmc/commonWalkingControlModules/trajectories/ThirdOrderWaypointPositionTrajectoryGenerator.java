package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.TreeMap;

import us.ihmc.utilities.Pair;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.Direction;
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
import com.yobotics.simulationconstructionset.util.trajectory.YoPolynomial;

public class ThirdOrderWaypointPositionTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private final String namePostFix = getClass().getSimpleName();
   private final YoVariableRegistry registry;
   private final List<EnumMap<Direction, YoPolynomial>> spaceSplines = new ArrayList<EnumMap<Direction, YoPolynomial>>();

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
   private final double allowedPercentDifferenceBetweenDesiredAndActualArcLengths;
   private final int arcLengthPrecisionRating;

   private ConcatenatedSplines concatenatedSplines;

   public ThirdOrderWaypointPositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, DoubleProvider stepTimeProvider,
           PositionProvider initialPositionProvider, VectorProvider initalVelocityProvider, PositionProvider finalPositionProvider,
           VectorProvider finalDesiredVelocityProvider, YoVariableRegistry parentRegistry, PositionProvider firstIntermediatePosition,
           PositionProvider secondIntermediatePosition, int desiredNumberOfSplines, double allowedPercentDifferenceBetweenDesiredAndActualArcLengths, int arcLengthPrecisionRating)
   {
      this.registry = new YoVariableRegistry(namePrefix + namePostFix);

      for (int i = 0; i < 3; i++)
      {
         spaceSplines.set(i, new EnumMap<Direction, YoPolynomial>(Direction.class));

         for (Direction direction : Direction.values())
         {
            spaceSplines.get(i).put(direction, new YoPolynomial(namePrefix + direction + i, 3, registry));
         }
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
      parentRegistry.addChild(registry);

      this.desiredNumberOfSplines = desiredNumberOfSplines;
      this.allowedPercentDifferenceBetweenDesiredAndActualArcLengths = allowedPercentDifferenceBetweenDesiredAndActualArcLengths;
      this.arcLengthPrecisionRating = arcLengthPrecisionRating;
   }

   //TODO smarter method
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
   
   //TODO smarter method
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

      double totalTime = this.stepTime.getDoubleValue();
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

      for (Direction direction : Direction.values())
      {
         double[] sValues = new double[] {0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0};

         int[] cubicSplines = new int[] {0, 2, 1};
         for (int i : cubicSplines)
         {
            double s0 = sValues[i];
            double sf = sValues[i + 1];
            double z0 = positions[i].get(direction);
            double zd0 = velocities[i].get(direction);
            double zf = positions[i + 1].get(direction);
            double zdf = velocities[i + 1].get(direction);
            YoPolynomial spaceSpline = spaceSplines.get(i).get(direction);
            if ((i == 0) || (i == 2))
            {
               spaceSpline.setCubic(s0, sf, z0, zd0, zf, zdf);
            }
            else if (i == 1)
            {
               spaceSplines.get(0).get(direction).compute(s0);
               double zdd0 = spaceSplines.get(0).get(direction).getAcceleration();
               spaceSplines.get(2).get(direction).compute(sf);
               double zddf = spaceSplines.get(2).get(direction).getAcceleration();

               spaceSpline.setQuintic(s0, sf, z0, zd0, zdd0, zf, zdf, zddf);
            }
         }

         TreeMap<Pair<Double, Double>, EnumMap<Direction, YoPolynomial>> splineMap = new TreeMap<Pair<Double, Double>, EnumMap<Direction, YoPolynomial>>();
         splineMap.put(new Pair<Double, Double>(0.0, stepTime / 3.0), spaceSplines.get(0));
         splineMap.put(new Pair<Double, Double>(stepTime / 3.0, (2.0 * stepTime) / 3.0), spaceSplines.get(1));
         splineMap.put(new Pair<Double, Double>((2.0 * stepTime) / 3.0, stepTime), spaceSplines.get(2));

         concatenatedSplines = new ConcatenatedSplines(splineMap, referenceFrame, arcLengthPrecisionRating);
         concatenatedSplines = new ConcatenatedSplines(concatenatedSplines, desiredNumberOfSplines, allowedPercentDifferenceBetweenDesiredAndActualArcLengths, arcLengthPrecisionRating);
      }
   }

   public boolean isDone()
   {
      return timeIntoStep.getDoubleValue() >= stepTime.getDoubleValue();
   }
}
