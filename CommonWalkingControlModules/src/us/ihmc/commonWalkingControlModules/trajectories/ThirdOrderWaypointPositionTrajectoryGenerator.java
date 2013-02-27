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
import com.yobotics.simulationconstructionset.util.trajectory.VectorProvider;
import com.yobotics.simulationconstructionset.util.trajectory.YoPolynomial;

public class ThirdOrderWaypointPositionTrajectoryGenerator
{
   private final String namePostFix = getClass().getSimpleName();
   private final YoVariableRegistry registry;
   private final List<EnumMap<Direction, YoPolynomial>> spaceSplines = new ArrayList<EnumMap<Direction, YoPolynomial>>();
   private final YoPolynomial timeSpline;

   private final DoubleProvider stepTimeProvider;
   private final PositionProvider[] positionSource = new PositionProvider[4];
   private final VectorProvider[] velocitySource = new VectorProvider[4];

   private final DoubleYoVariable stepTime;
   private final DoubleYoVariable timeIntoStep;

   private final double obstacleWidth;
   private final double obstacleLength;
   private final double obstacleHeight;

   private final YoFramePoint desiredPosition;
   private final YoFrameVector desiredVelocity;
   private final YoFrameVector desiredAcceleration;

   private final ReferenceFrame obstacleReferenceFrame;
   private final ReferenceFrame referenceFrame;
   private final FrameVector tempVector = new FrameVector(ReferenceFrame.getWorldFrame());
   
   private final int desiredNumberOfSplines;
   private final double tolerance;

   private ConcatenatedSplines concatenatedSplines;

   public ThirdOrderWaypointPositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, DoubleProvider stepTimeProvider,
           PositionProvider initialPositionProvider, VectorProvider initalVelocityProvider, PositionProvider finalPositionProvider,
           VectorProvider finalDesiredVelocityProvider, YoVariableRegistry parentRegistry, double obstacleWidth, double obstacleLength, double obstacleHeight,
           ReferenceFrame obstacleReferenceFrame, PositionProvider firstIntermediatePosition, PositionProvider secondIntermediatePosition,
           int desiredNumberOfSplines, int tolerance)
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

      timeSpline = new YoPolynomial(namePrefix + "Time", 3, registry);
      this.stepTimeProvider = stepTimeProvider;

      this.positionSource[0] = initialPositionProvider;
      this.positionSource[1] = firstIntermediatePosition;
      this.positionSource[2] = secondIntermediatePosition;
      this.positionSource[3] = finalPositionProvider;

      this.velocitySource[0] = initalVelocityProvider;

      // TODO make intermed velocity
      this.velocitySource[3] = finalDesiredVelocityProvider;

      this.stepTime = new DoubleYoVariable("stepTime", registry);
      this.timeIntoStep = new DoubleYoVariable("timeIntoStep", registry);

      this.desiredPosition = new YoFramePoint("desiredPosition", referenceFrame, registry);
      this.desiredVelocity = new YoFrameVector("desiredVelocity", referenceFrame, registry);
      this.desiredAcceleration = new YoFrameVector("desiredAcceleration", referenceFrame, registry);

      this.referenceFrame = referenceFrame;
      parentRegistry.addChild(registry);

      this.obstacleHeight = obstacleHeight;
      this.obstacleLength = obstacleLength;
      this.obstacleWidth = obstacleWidth;

      this.obstacleReferenceFrame = obstacleReferenceFrame;
      
      this.desiredNumberOfSplines = desiredNumberOfSplines;
      this.tolerance = tolerance;
   }
   
   public void compute(double t)
   {
      concatenatedSplines.compute(t);
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

         concatenatedSplines = new ConcatenatedSplines(splineMap, referenceFrame);
         concatenatedSplines = new ConcatenatedSplines(concatenatedSplines, desiredNumberOfSplines, tolerance);
      }

      // TODO get path lengths for each path, chop up total footstep time
//    timeSpline.setCubicUsingIntermediatePoints(0, totalTime / pathLength1, totalTime / pathLength2, totalTime, z0, zIntermediate1, zIntermediate2, zFinal);
//    timeSpline.setCubic(0.0, stepTime, 0.0, initialParameterd, 1.0, finalParameterd);

   }
}
