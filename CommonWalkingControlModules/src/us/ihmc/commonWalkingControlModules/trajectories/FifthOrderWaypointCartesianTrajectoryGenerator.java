package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.EnumMap;

import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.Direction;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.trajectory.PolynomialSpline;

public class FifthOrderWaypointCartesianTrajectoryGenerator implements CartesianTrajectoryGenerator
{
   private final String namePostFix = getClass().getSimpleName();
   private final YoVariableRegistry registry;
   private final EnumMap<Direction, PolynomialSpline> spaceSplines = new EnumMap<Direction, PolynomialSpline>(Direction.class);
   private final PolynomialSpline timeSpline;

   private final DoubleYoVariable waypointHeight;
   private final DoubleYoVariable totalTime;
   private final DoubleYoVariable timeIntoStep;

   private final ReferenceFrame referenceFrame;
   private final FrameVector tempVector = new FrameVector(ReferenceFrame.getWorldFrame());

   public FifthOrderWaypointCartesianTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, double totalTime, double groundClearance,
         YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + namePostFix);

      for (Direction direction : Direction.values())
      {
         spaceSplines.put(direction, new PolynomialSpline(namePrefix + direction, 6, registry));
      }
      timeSpline = new PolynomialSpline(namePrefix + "Time", 6, registry);

      this.waypointHeight = new DoubleYoVariable("waypointHeight", registry);
      this.totalTime = new DoubleYoVariable("stepTime", registry);
      this.timeIntoStep = new DoubleYoVariable("timeIntoStep", registry);
      this.referenceFrame = referenceFrame;
      parentRegistry.addChild(registry);

      MathTools.checkIfInRange(totalTime, 0.0, Double.POSITIVE_INFINITY);

      this.totalTime.set(totalTime);
      this.waypointHeight.set(groundClearance);
   }

   public void initialize(FramePoint initialPosition, FrameVector initialVelocity, FrameVector initialAcceleration, FramePoint finalDesiredPosition,
         FrameVector finalDesiredVelocity)
   {
      initialPosition.checkReferenceFrameMatch(referenceFrame);
      initialVelocity.checkReferenceFrameMatch(referenceFrame);
      initialAcceleration.checkReferenceFrameMatch(referenceFrame);
      finalDesiredPosition.checkReferenceFrameMatch(referenceFrame);
      finalDesiredVelocity.checkReferenceFrameMatch(referenceFrame);

      FramePoint intermediatePosition = new FramePoint(initialPosition);
      //      intermediatePosition.add(finalDesiredPosition);
      //      intermediatePosition.scale(0.5);
      intermediatePosition.setZ(Math.max(initialPosition.getZ(), finalDesiredPosition.getZ()) + waypointHeight.getDoubleValue());

      timeIntoStep.set(0.0);

      FrameVector initialDirection = new FrameVector(initialVelocity);
      FrameVector initialDirectionChange = new FrameVector(initialAcceleration);
      FrameVector finalDirection = new FrameVector(finalDesiredVelocity);

      if (initialDirection.length() == 0.0 && initialDirectionChange.length() != 0.0)
         initialDirection.set(initialDirectionChange);

      for (Direction direction : Direction.values())
      {
         double t0 = 0.0;
         double tIntermediate = 0.5; // TODO
         double tFinal = 1.0;
         double z0 = initialPosition.get(direction);
         double zd0 = initialDirection.get(direction);
         double zdd0 = initialDirectionChange.get(direction);
         double zIntermediate = intermediatePosition.get(direction);
         double zf = finalDesiredPosition.get(direction);
         double zdf = finalDirection.get(direction);
         PolynomialSpline spaceSpline = spaceSplines.get(direction);
         spaceSpline.setQuinticUsingWayPoint(t0, tIntermediate, tFinal, z0, zd0, zdd0, zIntermediate, zf, zdf);
      }

      double initialParameterd;
      if (initialVelocity.length() == 0.0)
         initialParameterd = 0.0;
      else
         initialParameterd = initialVelocity.length() / initialDirection.length();

      double initialParameterdd;
      if (initialDirection.length() == 0.0)
         initialParameterdd = 0.0;
      else
         initialParameterdd = (initialAcceleration.length() - initialDirectionChange.length() * MathTools.square(initialParameterd))
               / initialDirection.length();

      double finalParameterd;
      if (finalDesiredVelocity.length() == 0.0)
         finalParameterd = 0.0;
      else
         finalParameterd = finalDesiredVelocity.length() / finalDirection.length();
      timeSpline.setQuintic(0.0, totalTime.getDoubleValue(), 0.0, initialParameterd, initialParameterdd, 1.0, finalParameterd, 0.0);
   }

   public void computeNextTick(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack, double deltaT)
   {
      positionToPack.setToZero(referenceFrame);
      velocityToPack.setToZero(referenceFrame);
      accelerationToPack.setToZero(referenceFrame);
      tempVector.setToZero(referenceFrame);

      double totalTime = this.totalTime.getDoubleValue();
      if (timeIntoStep.getDoubleValue() > totalTime)
         timeIntoStep.set(totalTime);

      timeSpline.compute(timeIntoStep.getDoubleValue());
      double parameter = timeSpline.getPosition();
      double parameterd = timeSpline.getVelocity();
      double parameterdd = timeSpline.getAcceleration();

      if (parameter < 0.0)
      {
         parameter = 0.0;
         parameterd = 0.0;
         parameterdd = 0.0;
      }

      if (parameter > 1.0)
      {
         parameter = 1.0;
         parameterd = 0.0;
         parameterdd = 0.0;
      }

      for (Direction direction : Direction.values())
      {
         PolynomialSpline spaceSpline = spaceSplines.get(direction);
         spaceSpline.compute(parameter);
         positionToPack.set(direction, spaceSplines.get(direction).getPosition());
         tempVector.set(direction, spaceSplines.get(direction).getVelocity());
         accelerationToPack.set(direction, spaceSplines.get(direction).getAcceleration());
      }

      velocityToPack.setAndChangeFrame(tempVector);
      velocityToPack.scale(parameterd);

      accelerationToPack.scale(parameterd * parameterd);
      tempVector.scale(parameterdd);
      accelerationToPack.add(tempVector);

      timeIntoStep.add(deltaT);
   }

   public void updateFinalDesiredPosition(FramePoint finalDesiredPosition)
   {
      // empty
   }

   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public boolean isDone()
   {
      return timeIntoStep.getDoubleValue() >= totalTime.getDoubleValue();
   }

   public double getFinalTime()
   {
      return totalTime.getDoubleValue();
   }
}
