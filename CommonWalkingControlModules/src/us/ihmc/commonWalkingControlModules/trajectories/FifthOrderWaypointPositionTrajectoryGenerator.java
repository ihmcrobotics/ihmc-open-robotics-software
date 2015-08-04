package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.EnumMap;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.PositionProvider;
import us.ihmc.robotics.trajectories.providers.VectorProvider;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;
import us.ihmc.yoUtilities.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.YoPolynomial;


public class FifthOrderWaypointPositionTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private final String namePostFix = getClass().getSimpleName();
   private final YoVariableRegistry registry;
   private final EnumMap<Direction, YoPolynomial> spaceSplines = new EnumMap<Direction, YoPolynomial>(Direction.class);
   private final YoPolynomial timeSpline;

   private final DoubleProvider stepTimeProvider;
   private final PositionProvider initialPositionSource;
   private final VectorProvider initialVelocitySource;
   private final VectorProvider initialAccelerationSource;
   private final PositionProvider finalDesiredPositionSource;
   private final DoubleProvider finalPositionZOffsetProvider;
   private final VectorProvider finalDesiredVelocitySource;

   private final DoubleYoVariable waypointHeight;
   private final DoubleYoVariable stepTime;
   private final DoubleYoVariable timeIntoStep;

   private final YoFramePoint desiredPosition;
   private final YoFrameVector desiredVelocity;
   private final YoFrameVector desiredAcceleration;

   private final ReferenceFrame referenceFrame;
   private final FrameVector tempVector = new FrameVector(ReferenceFrame.getWorldFrame());

   public FifthOrderWaypointPositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, DoubleProvider stepTimeProvider,
           PositionProvider initialPositionProvider, VectorProvider initalVelocityProvider, VectorProvider initialAccelerationProvider,
           PositionProvider finalPositionProvider, DoubleProvider finalPositionZOffsetProvider, VectorProvider finalDesiredVelocityProvider, double groundClearance, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + namePostFix);

      for (Direction direction : Direction.values())
      {
         spaceSplines.put(direction, new YoPolynomial(namePrefix + direction, 5, registry));
      }

      timeSpline = new YoPolynomial(namePrefix + "Time", 4, registry);
      this.stepTimeProvider = stepTimeProvider;

      this.initialPositionSource = initialPositionProvider;
      this.initialVelocitySource = initalVelocityProvider;
      this.initialAccelerationSource = initialAccelerationProvider;
      this.finalDesiredPositionSource = finalPositionProvider;
      this.finalPositionZOffsetProvider = finalPositionZOffsetProvider;
      this.finalDesiredVelocitySource = finalDesiredVelocityProvider;

      this.waypointHeight = new DoubleYoVariable("waypointHeight", registry);
      this.stepTime = new DoubleYoVariable("stepTime", registry);
      this.timeIntoStep = new DoubleYoVariable("timeIntoStep", registry);

      this.desiredPosition = new YoFramePoint("desiredPosition", referenceFrame, registry);
      this.desiredVelocity = new YoFrameVector("desiredVelocity", referenceFrame, registry);
      this.desiredAcceleration = new YoFrameVector("desiredAcceleration", referenceFrame, registry);

      this.referenceFrame = referenceFrame;
      parentRegistry.addChild(registry);

      this.waypointHeight.set(groundClearance);
   }

   public void compute(double time)
   {
      tempVector.setToZero(referenceFrame);

      timeIntoStep.set(time);

      double totalTime = this.stepTime.getDoubleValue();
      if (time > totalTime)
         time = totalTime;

      timeSpline.compute(time);
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
         YoPolynomial spaceSpline = spaceSplines.get(direction);
         spaceSpline.compute(parameter);
         desiredPosition.set(direction, spaceSplines.get(direction).getPosition());
         tempVector.set(direction, spaceSplines.get(direction).getVelocity());
         desiredAcceleration.set(direction, spaceSplines.get(direction).getAcceleration());
      }

      desiredVelocity.set(tempVector);
      desiredVelocity.scale(parameterd);

      desiredAcceleration.scale(parameterd * parameterd);
      tempVector.scale(parameterdd);
      desiredAcceleration.add(tempVector);

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
      return timeIntoStep.getDoubleValue() >= stepTime.getDoubleValue();
   }

   public double getFinalTime()
   {
      return stepTime.getDoubleValue();
   }

   public void get(FramePoint positionToPack)
   {
      desiredPosition.getFrameTupleIncludingFrame(positionToPack);
   }

   public void packVelocity(FrameVector velocityToPack)
   {
      desiredVelocity.getFrameTupleIncludingFrame(velocityToPack);
   }

   public void packAcceleration(FrameVector accelerationToPack)
   {
      desiredAcceleration.getFrameTupleIncludingFrame(accelerationToPack);
   }

   public void initialize()
   {
      double stepTime = stepTimeProvider.getValue();
      MathTools.checkIfInRange(stepTime, 0.0, Double.POSITIVE_INFINITY);
      this.stepTime.set(stepTime);

      FramePoint initialPosition = new FramePoint(referenceFrame);
      FrameVector initialVelocity = new FrameVector(referenceFrame);
      FrameVector initialAcceleration = new FrameVector(referenceFrame);
      FramePoint finalDesiredPosition = new FramePoint(referenceFrame);
      FrameVector finalDesiredVelocity = new FrameVector(referenceFrame);

      initialPositionSource.get(initialPosition);
      initialVelocitySource.get(initialVelocity);
      initialAccelerationSource.get(initialAcceleration);
      finalDesiredPositionSource.get(finalDesiredPosition);
      finalDesiredVelocitySource.get(finalDesiredVelocity);

      initialPosition.changeFrame(referenceFrame);
      initialVelocity.changeFrame(referenceFrame);
      initialAcceleration.changeFrame(referenceFrame);
      finalDesiredPosition.changeFrame(referenceFrame);
      finalDesiredVelocity.changeFrame(referenceFrame);
      
      FramePoint intermediatePosition = new FramePoint(initialPosition);

//    FramePoint intermediatePosition = new FramePoint(referenceFrame);
//    intermediatePosition.interpolate(initialPosition, finalDesiredPosition, 0.1);
      intermediatePosition.setZ(Math.max(initialPosition.getZ(), finalDesiredPosition.getZ()) + waypointHeight.getDoubleValue());
      
      finalDesiredPosition.setZ(finalDesiredPosition.getZ() + finalPositionZOffsetProvider.getValue());

      timeIntoStep.set(0.0);

      FrameVector initialDirection = new FrameVector(initialVelocity);
      FrameVector initialDirectionChange = new FrameVector(initialAcceleration);
      FrameVector finalDirection = new FrameVector(finalDesiredVelocity);

      if ((initialDirection.length() == 0.0) && (initialDirectionChange.length() != 0.0))
         initialDirection.set(initialDirectionChange);

      for (Direction direction : Direction.values())
      {
         double t0 = 0.0;
         double tIntermediate = 0.4;    // 0.6; // TODO
         double tFinal = 1.0;
         double z0 = initialPosition.get(direction);
         double zd0 = initialDirection.get(direction);
         double zIntermediate = intermediatePosition.get(direction);
         double zf = finalDesiredPosition.get(direction);
         double zdf = finalDirection.get(direction);
         YoPolynomial spaceSpline = spaceSplines.get(direction);
         spaceSpline.setQuarticUsingWayPoint(t0, tIntermediate, tFinal, z0, zd0, zIntermediate, zf, zdf);
      }

      double initialParameterd;
      if (initialVelocity.length() == 0.0)
         initialParameterd = 0.0;
      else
         initialParameterd = initialVelocity.length() / initialDirection.length();

//    double initialParameterdd;
//    if (initialDirection.length() == 0.0)
//       initialParameterdd = 0.0;
//    else
//       initialParameterdd = (initialAcceleration.length() - initialDirectionChange.length() * MathTools.square(initialParameterd))
//             / initialDirection.length();

      double finalParameterd;
      if (finalDesiredVelocity.length() == 0.0)
         finalParameterd = 0.0;
      else
         finalParameterd = finalDesiredVelocity.length() / finalDirection.length();
      timeSpline.setCubic(0.0, stepTime, 0.0, initialParameterd, 1.0, finalParameterd);
   }

   public void packLinearData(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack)
   {
      get(positionToPack);
      packVelocity(velocityToPack);
      packAcceleration(accelerationToPack);
   }

   @Override
   public void showVisualization()
   {
      // TODO Auto-generated method stub
   }

   @Override
   public void hideVisualization()
   {
      // TODO Auto-generated method stub
   }
}
