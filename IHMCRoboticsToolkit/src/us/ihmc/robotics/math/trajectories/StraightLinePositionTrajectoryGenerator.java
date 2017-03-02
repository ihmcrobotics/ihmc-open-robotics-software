package us.ihmc.robotics.math.trajectories;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.PositionProvider;

public class StraightLinePositionTrajectoryGenerator implements PositionTrajectoryGenerator
{
   protected final YoVariableRegistry registry;
   private final DoubleYoVariable currentTime;
   private final YoFramePoint currentPosition;
   private final YoFrameVector currentVelocity;
   private final YoFrameVector currentAcceleration;

   private final DoubleYoVariable trajectoryTime;
   private final YoPolynomial parameterPolynomial;
   private final YoFramePoint initialPosition;
   private final YoFramePoint finalPosition;

   private final YoFrameVector differenceVector;

   private final PositionProvider initialPositionProvider;
   private final PositionProvider finalPositionProvider;

   private final FramePoint tempInitialPosition;
   private final FramePoint tempFinalPosition;

   private final BooleanYoVariable continuouslyUpdateFinalPosition;
   private final DoubleProvider trajectoryTimeProvider;

   public StraightLinePositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, DoubleProvider trajectoryTimeProvider,
         PositionProvider initialPositionProvider, PositionProvider finalPositionProvider, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.trajectoryTime = new DoubleYoVariable(namePrefix + "TrajectoryTime", registry);

      this.currentTime = new DoubleYoVariable(namePrefix + "CurrentTime", registry);
      this.currentPosition = new YoFramePoint(namePrefix + "CurrentPosition", referenceFrame, registry);
      this.currentVelocity = new YoFrameVector(namePrefix + "CurrentVelocity", referenceFrame, registry);
      this.currentAcceleration = new YoFrameVector(namePrefix + "CurrentAcceleration", referenceFrame, registry);

      this.differenceVector = new YoFrameVector(namePrefix + "DifferenceVector", referenceFrame, registry);

      this.parameterPolynomial = new YoPolynomial(namePrefix + "ParameterPolynomial", 6, registry);
      this.initialPosition = new YoFramePoint(namePrefix + "InitialPos", referenceFrame, registry);
      this.finalPosition = new YoFramePoint(namePrefix + "FinalPos", referenceFrame, registry);
      this.continuouslyUpdateFinalPosition = new BooleanYoVariable(namePrefix + "ContinuouslyUpdate", registry);

      this.initialPositionProvider = initialPositionProvider;
      this.finalPositionProvider = finalPositionProvider;

      tempInitialPosition = new FramePoint(referenceFrame);
      tempFinalPosition = new FramePoint(referenceFrame);

      this.trajectoryTimeProvider = trajectoryTimeProvider;
      parentRegistry.addChild(registry);
   }

   @Override
   public void initialize()
   {
      currentTime.set(0.0);
      trajectoryTime.set(trajectoryTimeProvider.getValue());
      parameterPolynomial.setQuintic(0.0, trajectoryTime.getDoubleValue(), 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);

      updateInitialPosition();
      updateFinalPosition();
   }

   private void updateInitialPosition()
   {
      initialPositionProvider.getPosition(tempInitialPosition);
      tempInitialPosition.changeFrame(initialPosition.getReferenceFrame());
      initialPosition.set(tempInitialPosition);
   }

   private void updateFinalPosition()
   {
      finalPositionProvider.getPosition(tempFinalPosition);
      tempFinalPosition.changeFrame(finalPosition.getReferenceFrame());
      finalPosition.set(tempFinalPosition);
   }

   @Override
   public void compute(double time)
   {
      if (continuouslyUpdateFinalPosition.getBooleanValue())
      {
         updateFinalPosition();
      }

      this.currentTime.set(time);
      time = MathTools.clamp(time, 0.0, trajectoryTime.getDoubleValue());
      parameterPolynomial.compute(time);
      differenceVector.sub(finalPosition, initialPosition);

      double parameter = isDone() ? 1.0 : parameterPolynomial.getPosition();
      double parameterd = isDone() ? 0.0 : parameterPolynomial.getVelocity();
      double parameterdd = isDone() ? 0.0 : parameterPolynomial.getAcceleration();

      currentPosition.interpolate(initialPosition, finalPosition, parameter);
      currentVelocity.set(differenceVector);
      currentVelocity.scale(parameterd);
      currentAcceleration.set(differenceVector);
      currentAcceleration.scale(parameterdd);
   }

   @Override
   public boolean isDone()
   {
      return currentTime.getDoubleValue() >= trajectoryTime.getDoubleValue();
   }

   @Override
   public void getPosition(FramePoint positionToPack)
   {
      currentPosition.getFrameTupleIncludingFrame(positionToPack);
   }

   @Override
   public void getVelocity(FrameVector velocityToPack)
   {
      currentVelocity.getFrameTupleIncludingFrame(velocityToPack);
   }

   @Override
   public void getAcceleration(FrameVector accelerationToPack)
   {
      currentAcceleration.getFrameTupleIncludingFrame(accelerationToPack);
   }

   public void setContinuouslyUpdateFinalPosition(boolean val)
   {
      continuouslyUpdateFinalPosition.set(val);
   }

   @Override
   public void getLinearData(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
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
