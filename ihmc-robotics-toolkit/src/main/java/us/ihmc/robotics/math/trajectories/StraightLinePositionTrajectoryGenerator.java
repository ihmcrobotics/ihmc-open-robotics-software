package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.PositionProvider;

public class StraightLinePositionTrajectoryGenerator implements PositionTrajectoryGenerator
{
   protected final YoVariableRegistry registry;
   private final YoDouble currentTime;
   private final YoFramePoint currentPosition;
   private final YoFrameVector currentVelocity;
   private final YoFrameVector currentAcceleration;

   private final YoDouble trajectoryTime;
   private final YoPolynomial parameterPolynomial;
   private final YoFramePoint initialPosition;
   private final YoFramePoint finalPosition;

   private final YoFrameVector differenceVector;

   private final PositionProvider initialPositionProvider;
   private final PositionProvider finalPositionProvider;

   private final FramePoint3D tempInitialPosition;
   private final FramePoint3D tempFinalPosition;

   private final YoBoolean continuouslyUpdateFinalPosition;
   private final DoubleProvider trajectoryTimeProvider;

   public StraightLinePositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, DoubleProvider trajectoryTimeProvider,
         PositionProvider initialPositionProvider, PositionProvider finalPositionProvider, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.trajectoryTime = new YoDouble(namePrefix + "TrajectoryTime", registry);

      this.currentTime = new YoDouble(namePrefix + "CurrentTime", registry);
      this.currentPosition = new YoFramePoint(namePrefix + "CurrentPosition", referenceFrame, registry);
      this.currentVelocity = new YoFrameVector(namePrefix + "CurrentVelocity", referenceFrame, registry);
      this.currentAcceleration = new YoFrameVector(namePrefix + "CurrentAcceleration", referenceFrame, registry);

      this.differenceVector = new YoFrameVector(namePrefix + "DifferenceVector", referenceFrame, registry);

      this.parameterPolynomial = new YoPolynomial(namePrefix + "ParameterPolynomial", 6, registry);
      this.initialPosition = new YoFramePoint(namePrefix + "InitialPos", referenceFrame, registry);
      this.finalPosition = new YoFramePoint(namePrefix + "FinalPos", referenceFrame, registry);
      this.continuouslyUpdateFinalPosition = new YoBoolean(namePrefix + "ContinuouslyUpdate", registry);

      this.initialPositionProvider = initialPositionProvider;
      this.finalPositionProvider = finalPositionProvider;

      tempInitialPosition = new FramePoint3D(referenceFrame);
      tempFinalPosition = new FramePoint3D(referenceFrame);

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
   public void getPosition(FramePoint3D positionToPack)
   {
      currentPosition.getFrameTupleIncludingFrame(positionToPack);
   }

   @Override
   public void getVelocity(FrameVector3D velocityToPack)
   {
      currentVelocity.getFrameTupleIncludingFrame(velocityToPack);
   }

   @Override
   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      currentAcceleration.getFrameTupleIncludingFrame(accelerationToPack);
   }

   public void setContinuouslyUpdateFinalPosition(boolean val)
   {
      continuouslyUpdateFinalPosition.set(val);
   }

   @Override
   public void getLinearData(FramePoint3D positionToPack, FrameVector3D velocityToPack, FrameVector3D accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   @Override
   public void showVisualization()
   {
   }

   @Override
   public void hideVisualization()
   {
   }
}
