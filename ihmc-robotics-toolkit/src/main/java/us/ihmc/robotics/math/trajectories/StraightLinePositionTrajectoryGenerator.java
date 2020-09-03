package us.ihmc.robotics.math.trajectories;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.PositionProvider;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class StraightLinePositionTrajectoryGenerator implements PositionTrajectoryGenerator
{
   protected final YoRegistry registry;
   private final YoDouble currentTime;
   private final YoFramePoint3D currentPosition;
   private final YoFrameVector3D currentVelocity;
   private final YoFrameVector3D currentAcceleration;

   private final YoDouble trajectoryTime;
   private final YoPolynomial parameterPolynomial;
   private final YoFramePoint3D initialPosition;
   private final YoFramePoint3D finalPosition;

   private final YoFrameVector3D differenceVector;

   private final PositionProvider initialPositionProvider;
   private final PositionProvider finalPositionProvider;

   private final FramePoint3D tempInitialPosition;
   private final FramePoint3D tempFinalPosition;

   private final YoBoolean continuouslyUpdateFinalPosition;
   private final DoubleProvider trajectoryTimeProvider;

   public StraightLinePositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, DoubleProvider trajectoryTimeProvider,
         PositionProvider initialPositionProvider, PositionProvider finalPositionProvider, YoRegistry parentRegistry)
   {
      this.registry = new YoRegistry(namePrefix + getClass().getSimpleName());
      this.trajectoryTime = new YoDouble(namePrefix + "TrajectoryTime", registry);

      this.currentTime = new YoDouble(namePrefix + "CurrentTime", registry);
      this.currentPosition = new YoFramePoint3D(namePrefix + "CurrentPosition", referenceFrame, registry);
      this.currentVelocity = new YoFrameVector3D(namePrefix + "CurrentVelocity", referenceFrame, registry);
      this.currentAcceleration = new YoFrameVector3D(namePrefix + "CurrentAcceleration", referenceFrame, registry);

      this.differenceVector = new YoFrameVector3D(namePrefix + "DifferenceVector", referenceFrame, registry);

      this.parameterPolynomial = new YoPolynomial(namePrefix + "ParameterPolynomial", 6, registry);
      this.initialPosition = new YoFramePoint3D(namePrefix + "InitialPos", referenceFrame, registry);
      this.finalPosition = new YoFramePoint3D(namePrefix + "FinalPos", referenceFrame, registry);
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
      positionToPack.setIncludingFrame(currentPosition);
   }

   @Override
   public void getVelocity(FrameVector3D velocityToPack)
   {
      velocityToPack.setIncludingFrame(currentVelocity);
   }

   @Override
   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      accelerationToPack.setIncludingFrame(currentAcceleration);
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
