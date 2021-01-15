package us.ihmc.robotics.math.trajectories;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
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

   private final ReferenceFrame referenceFrame;
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

   private final YoBoolean continuouslyUpdateFinalPosition;
   private final DoubleProvider trajectoryTimeProvider;

   public StraightLinePositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, DoubleProvider trajectoryTimeProvider,
         PositionProvider initialPositionProvider, PositionProvider finalPositionProvider, YoRegistry parentRegistry)
   {
      this.referenceFrame = referenceFrame;
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
      initialPosition.setMatchingFrame(initialPositionProvider.getPosition());
   }

   private void updateFinalPosition()
   {
      finalPosition.setMatchingFrame(finalPositionProvider.getPosition());
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
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public FramePoint3DReadOnly getPosition()
   {
      return currentPosition;
   }

   @Override
   public FrameVector3DReadOnly getVelocity()
   {
      return currentVelocity;
   }

   @Override
   public FrameVector3DReadOnly getAcceleration()
   {
      return currentAcceleration;
   }

   public void setContinuouslyUpdateFinalPosition(boolean val)
   {
      continuouslyUpdateFinalPosition.set(val);
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
