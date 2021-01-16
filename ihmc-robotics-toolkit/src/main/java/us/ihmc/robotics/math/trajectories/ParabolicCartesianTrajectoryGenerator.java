package us.ihmc.robotics.math.trajectories;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ParabolicCartesianTrajectoryGenerator implements FramePositionTrajectoryGenerator
{
   private final String namePostFix = getClass().getSimpleName();
   private final YoRegistry registry;
   private final YoMinimumJerkTrajectory minimumJerkTrajectory;
   private final YoParabolicTrajectoryGenerator parabolicTrajectoryGenerator;
   protected final YoDouble groundClearance;
   private final YoDouble stepTime;
   private final YoDouble timeIntoStep;
   private final DoubleProvider stepTimeProvider;
   private final FrameVector3D tempVector = new FrameVector3D(ReferenceFrame.getWorldFrame());

   private final FramePoint3D desiredPosition;
   private final FrameVector3D desiredVelocity;
   private final FrameVector3D desiredAcceleration;

   private final FramePoint3D initialDesiredPosition = new FramePoint3D();
   private final FramePoint3D finalDesiredPosition = new FramePoint3D();

   public ParabolicCartesianTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, DoubleProvider stepTimeProvider, double groundClearance,
                                                YoRegistry parentRegistry)
   {
      this.registry = new YoRegistry(namePrefix + namePostFix);
      this.minimumJerkTrajectory = new YoMinimumJerkTrajectory(namePrefix, registry);
      this.parabolicTrajectoryGenerator = new YoParabolicTrajectoryGenerator(namePrefix, referenceFrame, registry);
      this.groundClearance = new YoDouble("groundClearance", registry);
      this.stepTime = new YoDouble("stepTime", registry);
      this.timeIntoStep = new YoDouble("timeIntoStep", registry);
      parentRegistry.addChild(registry);

      desiredPosition = new FramePoint3D(referenceFrame);
      desiredVelocity = new FrameVector3D(referenceFrame);
      desiredAcceleration = new FrameVector3D(referenceFrame);

      this.stepTimeProvider = stepTimeProvider;
      this.groundClearance.set(groundClearance);
   }

   public void setInitialDesiredPosition(FramePoint3DReadOnly initialDesiredPosition)
   {
      this.initialDesiredPosition.setIncludingFrame(initialDesiredPosition);
   }

   public void setFinalDesiredPosition(FramePoint3DReadOnly finalDesiredPosition)
   {
      this.finalDesiredPosition.setIncludingFrame(finalDesiredPosition);
   }

   public ReferenceFrame getReferenceFrame()
   {
      return parabolicTrajectoryGenerator.getReferenceFrame();
   }

   public boolean isDone()
   {
      return timeIntoStep.getDoubleValue() >= stepTime.getDoubleValue();
   }

   public double getFinalTime()
   {
      return stepTime.getDoubleValue();
   }

   public void updateGroundClearance(double newGroundClearance)
   {
      this.groundClearance.set(newGroundClearance);
   }

   public double getCurrentGroundClearance()
   {
      return this.groundClearance.getDoubleValue();
   }

   @Override
   public FramePoint3DReadOnly getPosition()
   {
      double parameter = minimumJerkTrajectory.getPosition();

      parameter = MathTools.clamp(parameter, 0.0, 1.0);

      parabolicTrajectoryGenerator.getPosition(desiredPosition, parameter);

      return desiredPosition;
   }

   @Override
   public FrameVector3DReadOnly getVelocity()
   {
      double parameter = minimumJerkTrajectory.getPosition();
      parameter = MathTools.clamp(parameter, 0.0, 1.0);
      parabolicTrajectoryGenerator.getVelocity(tempVector, parameter);
      desiredVelocity.setIncludingFrame(tempVector);
      desiredVelocity.scale(minimumJerkTrajectory.getVelocity());

      return desiredVelocity;
   }

   @Override
   public FrameVector3DReadOnly getAcceleration()
   {
      double parameter = minimumJerkTrajectory.getPosition();
      parameter = MathTools.clamp(parameter, 0.0, 1.0);
      parabolicTrajectoryGenerator.getAcceleration(desiredAcceleration);
      desiredAcceleration.scale(minimumJerkTrajectory.getVelocity() * minimumJerkTrajectory.getVelocity());
      parabolicTrajectoryGenerator.getVelocity(tempVector, parameter);
      tempVector.scale(minimumJerkTrajectory.getAcceleration());
      desiredAcceleration.add(tempVector);

      return desiredAcceleration;
   }

   @Override
   public void showVisualization()
   {

   }

   @Override
   public void hideVisualization()
   {

   }

   public void computeNextTick(FramePoint3D positionToPack, double deltaT)
   {
      timeIntoStep.add(deltaT);
      compute(timeIntoStep.getDoubleValue());
      positionToPack.setIncludingFrame(getPosition());
   }

   @Override
   public void initialize()
   {
      timeIntoStep.set(0.0);
      this.stepTime.set(stepTimeProvider.getValue());

      if (stepTime.getDoubleValue() < 1e-10)
      {
         stepTime.set(1e-10);
      }

      minimumJerkTrajectory.setParams(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, stepTime.getDoubleValue());
      double middleOfTrajectoryParameter = 0.5;
      parabolicTrajectoryGenerator.initialize(initialDesiredPosition, finalDesiredPosition, groundClearance.getDoubleValue(), middleOfTrajectoryParameter);
   }

   public void compute(double time)
   {
      timeIntoStep.set(time);
      minimumJerkTrajectory.computeTrajectory(time);
   }

   public YoDouble getTimeIntoStep()
   {
      return timeIntoStep;
   }
}
