package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;

public class ParabolicCartesianTrajectoryGenerator implements CartesianTrajectoryGenerator
{
   private final String namePostFix = getClass().getSimpleName();
   private final YoVariableRegistry registry;
   private final YoMinimumJerkTrajectory minimumJerkTrajectory;
   private final YoParabolicTrajectoryGenerator parabolicTrajectoryGenerator;
   protected final YoDouble groundClearance;
   private final YoDouble stepTime;
   private final YoDouble timeIntoStep;
   private final DoubleProvider stepTimeProvider;
   private final FrameVector3D tempVector = new FrameVector3D(ReferenceFrame.getWorldFrame());

   public ParabolicCartesianTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, DoubleProvider stepTimeProvider, double groundClearance,
                                                YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + namePostFix);
      this.minimumJerkTrajectory = new YoMinimumJerkTrajectory(namePrefix, registry);
      this.parabolicTrajectoryGenerator = new YoParabolicTrajectoryGenerator(namePrefix, referenceFrame, registry);
      this.groundClearance = new YoDouble("groundClearance", registry);
      this.stepTime = new YoDouble("stepTime", registry);
      this.timeIntoStep = new YoDouble("timeIntoStep", registry);
      parentRegistry.addChild(registry);

      this.stepTimeProvider = stepTimeProvider;
      this.groundClearance.set(groundClearance);
   }

   @Override
   public void initialize(FramePoint3D initialPosition, FrameVector3D initialVelocity, FrameVector3D initialAcceleration, FramePoint3D finalDesiredPosition,
                          FrameVector3D finalDesiredVelocity)
   {
      this.initialize(initialPosition, initialVelocity, initialAcceleration, finalDesiredPosition, finalDesiredVelocity);
   }

   public void initialize(FramePoint3D initialPosition, FrameVector3DReadOnly initialVelocity, FrameVector3DReadOnly initialAcceleration,
                          FramePoint3DReadOnly finalDesiredPosition, FrameVector3D finalDesiredVelocity)
   {
      timeIntoStep.set(0.0);
      this.stepTime.set(stepTimeProvider.getValue());

      if (stepTime.getDoubleValue() < 1e-10)
      {
         stepTime.set(1e-10);
      }

      minimumJerkTrajectory.setParams(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, stepTime.getDoubleValue());
      double middleOfTrajectoryParameter = 0.5;
      parabolicTrajectoryGenerator.initialize(initialPosition, finalDesiredPosition, groundClearance.getDoubleValue(), middleOfTrajectoryParameter);

   }

   public void updateFinalDesiredPosition(FramePoint3D finalDesiredPosition)
   {
      // empty
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

   public void getPosition(FramePoint3D positionToPack)
   {
      double parameter = minimumJerkTrajectory.getPosition();

      parameter = MathTools.clamp(parameter, 0.0, 1.0);

      parabolicTrajectoryGenerator.getPosition(positionToPack, parameter);
   }

   public void getVelocity(FrameVector3D velocityToPack)
   {
      double parameter = minimumJerkTrajectory.getPosition();
      parameter = MathTools.clamp(parameter, 0.0, 1.0);
      parabolicTrajectoryGenerator.getVelocity(tempVector, parameter);
      velocityToPack.setIncludingFrame(tempVector);
      velocityToPack.scale(minimumJerkTrajectory.getVelocity());
   }

   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      double parameter = minimumJerkTrajectory.getPosition();
      parameter = MathTools.clamp(parameter, 0.0, 1.0);
      parabolicTrajectoryGenerator.getAcceleration(accelerationToPack);
      accelerationToPack.scale(minimumJerkTrajectory.getVelocity() * minimumJerkTrajectory.getVelocity());
      parabolicTrajectoryGenerator.getVelocity(tempVector, parameter);
      tempVector.scale(minimumJerkTrajectory.getAcceleration());
      accelerationToPack.add(tempVector);
   }

   public void computeNextTick(FramePoint3D positionToPack, FrameVector3D velocityToPack, FrameVector3D accelerationToPack, double deltaT)
   {
      timeIntoStep.add(deltaT);
      compute(timeIntoStep.getDoubleValue());
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   public void computeNextTick(FramePoint3D positionToPack, double deltaT)
   {
      timeIntoStep.add(deltaT);
      compute(timeIntoStep.getDoubleValue());
      getPosition(positionToPack);
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
