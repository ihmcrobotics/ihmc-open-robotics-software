package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.PositionProvider;

public class ParabolicPositionTrajectoryGenerator implements PositionTrajectoryGenerator
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
   private final PositionProvider initialPositionProvider;
   private final PositionProvider finalPositionProvider;

   public ParabolicPositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, DoubleProvider stepTimeProvider,
         PositionProvider initialPositionProvider, PositionProvider finalPositionProvider, double groundClearance, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + namePostFix);
      this.minimumJerkTrajectory = new YoMinimumJerkTrajectory(namePrefix, registry);
      this.parabolicTrajectoryGenerator = new YoParabolicTrajectoryGenerator(namePrefix, referenceFrame, registry);
      this.groundClearance = new YoDouble("groundClearance", registry);
      this.stepTime = new YoDouble("stepTime", registry);
      this.timeIntoStep = new YoDouble("timeIntoStep", registry);
      parentRegistry.addChild(registry);

      this.stepTimeProvider = stepTimeProvider;
      this.initialPositionProvider = initialPositionProvider;
      this.finalPositionProvider = finalPositionProvider;
      this.groundClearance.set(groundClearance);
   }

   public void computeNextTick(FramePoint3D positionToPack, FrameVector3D velocityToPack, FrameVector3D accelerationToPack, double deltaT)
   {
      timeIntoStep.add(deltaT);
      compute(timeIntoStep.getDoubleValue());
      getLinearData(positionToPack, velocityToPack, accelerationToPack);
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

      FramePoint3D initialPosition = new FramePoint3D(ReferenceFrame.getWorldFrame());
      initialPositionProvider.getPosition(initialPosition);

      FramePoint3D finalPosition = new FramePoint3D(ReferenceFrame.getWorldFrame());
      finalPositionProvider.getPosition(finalPosition);

      initialPosition.changeFrame(parabolicTrajectoryGenerator.getReferenceFrame());
      finalPosition.changeFrame(parabolicTrajectoryGenerator.getReferenceFrame());
      double maxAnkleHeight = Math.max(initialPosition.getZ(), finalPosition.getZ());
      double apexHeight = maxAnkleHeight + groundClearance.getDoubleValue();

      parabolicTrajectoryGenerator.initialize(initialPosition, finalPosition, apexHeight, middleOfTrajectoryParameter);
   }

   public void compute(double time)
   {
      timeIntoStep.set(time);
      minimumJerkTrajectory.computeTrajectory(time);
   }

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
