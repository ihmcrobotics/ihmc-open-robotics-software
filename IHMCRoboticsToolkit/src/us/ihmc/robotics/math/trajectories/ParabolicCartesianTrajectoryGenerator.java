package us.ihmc.robotics.math.trajectories;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;


public class ParabolicCartesianTrajectoryGenerator implements CartesianTrajectoryGenerator
{
   private final String namePostFix = getClass().getSimpleName();
   private final YoVariableRegistry registry;
   private final YoMinimumJerkTrajectory minimumJerkTrajectory;
   private final YoParabolicTrajectoryGenerator parabolicTrajectoryGenerator;
   protected final DoubleYoVariable groundClearance;
   private final DoubleYoVariable stepTime;
   private final DoubleYoVariable timeIntoStep;
   private final DoubleProvider stepTimeProvider;
   private final FrameVector tempVector = new FrameVector(ReferenceFrame.getWorldFrame());

   public ParabolicCartesianTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, DoubleProvider stepTimeProvider, double groundClearance,
           YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + namePostFix);
      this.minimumJerkTrajectory = new YoMinimumJerkTrajectory(namePrefix, registry);
      this.parabolicTrajectoryGenerator = new YoParabolicTrajectoryGenerator(namePrefix, referenceFrame, registry);
      this.groundClearance = new DoubleYoVariable("groundClearance", registry);
      this.stepTime = new DoubleYoVariable("stepTime", registry);
      this.timeIntoStep = new DoubleYoVariable("timeIntoStep", registry);
      parentRegistry.addChild(registry);

      this.stepTimeProvider = stepTimeProvider;
      this.groundClearance.set(groundClearance);
   }

   public void initialize(FramePoint initialPosition, FrameVector initialVelocity, FrameVector initialAcceleration, FramePoint finalDesiredPosition,
                          FrameVector finalDesiredVelocity)
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

   public void updateFinalDesiredPosition(FramePoint finalDesiredPosition)
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

   public void packPosition(FramePoint positionToPack)
   {
      double parameter = minimumJerkTrajectory.getPosition();

      parameter = MathTools.clipToMinMax(parameter, 0.0, 1.0);

      parabolicTrajectoryGenerator.packPosition(positionToPack, parameter);
   }

   public void packVelocity(FrameVector velocityToPack)
   {
      double parameter = minimumJerkTrajectory.getPosition();
      parameter = MathTools.clipToMinMax(parameter, 0.0, 1.0);
      parabolicTrajectoryGenerator.packVelocity(tempVector, parameter);
      velocityToPack.setIncludingFrame(tempVector);
      velocityToPack.scale(minimumJerkTrajectory.getVelocity());
   }

   public void packAcceleration(FrameVector accelerationToPack)
   {
      double parameter = minimumJerkTrajectory.getPosition();
      parameter = MathTools.clipToMinMax(parameter, 0.0, 1.0);
      parabolicTrajectoryGenerator.packAcceleration(accelerationToPack);
      accelerationToPack.scale(minimumJerkTrajectory.getVelocity() * minimumJerkTrajectory.getVelocity());
      parabolicTrajectoryGenerator.packVelocity(tempVector, parameter);
      tempVector.scale(minimumJerkTrajectory.getAcceleration());
      accelerationToPack.add(tempVector);
   }
   
   public void computeNextTick(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack, double deltaT)
   {
      timeIntoStep.add(deltaT);
      compute(timeIntoStep.getDoubleValue());
      packPosition(positionToPack);
      packVelocity(velocityToPack);
      packAcceleration(accelerationToPack);
   }
   
   public void computeNextTick(FramePoint positionToPack, double deltaT)
   {
      timeIntoStep.add(deltaT);
      compute(timeIntoStep.getDoubleValue());
      packPosition(positionToPack);
   }

   public void compute(double time)
   {
      timeIntoStep.set(time);
      minimumJerkTrajectory.computeTrajectory(time);
   }
}
