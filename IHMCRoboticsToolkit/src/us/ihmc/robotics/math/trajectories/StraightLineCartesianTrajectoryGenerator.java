package us.ihmc.robotics.math.trajectories;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.NDoFTrapezoidalVelocityTrajectory.AlphaToAlphaType;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class StraightLineCartesianTrajectoryGenerator implements CartesianTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   private final ReferenceFrame referenceFrame;
   private final YoDouble time;
   private final YoDouble maxVel;
   private final YoDouble maxAccel;

   private FramePointTrapezoidalVelocityTrajectory trajectory;


   public StraightLineCartesianTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, double maxVel, double maxAccel, YoDouble time,
           YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + "StraightLineCartesianTrajectoryGenerator");
      this.maxVel =  new YoDouble("straightLineTrajMaxVel", registry);
      this.maxAccel = new YoDouble("straightLineTrajMaxAccel", registry);
      
      this.referenceFrame = referenceFrame;
      this.time = time;

      this.maxVel.set(maxVel);
      this.maxAccel.set(maxAccel);
      parentRegistry.addChild(registry);
   }

   public void computeNextTick(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack, double deltaT)
   {
      double time = this.time.getDoubleValue();

      positionToPack.set(trajectory.getPosition(time));
      velocityToPack.set(trajectory.getVelocity(time));
      accelerationToPack.set(trajectory.getAcceleration(time));
   }

   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public void initialize(FramePoint initialPosition, FrameVector initialVelocity, FrameVector initialAcceleration, FramePoint finalDesiredPosition, FrameVector finalDesiredVelocity)
   {
      initialPosition = new FramePoint(initialPosition);
      initialPosition.changeFrame(referenceFrame);
      initialVelocity = new FrameVector(initialVelocity);
      initialVelocity.changeFrame(referenceFrame);
      finalDesiredPosition = new FramePoint(finalDesiredPosition);
      finalDesiredPosition.changeFrame(referenceFrame);

      FrameVector finalVelocity = new FrameVector(referenceFrame);

      double maxVelDouble = maxVel.getDoubleValue();
      double maxAccelDouble = maxAccel.getDoubleValue();

      FrameVector vMax = new FrameVector(referenceFrame, maxVelDouble, maxVelDouble, maxVelDouble);
      FrameVector aMax = new FrameVector(referenceFrame, maxAccelDouble, maxAccelDouble, maxAccelDouble);

      trajectory = new FramePointTrapezoidalVelocityTrajectory(time.getDoubleValue(), initialPosition, finalDesiredPosition, initialVelocity, finalVelocity,
              vMax, aMax, AlphaToAlphaType.LINEAR);
   }

   public void updateFinalDesiredPosition(FramePoint finalDesiredPosition)
   {
      // Just ignore on straight line...
   }

   public boolean isDone()
   {
      if(trajectory == null)
         return true;
      
      return (time.getDoubleValue() > trajectory.getTFMax());
   }

   public double getFinalTime()
   {
      return trajectory.getTFMax();
   }
   
   public void setMaxVelocity(double velocity)
   {
      maxVel.set(velocity);
   }
}
