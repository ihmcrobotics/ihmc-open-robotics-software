package us.ihmc.robotics.math.trajectories;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.math.trajectories.NDoFTrapezoidalVelocityTrajectory.AlphaToAlphaType;

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

   public void computeNextTick(FramePoint3D positionToPack, FrameVector3D velocityToPack, FrameVector3D accelerationToPack, double deltaT)
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

   public void initialize(FramePoint3D initialPosition, FrameVector3D initialVelocity, FrameVector3D initialAcceleration, FramePoint3D finalDesiredPosition, FrameVector3D finalDesiredVelocity)
   {
      initialPosition = new FramePoint3D(initialPosition);
      initialPosition.changeFrame(referenceFrame);
      initialVelocity = new FrameVector3D(initialVelocity);
      initialVelocity.changeFrame(referenceFrame);
      finalDesiredPosition = new FramePoint3D(finalDesiredPosition);
      finalDesiredPosition.changeFrame(referenceFrame);

      FrameVector3D finalVelocity = new FrameVector3D(referenceFrame);

      double maxVelDouble = maxVel.getDoubleValue();
      double maxAccelDouble = maxAccel.getDoubleValue();

      FrameVector3D vMax = new FrameVector3D(referenceFrame, maxVelDouble, maxVelDouble, maxVelDouble);
      FrameVector3D aMax = new FrameVector3D(referenceFrame, maxAccelDouble, maxAccelDouble, maxAccelDouble);

      trajectory = new FramePointTrapezoidalVelocityTrajectory(time.getDoubleValue(), initialPosition, finalDesiredPosition, initialVelocity, finalVelocity,
              vMax, aMax, AlphaToAlphaType.LINEAR);
   }

   public void updateFinalDesiredPosition(FramePoint3D finalDesiredPosition)
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
