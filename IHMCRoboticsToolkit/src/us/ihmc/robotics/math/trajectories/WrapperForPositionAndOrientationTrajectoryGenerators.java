package us.ihmc.robotics.math.trajectories;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;


public class WrapperForPositionAndOrientationTrajectoryGenerators implements PoseTrajectoryGenerator
{
   private final PositionTrajectoryGenerator positionTrajectoryGenerator;
   private final OrientationTrajectoryGenerator orientationTrajectoryGenerator;
   
   public WrapperForPositionAndOrientationTrajectoryGenerators(PositionTrajectoryGenerator positionTrajectoryGenerator, OrientationTrajectoryGenerator orientationTrajectoryGenerator)
   {
      this.positionTrajectoryGenerator = positionTrajectoryGenerator;
      this.orientationTrajectoryGenerator = orientationTrajectoryGenerator;
   }

   public void initialize()
   {
      positionTrajectoryGenerator.initialize();
      orientationTrajectoryGenerator.initialize();
   }

   public void compute(double time)
   {
      positionTrajectoryGenerator.compute(time);
      orientationTrajectoryGenerator.compute(time);
   }

   public boolean isDone()
   {
      return positionTrajectoryGenerator.isDone() && orientationTrajectoryGenerator.isDone();
   }

   public void get(FramePoint positionToPack)
   {
      positionTrajectoryGenerator.get(positionToPack);
   }

   public void packVelocity(FrameVector velocityToPack)
   {
      positionTrajectoryGenerator.packVelocity(velocityToPack);
   }

   public void packAcceleration(FrameVector accelerationToPack)
   {
      positionTrajectoryGenerator.packAcceleration(accelerationToPack);
   }

   public void packLinearData(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack)
   {
      positionTrajectoryGenerator.packLinearData(positionToPack, velocityToPack, accelerationToPack);
   }

   public void get(FrameOrientation orientationToPack)
   {
      orientationTrajectoryGenerator.get(orientationToPack);
   }
   
   public void packAngularVelocity(FrameVector angularVelocityToPack)
   {
      orientationTrajectoryGenerator.packAngularVelocity(angularVelocityToPack);
   }

   public void packAngularAcceleration(FrameVector angularAccelerationToPack)
   {
      orientationTrajectoryGenerator.packAngularAcceleration(angularAccelerationToPack);
   }

   public void packAngularData(FrameOrientation orientationToPack, FrameVector angularVelocityToPack, FrameVector angularAccelerationToPack)
   {
      orientationTrajectoryGenerator.packAngularData(orientationToPack, angularVelocityToPack, angularAccelerationToPack);
   }
   
   private final FramePoint tempPosition = new FramePoint();
   private final FrameOrientation tempOrientation = new FrameOrientation();
   
   public void get(FramePose framePoseToPack)
   {
      positionTrajectoryGenerator.get(tempPosition);
      framePoseToPack.changeFrame(tempPosition.getReferenceFrame());
      framePoseToPack.setPosition(tempPosition);

      orientationTrajectoryGenerator.get(tempOrientation);
      framePoseToPack.setOrientation(tempOrientation);
   }

   public void showVisualization()
   {
      positionTrajectoryGenerator.showVisualization();
   }

   public void hideVisualization()
   {
      positionTrajectoryGenerator.hideVisualization();
   }
}
