package us.ihmc.aware.planning;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.MinimumJerkTrajectory;

public class ThreeDoFSwingFootTrajectory
{
   final private MinimumJerkTrajectory xTrajectory;
   final private MinimumJerkTrajectory yTrajectory;
   final private ParabolicTrajectory zTrajectory;
   final private FramePoint position;
   final private FrameVector velocity;
   final private FrameVector acceleration;
   private ReferenceFrame referenceFrame;
   private boolean moveInitialized;

   public ThreeDoFSwingFootTrajectory()
   {
      xTrajectory = new MinimumJerkTrajectory();
      yTrajectory = new MinimumJerkTrajectory();
      zTrajectory = new ParabolicTrajectory();
      position = new FramePoint();
      velocity = new FrameVector();
      acceleration = new FrameVector();
      referenceFrame = ReferenceFrame.getWorldFrame();
      moveInitialized = false;
   }

   public void getPosition(FramePoint position)
   {
      position.setIncludingFrame(this.position);
   }

   public void getVelocity(FrameVector velocity)
   {
      velocity.setIncludingFrame(this.velocity);
   }

   public void getAcceleration(FrameVector acceleration)
   {
      acceleration.setIncludingFrame(this.acceleration);
   }

   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public void setMoveParameters(FramePoint initialPosition, FramePoint finalPosition, double groundClearance, double swingDuration)
   {
      initialPosition.checkReferenceFrameMatch(finalPosition);
      referenceFrame = initialPosition.getReferenceFrame();

      double midwayPositionZ = groundClearance + Math.max(initialPosition.getZ(), finalPosition.getZ());
      xTrajectory.setMoveParameters(initialPosition.getX(), 0, 0, finalPosition.getX(), 0, 0, swingDuration);
      yTrajectory.setMoveParameters(initialPosition.getY(), 0, 0, finalPosition.getY(), 0, 0, swingDuration);
      zTrajectory.setMoveParameters(initialPosition.getZ(), midwayPositionZ, finalPosition.getZ(), swingDuration);
      moveInitialized = true;

      position.changeFrame(referenceFrame);
      velocity.changeFrame(referenceFrame);
      acceleration.changeFrame(referenceFrame);
      computeTrajectory(0);
   }

   public void computeTrajectory(double timeInMove)
   {
      if (!moveInitialized)
         throw new RuntimeException("move parameters must be initialized before computing trajectory");

      xTrajectory.computeTrajectory(timeInMove);
      yTrajectory.computeTrajectory(timeInMove);
      zTrajectory.computeTrajectory(timeInMove);

      position.setX(xTrajectory.getPosition());
      position.setY(yTrajectory.getPosition());
      position.setZ(zTrajectory.getPosition());

      velocity.setX(xTrajectory.getVelocity());
      velocity.setY(yTrajectory.getVelocity());
      velocity.setZ(zTrajectory.getVelocity());

      acceleration.setX(xTrajectory.getAcceleration());
      acceleration.setY(yTrajectory.getAcceleration());
      acceleration.setZ(zTrajectory.getAcceleration());
   }
}