package us.ihmc.robotics.quadruped;

import java.io.PrintWriter;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.geometry.ReferenceFrameMismatchException;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public interface Footstep
{
   public abstract Footstep changeFrameCopy(ReferenceFrame desiredFrame);

   public abstract double distanceToFootstep(Footstep footstepToCheck);

   public abstract double distanceToFootstepInXY(Footstep footstepToCheck);

   public abstract double distanceSquaredToFootstep(Footstep footstepToCheck);

   public abstract RobotQuadrant getLegName();

   public abstract FramePoint getPositionFramePointCopy();

   public abstract ReferenceFrame getReferenceFrame();

   public abstract double getX();

   public abstract double getY();

   public abstract double getZ();

   public abstract boolean isEpsilonEqualTo(Footstep footstep, double epsilon);

   public abstract void save(PrintWriter printWriter);

// public FootstepInterface morph(FootstepInterface startFootstep, FootstepInterface endFootstep, double alpha);

   public void checkReferenceFrameMatch(ReferenceFrame frame) throws ReferenceFrameMismatchException;

   /**
    * yaws the Footstep about the point.
    *
    * @param pointToYawAbout FramePoint to yaw about
    * @param yaw Amount to yaw.
    * @return FootstepInterface
    */
   public abstract Footstep yawAboutPointCopy(FramePoint pointToYawAbout, double yaw);

   public abstract Footstep applyTransformCopy(RigidBodyTransform transform3D);

   public abstract Footstep morphCopy(Footstep footstep, double alpha);
}
