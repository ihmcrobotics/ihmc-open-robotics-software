package us.ihmc.quadrupedRobotics.footstep;

import java.io.PrintWriter;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.geometry.ReferenceFrameMismatchException;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public interface QuadrupedFootstep
{
   public abstract QuadrupedFootstep changeFrameCopy(ReferenceFrame desiredFrame);

   public abstract double distanceToFootstep(QuadrupedFootstep footstepToCheck);

   public abstract double distanceToFootstepInXY(QuadrupedFootstep footstepToCheck);

   public abstract double distanceSquaredToFootstep(QuadrupedFootstep footstepToCheck);

   public abstract RobotQuadrant getLegName();

   public abstract FramePoint getPositionFramePointCopy();

   public abstract ReferenceFrame getReferenceFrame();

   public abstract double getX();

   public abstract double getY();

   public abstract double getZ();

   public abstract boolean isEpsilonEqualTo(QuadrupedFootstep footstep, double epsilon);

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
   public abstract QuadrupedFootstep yawAboutPointCopy(FramePoint pointToYawAbout, double yaw);

   public abstract QuadrupedFootstep applyTransformCopy(RigidBodyTransform transform3D);

   public abstract QuadrupedFootstep morphCopy(QuadrupedFootstep footstep, double alpha);
}
