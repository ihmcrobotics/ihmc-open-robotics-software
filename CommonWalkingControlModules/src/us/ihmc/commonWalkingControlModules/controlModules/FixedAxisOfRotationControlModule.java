package us.ihmc.commonWalkingControlModules.controlModules;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.Orientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Twist;

public class FixedAxisOfRotationControlModule
{
   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame baseFrame;

   private final FramePose desiredPose;
   private final Twist desiredTwist = new Twist();
   private final SpatialAccelerationVector feedForward = new SpatialAccelerationVector();

   public FixedAxisOfRotationControlModule(ReferenceFrame bodyFrame, ReferenceFrame baseFrame)
   {
      this.bodyFrame = bodyFrame;
      this.baseFrame = baseFrame;
      this.desiredPose = new FramePose(baseFrame);
   }

   public void compute(double q, double qd, double qdd, Vector3d axisOfRotation, Vector3d offset)
   {
      Vector3d axisOfRotationDot = new Vector3d();
      Vector3d offsetDot = new Vector3d();

      Vector3d positionVector3d = new Vector3d();
      positionVector3d.cross(offset, axisOfRotation);
      positionVector3d.scale(q);
      desiredPose.setPosition(new FramePoint(desiredPose.getReferenceFrame(), positionVector3d));

      AxisAngle4d axisAngle = new AxisAngle4d(axisOfRotation, q);
      Quat4d quaternion = new Quat4d();
      quaternion.set(axisAngle);
      Orientation orientation = new Orientation(desiredPose.getReferenceFrame(), quaternion);
      desiredPose.setOrientation(orientation);

      desiredTwist.setScrew(bodyFrame, baseFrame, bodyFrame, qd, 0.0, axisOfRotation, offset);
      feedForward.setScrew(bodyFrame, baseFrame, bodyFrame, qd, qdd, 0.0, 0.0, axisOfRotation, axisOfRotationDot, offset, offsetDot);
   }

   public void pack(FramePose desiredPose, Twist desiredTwist, SpatialAccelerationVector feedForward)
   {
      desiredPose.set(this.desiredPose);
      desiredTwist.set(this.desiredTwist);
      feedForward.set(this.feedForward);
   }
}
