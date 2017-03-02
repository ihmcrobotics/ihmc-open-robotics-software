package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FramePose2d;

public class PoseReferenceFrame extends ReferenceFrame
{
   private static final long serialVersionUID = 1559362743221742457L;
   private final FramePose originPose;

   public PoseReferenceFrame(String frameName, ReferenceFrame parentFrame)
   {
      super(frameName, parentFrame);

      originPose = new FramePose(parentFrame);
   }

   public PoseReferenceFrame(String frameName, FramePose pose)
   {
      this(frameName, pose.getReferenceFrame());
      setPoseAndUpdate(pose);
   }

   public void setX(double x)
   {
      originPose.setX(x);
   }

   public void setY(double y)
   {
      originPose.setY(y);
   }

   public void setZ(double z)
   {
      originPose.setZ(z);
   }

   public void setPoseAndUpdate(FramePose pose)
   {
      originPose.setPose(pose);
      this.update();
   }

   public void setPoseAndUpdate(PoseReferenceFrame poseReferenceFrame)
   {
      originPose.setPose(poseReferenceFrame.originPose);
      this.update();
   }

   public void setPoseAndUpdate(FramePoint position, FrameOrientation orientation)
   {
      originPose.setPose(position, orientation);
      this.update();
   }

   public void setPoseAndUpdate(Point3D position, Quaternion orientation)
   {
      originPose.setPose(position, orientation);
      this.update();
   }

   public void setPoseAndUpdate(RigidBodyTransform transformToParent)
   {
      originPose.setPose(transformToParent);
      this.update();
   }

   public void setPositionWithoutChecksAndUpdate(Point3D position)
   {
      originPose.setPosition(position);
      this.update();
   }

   public void setPositionWithoutChecksAndUpdate(double x, double y, double z)
   {
      originPose.setPosition(x, y, z);
      this.update();
   }

   public void setPositionAndUpdate(FramePoint framePoint)
   {
      framePoint.checkReferenceFrameMatch(parentFrame);
      originPose.setPosition(framePoint);
      this.update();
   }

   public void setOrientationAndUpdate(Quaternion quat4d)
   {
      originPose.setOrientation(quat4d);
      this.update();
   }

   public void setOrientationAndUpdate(AxisAngle axisAngle4d)
   {
      originPose.setOrientation(axisAngle4d);
      this.update();
   }

   public void setOrientationAndUpdate(double qx, double qy, double qz, double qs)
   {
      originPose.setOrientation(qx, qy, qz, qs);
      update();
   }

   public void setOrientationAndUpdate(FrameOrientation frameOrientation)
   {
      frameOrientation.checkReferenceFrameMatch(parentFrame);
      originPose.setOrientation(frameOrientation);
      this.update();
   }
   
   public void setXYFromPosition2dAndUpdate(FramePoint2d position2d)
   {
      position2d.checkReferenceFrameMatch(parentFrame);
      originPose.setXYFromPosition2d(position2d);
      this.update();
   }

   public void translateAndUpdate(double x, double y, double z)
   {
      originPose.translate(x, y, z);
      this.update();
   }

   public double getX()
   {
      return originPose.getX();
   }

   public double getY()
   {
      return originPose.getY();
   }

   public double getZ()
   {
      return originPose.getZ();
   }

   public double getYaw()
   {
      return originPose.getYaw();
   }

   public double getPitch()
   {
      return originPose.getPitch();
   }

   public double getRoll()
   {
      return originPose.getRoll();
   }

   public void getPose(Point3D pointToPack, Quaternion quaternionToPack)
   {
      originPose.getPose(pointToPack, quaternionToPack);
   }

   /**
    * Same as the inherited method getTransformToParent(Transform3D) from ReferenceFrame, it is just for readability.
    * @param transformToPack
    */
   public void getPose(RigidBodyTransform transformToPack)
   {
      getTransformToParent(transformToPack);
   }

   public void getPoseIncludingFrame(FramePoint framePointToPack, FrameOrientation frameOrientationToPack)
   {
      originPose.getPoseIncludingFrame(framePointToPack, frameOrientationToPack);
   }

   public void getPoseIncludingFrame(FramePose framePoseToPack)
   {
      framePoseToPack.setPoseIncludingFrame(originPose);
   }
   
   public Point3DReadOnly getPosition()
   {
      return originPose.getPosition();
   }

   public void getPosition(Point3D pointToPack)
   {
      originPose.getPosition(pointToPack);
   }

   public void getPositionIncludingFrame(FramePoint framePointToPack)
   {
      originPose.getPositionIncludingFrame(framePointToPack);
   }
   
   public QuaternionReadOnly getOrientation()
   {
      return originPose.getOrientation();
   }

   public void getOrientation(Quaternion quaternionToPack)
   {
      originPose.getOrientation(quaternionToPack);
   }

   public void getOrientation(RotationMatrix matrixToPack)
   {
      originPose.getOrientation(matrixToPack);
   }

   public void getOrientationIncludingFrame(FrameOrientation frameOrientationToPack)
   {
      originPose.getOrientationIncludingFrame(frameOrientationToPack);
   }

   public void getPose2dIncludingFrame(FramePose2d framePose2dToPack)
   {
      originPose.getPose2dIncludingFrame(framePose2dToPack);
   }

   public void getPosition2dIncludingFrame(FramePoint2d framePoint2dToPack)
   {
      originPose.getPosition2dIncludingFrame(framePoint2dToPack);
   }

   public void interpolate(FramePose framePose1, FramePose framePose2, double alpha)
   {
      originPose.interpolate(framePose1, framePose2, alpha);
   }

   public void interpolate(PoseReferenceFrame poseReferenceFrame1, PoseReferenceFrame poseReferenceFrame2, double alpha)
   {
      originPose.interpolate(poseReferenceFrame1.originPose, poseReferenceFrame2.originPose, alpha);
   }

   public boolean epsilonEquals(PoseReferenceFrame otherPoseReferenceFrame, double epsilon)
   {
      return originPose.epsilonEquals(otherPoseReferenceFrame.originPose, epsilon);
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      originPose.checkReferenceFrameMatch(parentFrame);
      originPose.getPose(transformToParent);
   }

   @Override
   public String toString()
   {
      return super.toString() + ", originPose = " + originPose;
   }
}
