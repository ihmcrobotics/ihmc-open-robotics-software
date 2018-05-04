package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class PoseReferenceFrame extends ReferenceFrame
{
   private final FramePose3D originPose;

   public PoseReferenceFrame(String frameName, ReferenceFrame parentFrame)
   {
      super(frameName, parentFrame, parentFrame.isAStationaryFrame(), false);

      originPose = new FramePose3D(parentFrame);
   }

   public PoseReferenceFrame(String frameName, FramePose3D pose)
   {
      this(frameName, pose.getReferenceFrame());
      setPoseAndUpdate(pose);
   }

   public boolean containsNaN()
   {
      return originPose.containsNaN();
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

   public void setPoseAndUpdate(FramePose3D pose)
   {
      originPose.set(pose);
      this.update();
   }

   public void setPoseAndUpdate(PoseReferenceFrame poseReferenceFrame)
   {
      originPose.set(poseReferenceFrame.originPose);
      this.update();
   }

   public void setPoseAndUpdate(FramePoint3DReadOnly position, FrameQuaternionReadOnly orientation)
   {
      originPose.set(position, orientation);
      this.update();
   }

   public void setPoseAndUpdate(Point3DReadOnly position, QuaternionReadOnly orientation)
   {
      originPose.set(position, orientation);
      this.update();
   }

   public void setPoseAndUpdate(Pose3D pose)
   {
      originPose.set(pose);
      this.update();
   }

   public void setPoseAndUpdate(RigidBodyTransform transformToParent)
   {
      originPose.set(transformToParent);
      this.update();
   }

   public void setPositionWithoutChecksAndUpdate(Point3DReadOnly position)
   {
      originPose.setPosition(position);
      this.update();
   }

   public void setPositionWithoutChecksAndUpdate(double x, double y, double z)
   {
      originPose.setPosition(x, y, z);
      this.update();
   }

   public void setPositionAndUpdate(FramePoint3DReadOnly framePoint)
   {
      framePoint.checkReferenceFrameMatch(parentFrame);
      originPose.setPosition(framePoint);
      this.update();
   }

   public void setOrientationAndUpdate(QuaternionReadOnly quat4d)
   {
      originPose.setOrientation(quat4d);
      this.update();
   }

   public void setOrientationAndUpdate(AxisAngleReadOnly axisAngle4d)
   {
      originPose.setOrientation(axisAngle4d);
      this.update();
   }

   public void setOrientationAndUpdate(double qx, double qy, double qz, double qs)
   {
      originPose.setOrientation(qx, qy, qz, qs);
      update();
   }

   public void setOrientationAndUpdate(FrameQuaternionReadOnly frameOrientation)
   {
      frameOrientation.checkReferenceFrameMatch(parentFrame);
      originPose.setOrientation(frameOrientation);
      this.update();
   }
   
   public void setXYFromPosition2dAndUpdate(FramePoint2DReadOnly position2d)
   {
      position2d.checkReferenceFrameMatch(parentFrame);
      originPose.setPosition(position2d);
      this.update();
   }

   public void translateAndUpdate(double x, double y, double z)
   {
      originPose.prependTranslation(x, y, z);
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
      originPose.get(pointToPack, quaternionToPack);
   }

   /**
    * Same as the inherited method getTransformToParent(Transform3D) from ReferenceFrame, it is just for readability.
    * @param transformToPack
    */
   public void getPose(RigidBodyTransform transformToPack)
   {
      getTransformToParent(transformToPack);
   }

   public void getPoseIncludingFrame(FramePoint3D framePointToPack, FrameQuaternion frameOrientationToPack)
   {
      originPose.get(framePointToPack, frameOrientationToPack);
   }

   public void getPoseIncludingFrame(FramePose3D framePoseToPack)
   {
      framePoseToPack.setIncludingFrame(originPose);
   }
   
   public Point3DReadOnly getPosition()
   {
      return originPose.getPosition();
   }

   public void getPosition(Point3D pointToPack)
   {
      pointToPack.set(originPose.getPosition());
   }

   public void getPositionIncludingFrame(FramePoint3D framePointToPack)
   {
      framePointToPack.setIncludingFrame(originPose.getPosition());
   }
   
   public QuaternionReadOnly getOrientation()
   {
      return originPose.getOrientation();
   }

   public void getOrientation(Quaternion quaternionToPack)
   {
      quaternionToPack.set(originPose.getOrientation());
   }

   public void getOrientation(RotationMatrix matrixToPack)
   {
      matrixToPack.set(originPose.getOrientation());
   }

   public void getOrientationIncludingFrame(FrameQuaternion frameOrientationToPack)
   {
      frameOrientationToPack.setIncludingFrame(originPose.getOrientation());
   }

   public void getPose2dIncludingFrame(FramePose2D framePose2dToPack)
   {
      framePose2dToPack.setIncludingFrame(originPose);
   }

   public void getPosition2dIncludingFrame(FramePoint2D framePoint2dToPack)
   {
      framePoint2dToPack.setIncludingFrame(originPose.getPosition());
   }

   public void interpolate(FramePose3D framePose1, FramePose3D framePose2, double alpha)
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
      originPose.get(transformToParent);
   }

   @Override
   public String toString()
   {
      return super.toString() + ", originPose = " + originPose;
   }
}
