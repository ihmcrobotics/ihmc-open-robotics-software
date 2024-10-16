package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class PoseReferenceFrame extends ReferenceFrame
{
   private final FramePose3D originPose;

   public PoseReferenceFrame(String frameName, ReferenceFrame parentFrame)
   {
      super(frameName, parentFrame, parentFrame.isAStationaryFrame(), false);

      originPose = new FramePose3D(parentFrame);
   }

   public PoseReferenceFrame(String frameName, FramePose3DReadOnly pose)
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

   public void setPoseAndUpdate(FramePose3DReadOnly pose)
   {
      originPose.set(pose);
      this.update();
   }

   public void setPoseAndUpdate(PoseReferenceFrame poseReferenceFrame)
   {
      originPose.set(poseReferenceFrame.originPose);
      this.update();
   }

   public void setPoseAndUpdate(FramePoint3DReadOnly position, FrameOrientation3DReadOnly orientation)
   {
      originPose.set(position, orientation);
      this.update();
   }

   public void setPoseAndUpdate(Point3DReadOnly position, Orientation3DReadOnly orientation)
   {
      originPose.set(position, orientation);
      this.update();
   }

   public void setPoseAndUpdate(Pose3DReadOnly pose)
   {
      originPose.set(pose);
      this.update();
   }

   public void setPoseAndUpdate(RigidBodyTransformReadOnly transformToParent)
   {
      originPose.set(transformToParent);
      this.update();
   }

   public void setPositionWithoutChecksAndUpdate(Point3DReadOnly position)
   {
      originPose.getPosition().set(position);
      this.update();
   }

   public void setPositionWithoutChecksAndUpdate(double x, double y, double z)
   {
      originPose.getPosition().set(x, y, z);
      this.update();
   }

   public void setPositionAndUpdate(FramePoint3DReadOnly framePoint)
   {
      framePoint.checkReferenceFrameMatch(getParent());
      originPose.getPosition().set(framePoint);
      this.update();
   }

   public void setOrientationAndUpdate(Orientation3DReadOnly orientation3D)
   {
      originPose.getOrientation().set(orientation3D);
      this.update();
   }

   public void setOrientationAndUpdate(double qx, double qy, double qz, double qs)
   {
      originPose.getOrientation().set(qx, qy, qz, qs);
      update();
   }

   public void setOrientationAndUpdate(FrameOrientation3DReadOnly frameOrientation)
   {
      frameOrientation.checkReferenceFrameMatch(getParent());
      originPose.getOrientation().set(frameOrientation);
      this.update();
   }
   
   public void setXYFromPosition2dAndUpdate(FramePoint2DReadOnly position2d)
   {
      position2d.checkReferenceFrameMatch(getParent());
      originPose.getPosition().set(position2d);
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

   public Pose3DReadOnly getPose()
   {
      return originPose;
   }

   public void getPose(Point3DBasics pointToPack, Orientation3DBasics quaternionToPack)
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

   public void getPoseIncludingFrame(FramePoint3DBasics framePointToPack, FrameOrientation3DBasics frameOrientationToPack)
   {
      originPose.get(framePointToPack, frameOrientationToPack);
   }

   public void getPoseIncludingFrame(FramePose3DBasics framePoseToPack)
   {
      framePoseToPack.setIncludingFrame(originPose);
   }
   
   public Point3DReadOnly getPosition()
   {
      return originPose.getPosition();
   }

   public void getPosition(Point3DBasics pointToPack)
   {
      pointToPack.set(originPose.getPosition());
   }

   public void getPositionIncludingFrame(FramePoint3DBasics framePointToPack)
   {
      framePointToPack.setIncludingFrame(originPose.getPosition());
   }
   
   public QuaternionReadOnly getOrientation()
   {
      return originPose.getOrientation();
   }

   public void getOrientation(Orientation3DBasics quaternionToPack)
   {
      quaternionToPack.set(originPose.getOrientation());
   }

   public void getOrientation(RotationMatrixBasics matrixToPack)
   {
      matrixToPack.set(originPose.getOrientation());
   }

   public void getOrientationIncludingFrame(FrameQuaternionBasics frameOrientationToPack)
   {
      frameOrientationToPack.setIncludingFrame(originPose.getOrientation());
   }

   public void getPose2dIncludingFrame(FramePose2DBasics framePose2dToPack)
   {
      framePose2dToPack.setIncludingFrame(originPose);
   }

   public void getPosition2dIncludingFrame(FramePoint2DBasics framePoint2dToPack)
   {
      framePoint2dToPack.setIncludingFrame(originPose.getPosition());
   }

   public void interpolate(FramePose3DReadOnly framePose1, FramePose3DReadOnly framePose2, double alpha)
   {
      originPose.interpolate(framePose1, framePose2, alpha);
   }

   public void interpolate(PoseReferenceFrame poseReferenceFrame1, PoseReferenceFrame poseReferenceFrame2, double alpha)
   {
      originPose.interpolate(poseReferenceFrame1.getPose(), poseReferenceFrame2.getPose(), alpha);
   }

   public boolean epsilonEquals(PoseReferenceFrame otherPoseReferenceFrame, double epsilon)
   {
      return originPose.epsilonEquals(otherPoseReferenceFrame.originPose, epsilon);
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParentToPack)
   {
      originPose.checkReferenceFrameMatch(getParent());
      originPose.get(transformToParentToPack);
   }

   @Override
   public String toString()
   {
      return super.toString() + ", originPose = " + originPose;
   }
}
