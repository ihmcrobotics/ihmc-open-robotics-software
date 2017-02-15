package us.ihmc.robotics.geometry;

import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Quat4d;
import javax.vecmath.Quat4f;
import javax.vecmath.Tuple3d;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.transformables.Pose;
import us.ihmc.robotics.geometry.transformables.TransformablePoint3d;
import us.ihmc.robotics.geometry.transformables.TransformableQuat4d;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FramePose extends AbstractFrameObject<FramePose, Pose>
{
   private final Pose pose;

   public FramePose()
   {
      this(new FramePoint(), new FrameOrientation());
   }

   public FramePose(ReferenceFrame referenceFrame)
   {
      this(referenceFrame, new Pose());
   }
   
   public FramePose(ReferenceFrame referenceFrame, Pose pose)
   {
      super(referenceFrame, pose);
      this.pose = getGeometryObject();
   }

   public FramePose(FramePoint position, FrameOrientation orientation)
   {
      this(position.getReferenceFrame(), new Pose(position.getGeometryObject(), orientation.getGeometryObject()));
      
      if (position.getReferenceFrame() != orientation.getReferenceFrame())
      {
         throw new ReferenceFrameMismatchException("FramePose: The position frame (" + position.getReferenceFrame()
               + ") does not match the orientation frame (" + orientation.getReferenceFrame() + ")");
      }
   }

   public FramePose(ReferenceFrame referenceFrame, Tuple3d position, Quat4d orientation)
   {
      this(referenceFrame, new Pose());
      setPose(position, orientation);
   }

   public FramePose(FramePose framePose)
   {
      this(framePose.getReferenceFrame(), new Pose(framePose.getGeometryObject()));
   }

   public FramePose(ReferenceFrame referenceFrame, RigidBodyTransform transform)
   {
      this(referenceFrame);
      setPose(transform);
   }

   public FramePose(ReferenceFrame referenceFrame, Tuple3d point3d, AxisAngle4d axisAngle4d)
   {
      this(referenceFrame);
      setPose(point3d, axisAngle4d);
   }

   public static FramePose generateRandomFramePose(Random random, ReferenceFrame referenceFrame, double maxAbsoluteX, double maxAbsoluteY, double maxAbsoluteZ)
   {
      return new FramePose(referenceFrame, RandomTools.generateRandomPoint(random, maxAbsoluteX, maxAbsoluteY, maxAbsoluteZ), RandomTools
            .generateRandomQuaternion(random));
   }

   public static FramePose generateRandomFramePose(Random random, ReferenceFrame referenceFrame, double[] xyzMin, double[] xyzMax, double[] yawPitchRollMin, double[] yawPitchRollMax)
   {
      FramePose randomFramePose = new FramePose(
            FramePoint.generateRandomFramePoint(random, referenceFrame, xyzMin[0], xyzMax[0], xyzMin[1], xyzMax[1], xyzMin[2], xyzMax[2]), FrameOrientation
            .generateRandomFrameOrientation(random, referenceFrame, yawPitchRollMin[0], yawPitchRollMax[0], yawPitchRollMin[1], yawPitchRollMax[1],
                  yawPitchRollMin[2], yawPitchRollMax[2]));
      return randomFramePose;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public void setX(double x)
   {
      pose.setX(x);
   }

   public void setY(double y)
   {
      pose.setY(y);
   }

   public void setZ(double z)
   {
      pose.setZ(z);
   }

   public void setPose(FramePose pose)
   {
      set(pose);
   }

   public void setPose(FramePoint position, FrameOrientation orientation)
   {
      setPosition(position);
      setOrientation(orientation);
   }

   public void setPose(Tuple3d position, Quat4d orientation)
   {
      pose.setPose(position, orientation);
   }

   public void setPose(Tuple3f position, Quat4f orientation)
   {
      pose.setPose(position, orientation);
   }

   private void setPose(Tuple3d position, AxisAngle4d orientation)
   {
      pose.setPose(position, orientation);
   }

   public void setPose(RigidBodyTransform transform)
   {
      pose.setPose(transform);
   }

   public void setPoseIncludingFrame(FramePose framePose)
   {
      this.pose.set(framePose.pose);
      referenceFrame = framePose.referenceFrame;
   }

   public void setPoseIncludingFrame(FramePoint position, FrameOrientation orientation)
   {
      position.checkReferenceFrameMatch(orientation);
      referenceFrame = position.referenceFrame;
      setPose(position.getPoint(), orientation.getQuaternion());
   }

   public void setPoseIncludingFrame(ReferenceFrame referenceFrame, Point3d position, Quat4d orientation)
   {
      setPose(position, orientation);
      this.referenceFrame = referenceFrame;
   }

   public void setPoseIncludingFrame(ReferenceFrame referenceFrame, Point3f position, Quat4f orientation)
   {
      setPose(position, orientation);
      this.referenceFrame = referenceFrame;
   }

   public void setPoseIncludingFrame(ReferenceFrame referenceFrame, RigidBodyTransform transform)
   {
      pose.setPose(transform);
      this.referenceFrame = referenceFrame;
   }

   public void setPosition(FramePoint framePoint)
   {
      checkReferenceFrameMatch(framePoint);
      pose.setPosition(framePoint.getPoint());
   }

   public void setPosition(Tuple3d position)
   {
      pose.setPosition(position);
   }
   
   public void setPosition(double x, double y, double z)
   {
      this.pose.setPosition(x, y, z);
   }

   public void setOrientation(double qx, double qy, double qz, double qs)
   {
      this.pose.setOrientation(qx, qy, qz, qs);
   }

   public void setOrientation(Quat4d orientation)
   {
      this.pose.setOrientation(orientation);
   }

   public void setOrientation(Matrix3d orientation)
   {
      this.pose.setOrientation(orientation);
   }

   public void setOrientation(AxisAngle4d orientation)
   {
      this.pose.setOrientation(orientation);
   }

   public void setYawPitchRoll(double[] yawPitchRoll)
   {
      this.pose.setYawPitchRoll(yawPitchRoll);
   }

   public void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      this.pose.setYawPitchRoll(yaw, pitch, roll);
   }

   public void setOrientation(FrameOrientation frameOrientation)
   {
      checkReferenceFrameMatch(frameOrientation);
      pose.setOrientation(frameOrientation.getQuaternion());
   }

   public void setXYFromPosition2d(FramePoint2d position2d)
   {
      pose.setXY(position2d.getPoint());
   }

   @Override
   public void setToZero(ReferenceFrame referenceFrame)
   {
      super.setToZero();
      this.referenceFrame = referenceFrame;
   }

   public void getPose(Point3d pointToPack, Quat4d quaternionToPack)
   {
      getPosition(pointToPack);
      getOrientation(quaternionToPack);
   }
   
   public void getPose(Vector3d vector3dToPack, Quat4d quaternionToPack)
   {
      getPosition(vector3dToPack);
      getOrientation(quaternionToPack);
   }
   
   public void getPose(Vector3d vector3dToPack, AxisAngle4d axisAngleToPack)
   {
      getPosition(vector3dToPack);
      getOrientation(axisAngleToPack);
   }

   public void getPose(RigidBodyTransform transformToPack)
   {
      pose.getPose(transformToPack);
   }

   public void getPoseIncludingFrame(FramePoint framePointToPack, FrameOrientation orientationToPack)
   {
      getPositionIncludingFrame(framePointToPack);
      getOrientationIncludingFrame(orientationToPack);
   }
   
   public TransformablePoint3d getPositionUnsafe()
   {
      return pose.getPositionUnsafe();
   }

   public void getPosition(Tuple3d tupleToPack)
   {
      pose.getPosition(tupleToPack);
   }

   public void getPositionIncludingFrame(FrameTuple<?, ?> frameTupleToPack)
   {
      frameTupleToPack.setIncludingFrame(referenceFrame, pose.getPoint());
   }

   public void getRigidBodyTransform(RigidBodyTransform transformToPack)
   {
      pose.getRigidBodyTransform(transformToPack);
   }
   
   public TransformableQuat4d getOrientationUnsafe()
   {
      return pose.getOrientationUnsafe();
   }
   
   public void getOrientation(Matrix3d matrixToPack)
   {
      pose.getOrientation(matrixToPack);
   }

   public void getOrientation(Quat4d quaternionToPack)
   {
      pose.getOrientation(quaternionToPack);
   }

   public void getOrientation(double[] yawPitchRoll)
   {
      pose.getYawPitchRoll(yawPitchRoll);
   }
   
   public void getOrientation(AxisAngle4d axisAngleToPack)
   {
      pose.getOrientation(axisAngleToPack);
   }

   public void getOrientationIncludingFrame(FrameOrientation orientationToPack)
   {
      orientationToPack.setIncludingFrame(referenceFrame, pose.getOrientation());
   }

   public void getPose2dIncludingFrame(FramePose2d framePose2dToPack)
   {
      framePose2dToPack.setPoseIncludingFrame(referenceFrame, getX(), getY(), getYaw());
   }

   public void getPosition2dIncludingFrame(FramePoint2d framePoint2dToPack)
   {
      framePoint2dToPack.setIncludingFrame(referenceFrame, pose.getPoint().getX(), pose.getPoint().getY());
   }

   public void getOrientation2dIncludingFrame(FrameOrientation2d frameOrientation2dToPack)
   {
      frameOrientation2dToPack.setIncludingFrame(referenceFrame, pose.getYaw());
   }
   
   public void rotatePoseAboutAxis(FrameVector rotatationAxis, FramePoint rotationAxisOrigin, double angle)
   {
      ReferenceFrame frameWhoseZAxisIsRotationAxis = ReferenceFrame.constructReferenceFrameFromPointAndZAxis("rotationAxisFrame", rotationAxisOrigin,
            rotatationAxis);

      rotatePoseAboutAxis(frameWhoseZAxisIsRotationAxis, Axis.Z, angle);
   }
   
   public FramePose getRotatedAboutAxisCopy(ReferenceFrame rotationAxisFrame, Axis rotationAxis, double angle)
   {
      FramePose ret = new FramePose(this);
      ret.rotatePoseAboutAxis(rotationAxisFrame, rotationAxis, angle);
      return ret;
   }

   public void rotatePoseAboutAxis(ReferenceFrame rotationAxisFrame, Axis rotationAxis, double angle)
   {
      rotatePoseAboutAxis(rotationAxisFrame, rotationAxis, angle, false, false);
   }
   
   public void rotatePoseAboutAxis(ReferenceFrame rotationAxisFrame, Axis rotationAxis, double angle, boolean lockPosition, boolean lockOrientation)
   {
      ReferenceFrame initialFrame = this.referenceFrame;

      this.changeFrame(rotationAxisFrame);

      RigidBodyTransform axisRotationTransform = new RigidBodyTransform();
      TransformTools.rotate(axisRotationTransform, angle, rotationAxis);
      
      if (!lockPosition)
      {
         Vector3d tempVector = new Vector3d();
         this.getPosition(tempVector);
         axisRotationTransform.transform(tempVector);
         this.setPosition(tempVector);
      }
      
      if(!lockOrientation)
         this.pose.getOrientation().applyTransform(axisRotationTransform);

      this.changeFrame(initialFrame);
   }
   
   public void translate(Tuple3d translation)
   {
      pose.translate(translation);
   }

   public void translate(double x, double y, double z)
   {
      pose.translate(x, y, z);
   }

   public double getX()
   {
      return pose.getX();
   }

   public double getY()
   {
      return pose.getY();
   }

   public double getZ()
   {
      return pose.getZ();
   }

   public double getYaw()
   {
      return pose.getYaw();
   }

   public double getPitch()
   {
      return pose.getPitch();
   }

   public double getRoll()
   {
      return pose.getRoll();
   }

   public void interpolate(FramePose framePose1, FramePose framePose2, double alpha)
   {
      checkReferenceFrameMatch(framePose1);
      framePose1.checkReferenceFrameMatch(framePose2);
      this.pose.interpolate(framePose1.pose, framePose2.pose, alpha);
   }

//   @Override
//   public void changeFrame(ReferenceFrame desiredFrame)
//   {
//      // this is in the correct frame already
//      if (desiredFrame == referenceFrame)
//      {
//         return;
//      }
//
//      position.changeFrame(desiredFrame);
//      orientation.changeFrame(desiredFrame);
//      referenceFrame = desiredFrame;
//   }
//   
//   @Override
//   public void changeFrameUsingTransform(ReferenceFrame desiredFrame, RigidBodyTransform transformToNewFrame)
//   {
//      position.changeFrameUsingTransform(desiredFrame, transformToNewFrame);
//      orientation.changeFrameUsingTransform(desiredFrame, transformToNewFrame);
//      
//      referenceFrame = desiredFrame;
//   }
//
//   @Override
//   public void applyTransform(RigidBodyTransform transform)
//   {
//      position.applyTransform(transform);
//      orientation.applyTransform(transform);
//   }

//   public boolean epsilonEquals(FramePose framePose, double epsilon)
//   {
//      if (!position.epsilonEquals(framePose.position, epsilon))
//         return false;
//
//      if (!orientation.epsilonEquals(framePose.orientation, epsilon))
//         return false;
//
//      return true;
//   }
//   
//   public boolean epsilonEquals(FramePose framePose, double positionEpsilon, double orientationEpsilon)
//   {
//      if (!position.epsilonEquals(framePose.position, positionEpsilon))
//         return false;
//      
//      if (!orientation.epsilonEquals(framePose.orientation, orientationEpsilon))
//         return false;
//      
//      return true;
//   }

   public String printOutPosition()
   {
      return pose.printOutPosition();
   }

   public String printOutOrientation()
   {
      return pose.printOutOrientation();
   }
   
   public FramePoint getFramePointCopy()
   {
      FramePoint ret = new FramePoint(getReferenceFrame(), pose.getPoint());
      return ret;
   }
   
   public FrameOrientation getFrameOrientationCopy()
   {
      FrameOrientation ret = new FrameOrientation(getReferenceFrame(), pose.getOrientation());
      return ret;
   }

   public FrameVector getTranslationToOtherPoseTotal(FramePose otherPose)
   {
      checkReferenceFrameMatch(otherPose);

      FrameVector ret = new FrameVector(referenceFrame);
      ret.sub(otherPose.pose.getPoint(), this.pose.getPoint());

      return ret;
   }

   public FrameVector getTranslationNOTDueToRotationAboutFrame(FramePose otherPose)
   {
      checkReferenceFrameMatch(otherPose);

      RigidBodyTransform transformToOtherPose = getTransformFromThisToThat(otherPose);
      FrameVector ret = new FrameVector(referenceFrame);
      transformToOtherPose.getTranslation(ret.tuple);

      return ret;
   }

   public FrameVector getTranslationDueToRotationAboutFrame(FramePose otherPose)
   {
      checkReferenceFrameMatch(otherPose);

      FrameVector ret = getTranslationToOtherPoseTotal(otherPose);
      ret.sub(getTranslationNOTDueToRotationAboutFrame(otherPose));
      return ret;
   }

   public double getPositionDistance(FramePose framePose)
   {
      checkReferenceFrameMatch(framePose);
      
      return this.pose.getPoint().distance(framePose.pose.getPoint());
   }
   
   public RigidBodyTransform getTransformFromThisToThat(FramePose thatPose)
   {
      checkReferenceFrameMatch(thatPose);

      RigidBodyTransform transformToThis = new RigidBodyTransform();
      this.getPose(transformToThis);

      RigidBodyTransform transformToThat = new RigidBodyTransform();
      thatPose.getPose(transformToThat);

      return TransformTools.getTransformFromA2toA1(transformToThat, transformToThis);
   }

   public double getSpatialAxisOfRotationAndAngleToOtherPose(FramePose otherPose, FrameVector rotationAxisToPack, FramePoint rotationAxisOriginToPack)
   {
      double rotationAngle = getAxisAngleRotationToOtherPose(otherPose, rotationAxisToPack);

      getOriginOfSpatialAxisOfRotationToOtherPose(otherPose, rotationAxisToPack, rotationAngle, rotationAxisOriginToPack);

      return rotationAngle;
   }

   private void getOriginOfSpatialAxisOfRotationToOtherPose(FramePose otherPose, FrameVector rotationAxis, double rotationAngle, FramePoint originToPack)
   {
      otherPose.checkReferenceFrameMatch(rotationAxis);
      otherPose.checkReferenceFrameMatch(originToPack);
      
      double epsilon = 1e-7;
      boolean rotationAngleEpsilonEqualsZero = Math.abs(rotationAngle) < epsilon;
      boolean rotationAngleEpsilonEqualsPlusOrMinusPi = Math.abs(Math.abs(rotationAngle) - Math.PI) < epsilon;

      if (rotationAngleEpsilonEqualsZero)
      {
         originToPack.setToZero(referenceFrame);
      }
      else if (rotationAngleEpsilonEqualsPlusOrMinusPi)
      {
         originToPack.setToZero(referenceFrame);
         originToPack.interpolate(this.pose.getPoint(), otherPose.pose.getPoint(), 0.5);
      }
      else
      {
         GeometryTools.getTopVertexOfIsoscelesTriangle(pose.getPoint(), otherPose.pose.getPoint(), rotationAxis.getVector(), rotationAngle, originToPack.getPoint());
      }
   }

   public double getAxisAngleRotationToOtherPose(FramePose otherPose, FrameVector rotationAxisToPack)
   {
      AxisAngle4d rotationAxisAngle = new AxisAngle4d();
      getAxisAngleRotationToOtherPose(otherPose, rotationAxisAngle);

      rotationAxisToPack.setIncludingFrame(this.referenceFrame, rotationAxisAngle.getX(), rotationAxisAngle.getY(), rotationAxisAngle.getZ());

      return rotationAxisAngle.getAngle();
   }

   public void getAxisAngleRotationToOtherPose(FramePose otherPose, AxisAngle4d rotationToPack)
   {
      getTransformFromThisToThat(otherPose).getRotation(rotationToPack); //FIXME: this returns nonsense when angle = Math.PI
   }

   public double getOrientationDistance(FramePose framePose)
   {
      AxisAngle4d rotationFromThatToThis = new AxisAngle4d();
      getAxisAngleRotationToOtherPose(framePose, rotationFromThatToThis);

      return Math.abs(rotationFromThatToThis.getAngle());
   }
   
   public double getEffectiveDistanceToFramePose(FramePose framePose, double radiusOfRotation)
   {
      checkReferenceFrameMatch(framePose);
      
      RigidBodyTransform transformThis = new RigidBodyTransform();
      this.getPose(transformThis);
      transformThis.invert();
      
      
      RigidBodyTransform transformThat = new RigidBodyTransform();
      framePose.getPose(transformThat);
      transformThat.invert();
      
      return TransformTools.getSizeOfTransformBetweenTwoWithRotationScaled(transformThis, transformThat, radiusOfRotation);
   }
   

   @Override
   public String toString()
   {
      return "Position: " + pose.getPoint().toString() + "\n" + pose.getOrientation().toStringAsYawPitchRoll() + "  -- " + referenceFrame.getName();
   }

   public boolean epsilonEquals(FramePose other, double positionErrorMargin, double orientationErrorMargin)
   {
      return pose.epsilonEquals(other.pose, positionErrorMargin, orientationErrorMargin);
   }

   @Override
   public boolean epsilonEquals(FramePose other, double epsilon)
   {
      return pose.epsilonEquals(other.pose, epsilon);
   }
   
}
