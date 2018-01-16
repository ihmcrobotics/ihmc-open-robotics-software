package us.ihmc.robotics.geometry;

import java.util.Random;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.FrameGeometryObject;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameTuple3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.random.RandomGeometry;

public class FramePose extends FrameGeometryObject<FramePose, Pose3D>
{
   private final Pose3D pose;

   public FramePose()
   {
      this(new FramePoint3D(), new FrameQuaternion());
   }

   public FramePose(ReferenceFrame referenceFrame)
   {
      this(referenceFrame, new Pose3D());
   }

   public FramePose(ReferenceFrame referenceFrame, Pose3D pose)
   {
      super(referenceFrame, pose);
      this.pose = getGeometryObject();
   }

   public FramePose(FramePoint3D position, FrameQuaternion orientation)
   {
      this(position.getReferenceFrame(), new Pose3D(position, orientation));

      if (position.getReferenceFrame() != orientation.getReferenceFrame())
      {
         throw new ReferenceFrameMismatchException("FramePose: The position frame (" + position.getReferenceFrame() + ") does not match the orientation frame ("
               + orientation.getReferenceFrame() + ")");
      }
   }

   public FramePose(ReferenceFrame referenceFrame, Tuple3DReadOnly position, QuaternionReadOnly orientation)
   {
      this(referenceFrame, new Pose3D());
      setPose(position, orientation);
   }

   public FramePose(FramePose framePose)
   {
      this(framePose.getReferenceFrame(), new Pose3D(framePose.getGeometryObject()));
   }

   public FramePose(ReferenceFrame referenceFrame, RigidBodyTransform transform)
   {
      this(referenceFrame);
      setPose(transform);
   }

   public FramePose(ReferenceFrame referenceFrame, Tuple3DReadOnly point3d, AxisAngleReadOnly axisAngle4d)
   {
      this(referenceFrame);
      setPose(point3d, axisAngle4d);
   }

   public static FramePose generateRandomFramePose(Random random, ReferenceFrame referenceFrame, double maxAbsoluteX, double maxAbsoluteY, double maxAbsoluteZ)
   {
      return new FramePose(referenceFrame, RandomGeometry.nextPoint3D(random, maxAbsoluteX, maxAbsoluteY, maxAbsoluteZ), RandomGeometry.nextQuaternion(random));
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

   public void setPose(FramePoint3D position, FrameQuaternion orientation)
   {
      setPosition(position);
      setOrientation(orientation);
   }

   public void setPose(Tuple3DReadOnly position, QuaternionReadOnly orientation)
   {
      pose.set(position, orientation);
   }

   private void setPose(Tuple3DReadOnly position, AxisAngleReadOnly orientation)
   {
      pose.set(position, orientation);
   }

   public void setPose(RigidBodyTransform transform)
   {
      pose.set(transform);
   }

   public void setPoseIncludingFrame(FramePose framePose)
   {
      pose.set(framePose.pose);
      referenceFrame = framePose.referenceFrame;
   }

   public void setPoseIncludingFrame(FramePoint3D position, FrameQuaternion orientation)
   {
      position.checkReferenceFrameMatch(orientation);
      referenceFrame = position.getReferenceFrame();
      setPose(position.getPoint(), orientation.getQuaternion());
   }

   public void setPoseIncludingFrame(ReferenceFrame referenceFrame, Point3DReadOnly position, QuaternionReadOnly orientation)
   {
      setPose(position, orientation);
      this.referenceFrame = referenceFrame;
   }

   public void setPoseIncludingFrame(ReferenceFrame referenceFrame, RigidBodyTransform transform)
   {
      pose.set(transform);
      this.referenceFrame = referenceFrame;
   }

   public void setPosition(FramePoint3D framePoint)
   {
      checkReferenceFrameMatch(framePoint);
      pose.setPosition(framePoint.getPoint());
   }

   public void setPosition(Tuple3DReadOnly position)
   {
      pose.setPosition(position);
   }

   public void setPosition(double x, double y, double z)
   {
      pose.setPosition(x, y, z);
   }

   public void setOrientation(double qx, double qy, double qz, double qs)
   {
      pose.setOrientation(qx, qy, qz, qs);
   }

   public void setOrientation(QuaternionReadOnly orientation)
   {
      pose.setOrientation(orientation);
   }

   public void setOrientation(RotationMatrixReadOnly orientation)
   {
      pose.setOrientation(orientation);
   }

   public void setOrientation(AxisAngleReadOnly orientation)
   {
      pose.setOrientation(orientation);
   }

   public void setYawPitchRoll(double[] yawPitchRoll)
   {
      pose.setOrientationYawPitchRoll(yawPitchRoll);
   }

   public void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      pose.setOrientationYawPitchRoll(yaw, pitch, roll);
   }

   public void setOrientation(FrameQuaternion frameOrientation)
   {
      checkReferenceFrameMatch(frameOrientation);
      pose.setOrientation(frameOrientation.getQuaternion());
   }

   public void setXYFromPosition2d(FramePoint2D position2d)
   {
      pose.setPositionXY(position2d.getPoint());
   }

   public void getPose(Tuple3DBasics tupleToPack, QuaternionBasics quaternionToPack)
   {
      getPosition(tupleToPack);
      getOrientation(quaternionToPack);
   }

   public void getPose(Tuple3DBasics tupleToPack, AxisAngleBasics axisAngleToPack)
   {
      getPosition(tupleToPack);
      getOrientation(axisAngleToPack);
   }

   public void getPose(RigidBodyTransform transformToPack)
   {
      pose.get(transformToPack);
   }

   public void getPoseIncludingFrame(FramePoint3D framePointToPack, FrameQuaternion orientationToPack)
   {
      getPositionIncludingFrame(framePointToPack);
      getOrientationIncludingFrame(orientationToPack);
   }

   public Point3DReadOnly getPosition()
   {
      return pose.getPosition();
   }

   public void getPosition(Tuple3DBasics tupleToPack)
   {
      pose.getPosition(tupleToPack);
   }

   public void getPositionIncludingFrame(FrameTuple3D<?, ?> frameTupleToPack)
   {
      frameTupleToPack.setIncludingFrame(referenceFrame, pose.getPosition());
   }

   public void getRigidBodyTransform(RigidBodyTransform transformToPack)
   {
      pose.get(transformToPack);
   }

   public QuaternionReadOnly getOrientation()
   {
      return pose.getOrientation();
   }

   public void getOrientation(RotationMatrix matrixToPack)
   {
      pose.getOrientation(matrixToPack);
   }

   public void getOrientation(QuaternionBasics quaternionToPack)
   {
      pose.getOrientation(quaternionToPack);
   }

   public void getOrientation(double[] yawPitchRoll)
   {
      pose.getOrientationYawPitchRoll(yawPitchRoll);
   }

   public void getOrientation(AxisAngleBasics axisAngleToPack)
   {
      pose.getOrientation(axisAngleToPack);
   }

   public void getOrientationIncludingFrame(FrameQuaternion orientationToPack)
   {
      orientationToPack.setIncludingFrame(referenceFrame, pose.getOrientation());
   }

   /**
    * Computes and packs the orientation described by the quaternion part of this pose as a rotation
    * vector.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param rotationVectorToPack the vector in which the rotation vector is stored. Modified.
    */
   public void getRotationVector(Vector3DBasics rotationVectorToPack)
   {
      pose.getRotationVector(rotationVectorToPack);
   }

   /**
    * Computes and packs the orientation described by the quaternion part of this pose as a rotation
    * vector. The reference frame of the argument is changed to {@code this.referenceFrame}.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param frameRotationVectorToPack the vector in which the rotation vector and the reference
    *           frame of this pose are stored. Modified.
    */
   public void getRotationVectorIncludingFrame(FrameVector3D frameRotationVectorToPack)
   {
      frameRotationVectorToPack.setToZero(getReferenceFrame());
      pose.getRotationVector(frameRotationVectorToPack.getVector());
   }

   public void getPose2dIncludingFrame(FramePose2d framePose2dToPack)
   {
      framePose2dToPack.setIncludingFrame(referenceFrame, getX(), getY(), getYaw());
   }

   public void getPosition2dIncludingFrame(FramePoint2D framePoint2dToPack)
   {
      framePoint2dToPack.setIncludingFrame(referenceFrame, pose.getPosition().getX(), pose.getPosition().getY());
   }

   public void getOrientation2dIncludingFrame(FrameOrientation2d frameOrientation2dToPack)
   {
      frameOrientation2dToPack.setIncludingFrame(referenceFrame, pose.getYaw());
   }

   public void rotatePoseAboutAxis(FrameVector3D rotatationAxis, FramePoint3D rotationAxisOrigin, double angle)
   {
      ReferenceFrame frameWhoseZAxisIsRotationAxis = GeometryTools.constructReferenceFrameFromPointAndZAxis("rotationAxisFrame", rotationAxisOrigin,
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
      ReferenceFrame initialFrame = referenceFrame;

      changeFrame(rotationAxisFrame);

      AxisAngle axisAngle = new AxisAngle(0.0, 0.0, 0.0, angle);
      axisAngle.setElement(rotationAxis.ordinal(), 1.0);

      if (!lockPosition)
      {
         Point3D newPosition = new Point3D(pose.getPosition());
         axisAngle.transform(newPosition);
         setPosition(newPosition);
      }

      if (!lockOrientation)
      {
         Quaternion newOrientation = new Quaternion(pose.getOrientation());
         axisAngle.transform(newOrientation);
         setOrientation(newOrientation);
      }

      changeFrame(initialFrame);
   }

   /**
    * Normalizes the quaternion part of this pose to ensure it is a unit-quaternion describing a
    * proper orientation.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the quaternion contains {@link Double#NaN}, this method is ineffective.
    * </ul>
    * </p>
    */
   public void normalizeQuaternion()
   {
      pose.normalizeQuaternion();
   }

   /**
    * Normalizes the quaternion part of this pose and then limits the angle of the rotation it
    * represents to be &in; [-<i>pi</i>;<i>pi</i>].
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the quaternion contains {@link Double#NaN}, this method is ineffective.
    * </ul>
    * </p>
    */
   public void normalizeQuaternionAndLimitToPi()
   {
      pose.normalizeQuaternionAndLimitToPi();
   }

   public void prependTranslation(double x, double y, double z)
   {
      pose.prependTranslation(x, y, z);
   }

   public void prependTranslation(Tuple3DReadOnly translation)
   {
      pose.prependTranslation(translation);
   }

   public void prependRotation(QuaternionReadOnly rotation)
   {
      pose.prependRotation(rotation);
   }

   public void prependRotation(RotationMatrixReadOnly rotation)
   {
      pose.prependRotation(rotation);
   }

   public void prependRotation(AxisAngleReadOnly rotation)
   {
      pose.prependRotation(rotation);
   }

   public void prependYawRotation(double yaw)
   {
      pose.prependYawRotation(yaw);
   }

   public void prependPitchRotation(double pitch)
   {
      pose.prependPitchRotation(pitch);
   }

   public void prependRollRotation(double roll)
   {
      pose.prependRollRotation(roll);
   }

   public void appendTranslation(double x, double y, double z)
   {
      pose.appendTranslation(x, y, z);
   }

   public void appendTranslation(Tuple3DReadOnly translation)
   {
      pose.appendTranslation(translation);
   }

   public void appendRotation(QuaternionReadOnly rotation)
   {
      pose.appendRotation(rotation);
   }

   public void appendRotation(RotationMatrixReadOnly rotation)
   {
      pose.appendRotation(rotation);
   }

   public void appendYawRotation(double yaw)
   {
      pose.appendYawRotation(yaw);
   }

   public void appendPitchRotation(double pitch)
   {
      pose.appendPitchRotation(pitch);
   }

   public void appendRollRotation(double roll)
   {
      pose.appendRollRotation(roll);
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
      pose.interpolate(framePose1.pose, framePose2.pose, alpha);
   }

   public String printOutPosition()
   {
      return pose.printOutPosition();
   }

   public String printOutOrientation()
   {
      return pose.printOutOrientation();
   }

   public FramePoint3D getFramePointCopy()
   {
      FramePoint3D ret = new FramePoint3D(getReferenceFrame(), pose.getPosition());
      return ret;
   }

   public FrameQuaternion getFrameOrientationCopy()
   {
      FrameQuaternion ret = new FrameQuaternion(getReferenceFrame(), pose.getOrientation());
      return ret;
   }

   public FrameVector3D getTranslationToOtherPoseTotal(FramePose otherPose)
   {
      checkReferenceFrameMatch(otherPose);

      FrameVector3D ret = new FrameVector3D(referenceFrame);
      ret.sub(otherPose.pose.getPosition(), pose.getPosition());

      return ret;
   }

   public FrameVector3D getTranslationNOTDueToRotationAboutFrame(FramePose otherPose)
   {
      checkReferenceFrameMatch(otherPose);

      RigidBodyTransform transformToOtherPose = getTransformFromThisToThat(otherPose);
      FrameVector3D ret = new FrameVector3D(referenceFrame);
      transformToOtherPose.getTranslation(ret);

      return ret;
   }

   public FrameVector3D getTranslationDueToRotationAboutFrame(FramePose otherPose)
   {
      checkReferenceFrameMatch(otherPose);

      FrameVector3D ret = getTranslationToOtherPoseTotal(otherPose);
      ret.sub(getTranslationNOTDueToRotationAboutFrame(otherPose));
      return ret;
   }

   public double getPositionDistance(FramePose framePose)
   {
      checkReferenceFrameMatch(framePose);

      return pose.getPosition().distance(framePose.pose.getPosition());
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

   public double getSpatialAxisOfRotationAndAngleToOtherPose(FramePose otherPose, FrameVector3D rotationAxisToPack, FramePoint3D rotationAxisOriginToPack)
   {
      double rotationAngle = getAxisAngleRotationToOtherPose(otherPose, rotationAxisToPack);

      getOriginOfSpatialAxisOfRotationToOtherPose(otherPose, rotationAxisToPack, rotationAngle, rotationAxisOriginToPack);

      return rotationAngle;
   }

   private void getOriginOfSpatialAxisOfRotationToOtherPose(FramePose otherPose, FrameVector3D rotationAxis, double rotationAngle, FramePoint3D originToPack)
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
         originToPack.interpolate(pose.getPosition(), otherPose.pose.getPosition(), 0.5);
      }
      else
      {
         EuclidGeometryTools.topVertex3DOfIsoscelesTriangle3D(pose.getPosition(), otherPose.pose.getPosition(), rotationAxis.getVector(), rotationAngle,
                                                              originToPack.getPoint());
      }
   }

   public double getAxisAngleRotationToOtherPose(FramePose otherPose, FrameVector3D rotationAxisToPack)
   {
      AxisAngle rotationAxisAngle = new AxisAngle();
      getAxisAngleRotationToOtherPose(otherPose, rotationAxisAngle);

      rotationAxisToPack.setIncludingFrame(referenceFrame, rotationAxisAngle.getX(), rotationAxisAngle.getY(), rotationAxisAngle.getZ());

      return rotationAxisAngle.getAngle();
   }

   public void getAxisAngleRotationToOtherPose(FramePose otherPose, AxisAngle rotationToPack)
   {
      getTransformFromThisToThat(otherPose).getRotation(rotationToPack); //FIXME: this returns nonsense when angle = Math.PI
   }

   public double getOrientationDistance(FramePose framePose)
   {
      AxisAngle rotationFromThatToThis = new AxisAngle();
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

   public boolean epsilonEquals(FramePose other, double positionErrorMargin, double orientationErrorMargin)
   {
      return pose.getOrientation().epsilonEquals(other.pose.getOrientation(), orientationErrorMargin) && pose.getPosition().epsilonEquals(other.pose.getPosition(), positionErrorMargin);
   }
}
