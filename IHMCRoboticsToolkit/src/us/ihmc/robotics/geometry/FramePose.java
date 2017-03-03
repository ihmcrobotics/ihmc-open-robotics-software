package us.ihmc.robotics.geometry;

import java.util.Random;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.transformables.Pose;
import us.ihmc.robotics.random.RandomGeometry;
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
         throw new ReferenceFrameMismatchException("FramePose: The position frame (" + position.getReferenceFrame() + ") does not match the orientation frame ("
               + orientation.getReferenceFrame() + ")");
      }
   }

   public FramePose(ReferenceFrame referenceFrame, Tuple3DReadOnly position, QuaternionReadOnly orientation)
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

   public FramePose(ReferenceFrame referenceFrame, Tuple3DReadOnly point3d, AxisAngleReadOnly axisAngle4d)
   {
      this(referenceFrame);
      setPose(point3d, axisAngle4d);
   }

   public static FramePose generateRandomFramePose(Random random, ReferenceFrame referenceFrame, double maxAbsoluteX, double maxAbsoluteY, double maxAbsoluteZ)
   {
      return new FramePose(referenceFrame, RandomGeometry.nextPoint3D(random, maxAbsoluteX, maxAbsoluteY, maxAbsoluteZ),
                           RandomGeometry.nextQuaternion(random));
   }

   public static FramePose generateRandomFramePose(Random random, ReferenceFrame referenceFrame, double[] xyzMin, double[] xyzMax, double[] yawPitchRollMin,
                                                   double[] yawPitchRollMax)
   {
      FramePose randomFramePose = new FramePose(FramePoint.generateRandomFramePoint(random, referenceFrame, xyzMin[0], xyzMax[0], xyzMin[1], xyzMax[1],
                                                                                    xyzMin[2], xyzMax[2]),
                                                FrameOrientation.generateRandomFrameOrientation(random, referenceFrame, yawPitchRollMin[0], yawPitchRollMax[0],
                                                                                                yawPitchRollMin[1], yawPitchRollMax[1], yawPitchRollMin[2],
                                                                                                yawPitchRollMax[2]));
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

   public void setPose(Tuple3DReadOnly position, QuaternionReadOnly orientation)
   {
      pose.setPose(position, orientation);
   }

   private void setPose(Tuple3DReadOnly position, AxisAngleReadOnly orientation)
   {
      pose.setPose(position, orientation);
   }

   public void setPose(RigidBodyTransform transform)
   {
      pose.setPose(transform);
   }

   public void setPoseIncludingFrame(FramePose framePose)
   {
      pose.set(framePose.pose);
      referenceFrame = framePose.referenceFrame;
   }

   public void setPoseIncludingFrame(FramePoint position, FrameOrientation orientation)
   {
      position.checkReferenceFrameMatch(orientation);
      referenceFrame = position.referenceFrame;
      setPose(position.getPoint(), orientation.getQuaternion());
   }

   public void setPoseIncludingFrame(ReferenceFrame referenceFrame, Point3DReadOnly position, QuaternionReadOnly orientation)
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
      pose.setYawPitchRoll(yawPitchRoll);
   }

   public void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      pose.setYawPitchRoll(yaw, pitch, roll);
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
      pose.getPose(transformToPack);
   }

   public void getPoseIncludingFrame(FramePoint framePointToPack, FrameOrientation orientationToPack)
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

   public void getPositionIncludingFrame(FrameTuple<?, ?> frameTupleToPack)
   {
      frameTupleToPack.setIncludingFrame(referenceFrame, pose.getPoint());
   }

   public void getRigidBodyTransform(RigidBodyTransform transformToPack)
   {
      pose.getRigidBodyTransform(transformToPack);
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
      pose.getYawPitchRoll(yawPitchRoll);
   }

   public void getOrientation(AxisAngleBasics axisAngleToPack)
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
      ReferenceFrame initialFrame = referenceFrame;

      changeFrame(rotationAxisFrame);

      RigidBodyTransform axisRotationTransform = new RigidBodyTransform();
      TransformTools.rotate(axisRotationTransform, angle, rotationAxis);

      if (!lockPosition)
      {
         Vector3D tempVector = new Vector3D();
         getPosition(tempVector);
         axisRotationTransform.transform(tempVector);
         this.setPosition(tempVector);
      }

      if (!lockOrientation)
         pose.applyTransformToOrientationOnly(axisRotationTransform);

      changeFrame(initialFrame);
   }

   public void translate(Tuple3DBasics translation)
   {
      pose.translate(translation);
   }

   public void translate(double x, double y, double z)
   {
      pose.translate(x, y, z);
   }
   
   public void translateLocally(Vector3DBasics translation)
   {
      pose.transformToWorld(translation);
      pose.translate(translation);
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
      ret.sub(otherPose.pose.getPoint(), pose.getPoint());

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

      return pose.getPoint().distance(framePose.pose.getPoint());
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
         originToPack.interpolate(pose.getPoint(), otherPose.pose.getPoint(), 0.5);
      }
      else
      {
         GeometryTools.getTopVertexOfIsoscelesTriangle(pose.getPoint(), otherPose.pose.getPoint(), rotationAxis.getVector(), rotationAngle,
                                                       originToPack.getPoint());
      }
   }

   public double getAxisAngleRotationToOtherPose(FramePose otherPose, FrameVector rotationAxisToPack)
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

   @Override
   public String toString()
   {
      return "Position: " + pose.getPoint().toString() + "\n" + pose.getOrientation().toString() + "  -- " + referenceFrame.getName();
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
