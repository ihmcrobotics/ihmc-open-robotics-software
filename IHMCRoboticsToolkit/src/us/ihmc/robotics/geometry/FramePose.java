package us.ihmc.robotics.geometry;

import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.Axis;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.random.RandomTools;

public class FramePose extends ReferenceFrameHolder
{
   private final FramePoint position;
   private final FrameOrientation orientation;
   private ReferenceFrame referenceFrame;
   private final Vector3d tempVector = new Vector3d();
   private final Matrix3d tempMatrix = new Matrix3d();

   public FramePose()
   {
      this(new FramePoint(), new FrameOrientation());
   }

   public FramePose(ReferenceFrame referenceFrame)
   {
      position = new FramePoint(referenceFrame);
      orientation = new FrameOrientation(referenceFrame);
      this.referenceFrame = referenceFrame;
   }

   public FramePose(FramePoint position, FrameOrientation orientation)
   {
      if (position.getReferenceFrame() != orientation.getReferenceFrame())
      {
         throw new ReferenceFrameMismatchException("FramePose: The position frame (" + position.getReferenceFrame()
               + ") does not match the orientation frame (" + orientation.getReferenceFrame() + ")");
      }
      this.position = new FramePoint(position);
      this.orientation = new FrameOrientation(orientation);
      referenceFrame = position.getReferenceFrame();
   }

   public FramePose(ReferenceFrame referenceFrame, Point3d position, AxisAngle4d orientation)
   {
      this.position = new FramePoint(referenceFrame, position);
      this.orientation = new FrameOrientation(referenceFrame, orientation);
      this.referenceFrame = referenceFrame;
   }

   public FramePose(ReferenceFrame referenceFrame, Point3d position, Quat4d orientation)
   {
      this.position = new FramePoint(referenceFrame, position);
      this.orientation = new FrameOrientation(referenceFrame, orientation);
      this.referenceFrame = referenceFrame;
   }

   public FramePose(FramePose framePose)
   {
      position = new FramePoint(framePose.position);
      orientation = new FrameOrientation(framePose.orientation);
      referenceFrame = framePose.referenceFrame;
   }

   public FramePose(ReferenceFrame referenceFrame, RigidBodyTransform transform)
   {
      transform.get(tempMatrix, tempVector);
      position = new FramePoint(referenceFrame, tempVector);
      orientation = new FrameOrientation(referenceFrame, tempMatrix);
      this.referenceFrame = referenceFrame;
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
      position.setX(x);
   }

   public void setY(double y)
   {
      position.setY(y);
   }

   public void setZ(double z)
   {
      position.setZ(z);
   }

   public void setPose(FramePose pose)
   {
      position.set(pose.position);
      orientation.set(pose.orientation);
   }

   public void setPose(FramePoint position, FrameOrientation orientation)
   {
      setPosition(position);
      setOrientation(orientation);
   }

   public void setPose(Point3d point, Quat4d orientation)
   {
      this.position.set(point);
      this.orientation.set(orientation);
   }
   
   public void setPose(Vector3d position, Quat4d orientation)
   {
      this.position.set(position);
      this.orientation.set(orientation);
   }

   public void setPose(RigidBodyTransform transform)
   {
      transform.get(tempVector);
      position.set(tempVector);
      transform.get(tempMatrix);
      orientation.set(tempMatrix);
   }

   public void setPoseIncludingFrame(FramePose pose)
   {
      position.setIncludingFrame(pose.position);
      orientation.setIncludingFrame(pose.orientation);
      referenceFrame = pose.referenceFrame;
   }

   public void setPoseIncludingFrame(FramePoint position, FrameOrientation orientation)
   {
      position.checkReferenceFrameMatch(orientation);
      this.position.setIncludingFrame(position);
      this.orientation.setIncludingFrame(orientation);
      referenceFrame = position.referenceFrame;
   }

   public void setPoseIncludingFrame(ReferenceFrame referenceFrame, Point3d point, Quat4d orientation)
   {
      position.setIncludingFrame(referenceFrame, point);
      this.orientation.setIncludingFrame(referenceFrame, orientation);
      this.referenceFrame = referenceFrame;
   }

   public void setPoseIncludingFrame(ReferenceFrame referenceFrame, RigidBodyTransform transform)
   {
      transform.get(tempVector);
      position.setIncludingFrame(referenceFrame, tempVector);
      transform.get(tempMatrix);
      orientation.setIncludingFrame(referenceFrame, tempMatrix);
      this.referenceFrame = referenceFrame;
   }

   public void setPosition(FramePoint position)
   {
      this.position.set(position);
   }

   public void setPosition(Tuple3d position)
   {
      this.position.set(position);
   }
   
   public void setPosition(double x, double y, double z)
   {
      this.position.set(x, y, z);
   }

   public void setOrientation(Quat4d orientation)
   {
      this.orientation.set(orientation);
   }

   public void setOrientation(Matrix3d orientation)
   {
      this.orientation.set(orientation);
   }

   public void setOrientation(AxisAngle4d orientation)
   {
      this.orientation.set(orientation);
   }

   public void setOrientation(double[] yawPitchRoll)
   {
      orientation.setYawPitchRoll(yawPitchRoll);
   }

   public void setOrientation(double yaw, double pitch, double roll)
   {
      orientation.setYawPitchRoll(yaw, pitch, roll);
   }

   public void setOrientation(FrameOrientation orientation)
   {
      this.orientation.set(orientation);
   }

   public void setXYFromPosition2d(FramePoint2d position2d)
   {
      position.setXY(position2d);
   }

   public void setToZero(ReferenceFrame referenceFrame)
   {
      position.setToZero(referenceFrame);
      orientation.setToZero(referenceFrame);
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
      orientation.getTransform3D(transformToPack);
      position.get(tempVector);
      transformToPack.setTranslation(tempVector);
   }

   public void getPoseIncludingFrame(FramePoint framePointToPack, FrameOrientation orientationToPack)
   {
      getPositionIncludingFrame(framePointToPack);
      getOrientationIncludingFrame(orientationToPack);
   }

   public void getPosition(Tuple3d tupleToPack)
   {
      position.changeFrame(referenceFrame);
      position.get(tupleToPack);
   }

   public void getPositionIncludingFrame(FrameTuple<?> frameTupleToPack)
   {
      position.changeFrame(referenceFrame);
      frameTupleToPack.setIncludingFrame(position);
   }

   public void getRigidBodyTransform(RigidBodyTransform transformToPack)
   {
      orientation.changeFrame(referenceFrame);
      position.changeFrame(referenceFrame);
      
      position.get(tempVector);
      orientation.getMatrix3d(tempMatrix);
      
      transformToPack.setTranslation(tempVector);
      transformToPack.setRotation(tempMatrix);
   }
   
   public void getOrientation(Matrix3d matrixToPack)
   {
      orientation.changeFrame(referenceFrame);
      orientation.getMatrix3d(matrixToPack);
   }

   public void getOrientation(Quat4d quaternionToPack)
   {
      orientation.changeFrame(referenceFrame);
      orientation.getQuaternion(quaternionToPack);
   }

   public void getOrientation(double[] yawPitchRoll)
   {
      orientation.getYawPitchRoll(yawPitchRoll);
   }
   
   public void getOrientation(AxisAngle4d axisAngleToPack)
   {
      orientation.getAxisAngle(axisAngleToPack);
   }
   

   public void getOrientationIncludingFrame(FrameOrientation orientationToPack)
   {
      orientation.changeFrame(referenceFrame);
      orientationToPack.setIncludingFrame(orientation);
   }

   public void getPose2dIncludingFrame(FramePose2d framePose2dToPack)
   {
      framePose2dToPack.setPoseIncludingFrame(referenceFrame, getX(), getY(), getYaw());
   }

   public void getPosition2dIncludingFrame(FramePoint2d framePoint2dToPack)
   {
      position.getFramePoint2d(framePoint2dToPack);
   }

   public void getOrientation2dIncludingFrame(FrameOrientation2d frameOrientation2dToPack)
   {
      orientation.getFrameOrientation2dIncludingFrame(frameOrientation2dToPack);
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
         this.getPosition(tempVector);
         axisRotationTransform.transform(tempVector);
         this.setPosition(tempVector);
      }
      
      if(!lockOrientation)
         this.orientation.applyTransform(axisRotationTransform);

      this.changeFrame(initialFrame);
   }
   
   public void translate(Tuple3d translation)
   {
      position.add(translation);
   }

   public void translate(double x, double y, double z)
   {
      position.add(x, y, z);
   }

   public double getX()
   {
      return position.getX();
   }

   public double getY()
   {
      return position.getY();
   }

   public double getZ()
   {
      return position.getZ();
   }

   public double getYaw()
   {
      return orientation.getYaw();
   }

   public double getPitch()
   {
      return orientation.getPitch();
   }

   public double getRoll()
   {
      return orientation.getRoll();
   }

   public void interpolate(FramePose framePose1, FramePose framePose2, double alpha)
   {
      position.interpolate(framePose1.position, framePose2.position, alpha);
      orientation.interpolate(framePose1.orientation, framePose2.orientation, alpha);
   }

   public void changeFrame(ReferenceFrame desiredFrame)
   {
      // this is in the correct frame already
      if (desiredFrame == referenceFrame)
      {
         return;
      }

      position.changeFrame(desiredFrame);
      orientation.changeFrame(desiredFrame);
      referenceFrame = desiredFrame;
   }

   public boolean epsilonEquals(FramePose framePose, double epsilon)
   {
      if (!position.epsilonEquals(framePose.position, epsilon))
         return false;

      if (!orientation.epsilonEquals(framePose.orientation, epsilon))
         return false;

      return true;
   }
   
   public boolean epsilonEquals(FramePose framePose, double positionEpsilon, double orientationEpsilon)
   {
      if (!position.epsilonEquals(framePose.position, positionEpsilon))
         return false;
      
      if (!orientation.epsilonEquals(framePose.orientation, orientationEpsilon))
         return false;
      
      return true;
   }

   public String printOutPosition()
   {
      return position.toString();
   }

   public String printOutOrientation()
   {
      return orientation.toString();
   }
   
   public FramePoint getFramePointCopy()
   {
      FramePoint ret = new FramePoint(position.getReferenceFrame(), position.getPoint());
      return ret;
   }
   
   public FrameOrientation getFrameOrientationCopy()
   {
      FrameOrientation ret = new FrameOrientation(orientation.getReferenceFrame(), orientation.getQuaternionCopy());
      return ret;
   }

   public FrameVector getTranslationToOtherPoseTotal(FramePose otherPose)
   {
      checkReferenceFrameMatch(otherPose);

      FrameVector ret = new FrameVector(referenceFrame);
      ret.sub(otherPose.position, this.position);

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
      
      return position.distance(framePose.getFramePointCopy());
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
         originToPack.interpolate(this.position, otherPose.position, 0.5);
      }
      else
      {
         GeometryTools.getTopVertexOfIsoscelesTriangle(position, otherPose.position, rotationAxis, rotationAngle, originToPack);
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
      double[] yawPitchRoll = new double[3];
      orientation.getYawPitchRoll(yawPitchRoll);
      return "Position: " + position.toString() + "\n" + orientation.toStringAsYawPitchRoll();
   }


}
