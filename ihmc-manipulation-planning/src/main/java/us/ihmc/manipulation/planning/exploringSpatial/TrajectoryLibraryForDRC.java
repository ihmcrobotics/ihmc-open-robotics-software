package us.ihmc.manipulation.planning.exploringSpatial;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class TrajectoryLibraryForDRC
{
   /**
    * (knobGraspingPose) is a pose to grasp door knob.
    * The axis for twisting knob is located toward the z-direction of the (knobGraspingPose).
    * The axis for opening door is located toward the reverse z-direction of the (knobGraspingPose).
    */
   public static Pose3D computeOpeningDoorTrajectory(double time, double trajectoryTime, double openingRadius, double openingAngle, boolean openingDirectionCW,
                                                     Pose3D knobGraspingPose, double twistTime, double twistRadius, double twistAngle, boolean twistDirectionCW)
   {
      RigidBodyTransform handControl = new RigidBodyTransform(knobGraspingPose.getOrientation(), knobGraspingPose.getPosition());

      double distanceBetweenKnobAndDoor = 0.0;
      Vector3D zVectorOfKnob = new Vector3D();
      handControl.getRotation().getColumn(2, zVectorOfKnob);

      Vector3D translationFromKnobToDoorPlane = new Vector3D();

      if (zVectorOfKnob.getY() > 0)
      {
         translationFromKnobToDoorPlane.setX(zVectorOfKnob.getY());
         translationFromKnobToDoorPlane.setY(-zVectorOfKnob.getX());
      }
      else
      {
         translationFromKnobToDoorPlane.setX(-zVectorOfKnob.getY());
         translationFromKnobToDoorPlane.setY(zVectorOfKnob.getX());
      }
      Vector3D twistAxis = new Vector3D(translationFromKnobToDoorPlane); // toward knob center to door plane.

      translationFromKnobToDoorPlane.scale(distanceBetweenKnobAndDoor);

      // twist
      {
         double phase = time / twistTime;
         if (phase > 1.0)
            phase = 1.0;
         handControl.appendTranslation(0, 0, twistRadius);

         twistAxis.normalize();
         AxisAngle twistAxisAngle = new AxisAngle(twistAxis, twistDirectionCW ? (twistAngle * phase) : (-twistAngle * phase));

         handControl.multiply(new RigidBodyTransform(twistAxisAngle, new Point3D()));
         handControl.appendTranslation(0, 0, -twistRadius);
      }

      // opening      
      if (time > twistTime)
      {
         double phase = (time - twistTime) / (trajectoryTime - twistTime);

         RigidBodyTransform knobCenterControl = new RigidBodyTransform(knobGraspingPose.getOrientation(), knobGraspingPose.getPosition());
         RotationMatrix knobCenterOrientationControl = new RotationMatrix(knobGraspingPose.getOrientation());
         knobCenterControl.prependTranslation(translationFromKnobToDoorPlane);
         knobCenterControl.appendTranslation(0, 0, twistRadius - openingRadius);

         knobCenterOrientationControl.prependYawRotation(openingDirectionCW ? (-openingAngle * phase) : (openingAngle * phase));
         knobCenterControl.getRotation().set(knobCenterOrientationControl);

         knobCenterControl.appendTranslation(0, 0, -twistRadius + openingRadius);

         translationFromKnobToDoorPlane.negate();
         knobCenterControl.prependTranslation(translationFromKnobToDoorPlane);

         handControl = new RigidBodyTransform(knobCenterControl);
         handControl.appendTranslation(0, 0, twistRadius);

         twistAxis.normalize();
         AxisAngle twistAxisAngle = new AxisAngle(twistAxis, twistDirectionCW ? (twistAngle * 1.0) : (-twistAngle * 1.0));
         handControl.multiply(new RigidBodyTransform(twistAxisAngle, new Point3D()));
         handControl.appendTranslation(0, 0, -twistRadius);
      }

      return new Pose3D(handControl);
   }

   /**
    * X-axis of hand is located toward Sky.
    * Z-axis of hand is located toward (wallNormalVector).
    */
   public static Pose3D computeCuttingWallTrajectory(double time, double trajectoryTime, double cuttingRadius, boolean cuttingDirectionCW,
                                                     Point3D cuttingCenterPosition, Vector3D wallNormalVector)
   {
      Vector3D xAxisRotationMatrix = new Vector3D(0, 0, 1);
      Vector3D yAxisRotationMatrix = new Vector3D();
      Vector3D zAxisRotationMatrix = new Vector3D(wallNormalVector);

      zAxisRotationMatrix.normalize();
      yAxisRotationMatrix.cross(zAxisRotationMatrix, xAxisRotationMatrix);

      RotationMatrix cuttingRotationMatrix = new RotationMatrix();
      cuttingRotationMatrix.setColumns(xAxisRotationMatrix, yAxisRotationMatrix, zAxisRotationMatrix);

      RigidBodyTransform handControl = new RigidBodyTransform(cuttingRotationMatrix, cuttingCenterPosition);

      double phase = time / trajectoryTime;

      handControl.appendYawRotation(cuttingDirectionCW ? -phase * 2 * Math.PI : phase * 2 * Math.PI);
      handControl.appendTranslation(0, cuttingRadius, 0);
      handControl.appendYawRotation(cuttingDirectionCW ? phase * 2 * Math.PI : -phase * 2 * Math.PI);

      return new Pose3D(handControl);
   }

   /**
    * Valkyrie is able to complete only half rotating motion.
    */
   public static Pose3D computeClosingValveTrajectory(double time, double trajectoryTime, double radius, boolean closingDirectionCW, double closingAngle,
                                                      Point3D valveCenterPosition, Vector3D valveNormalVector)
   {
      Vector3D xAxisHandFrame = new Vector3D(valveNormalVector);
      Vector3D yAxisHandFrame = new Vector3D();
      Vector3D zAxisHandFrame = new Vector3D(0, 0, 1);

      xAxisHandFrame.negate();
      xAxisHandFrame.normalize();
      yAxisHandFrame.cross(zAxisHandFrame, xAxisHandFrame);

      RotationMatrix handFrame = new RotationMatrix();
      handFrame.setColumns(xAxisHandFrame, yAxisHandFrame, zAxisHandFrame);
      handFrame.appendRollRotation(closingDirectionCW ? -0.5 * Math.PI : 0.5 * Math.PI);

      RigidBodyTransform handControl = new RigidBodyTransform(handFrame, valveCenterPosition);

      double phase = time / trajectoryTime;

      handControl.appendRollRotation(closingDirectionCW ? phase * closingAngle : -phase * closingAngle);
      handControl.appendTranslation(0, -radius, 0);

      return new Pose3D(handControl);
   }

   /**
    * trajectory for current transform to goal transform
    */
   public static Pose3D computeLinearTrajectory(double time, double trajectoryTime, RigidBodyTransform from, RigidBodyTransform to)
   {
      double progress = time / trajectoryTime;

      Point3D fromPoint = new Point3D(from.getTranslation());
      Point3D toPoint = new Point3D(to.getTranslation());

      Quaternion fromOrienation = new Quaternion(from.getRotation());
      Quaternion toOrienation = new Quaternion(to.getRotation());

      Point3D point = new Point3D();
      Quaternion orientation = new Quaternion();

      point.interpolate(fromPoint, toPoint, progress);
      orientation.interpolate(fromOrienation, toOrienation, progress);

      return new Pose3D(point, orientation);
   }

   /**
    * circular trajectory
    */
   public static Pose3D computeCircleTrajectory(double time, double trajectoryTime, double circleRadius, Point3DReadOnly circleCenter,
                                                Quaternion circleRotation, QuaternionReadOnly constantOrientation, boolean ccw, double phase)
   {
      double theta = (ccw ? -time : time) / trajectoryTime * 2.0 * Math.PI + phase;
      double z = circleRadius * Math.sin(theta);
      double y = circleRadius * Math.cos(theta);
      Point3D point = new Point3D(0.0, y, z);
      circleRotation.transform(point);
      point.add(circleCenter);

      return new Pose3D(point, constantOrientation);
   }
}
