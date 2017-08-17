package us.ihmc.manipulation.planning.rrt.constrainedplanning.tools;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.rotationConversion.AxisAngleConversion;
import us.ihmc.euclid.rotationConversion.RotationVectorConversion;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.screwTheory.RigidBody;

public class ControlFrameMovement
{
   private static double ratioPositionToOrientation = 1.0;

   private Pose3D controlPose;
   private Pose3D controlPoseOld;

   private double deltaPose;

   public ControlFrameMovement()
   {
      
   }
   
   public ControlFrameMovement(RigidBody controlBody)
   {
      this();
      Pose3D controlPose = new Pose3D(controlBody.getBodyFixedFrame().getTransformToWorldFrame());
      setInitialPose(controlPose);
   }
   
   public ControlFrameMovement(Pose3D controlPose)
   {
      this();
      setInitialPose(controlPose);
   }
   
   private void setInitialPose(Pose3D controlPose)
   {
      this.controlPoseOld = new Pose3D(controlPose);
      this.deltaPose = 0.0;
   }

   public double getDeltaPose(RigidBody controlBody)
   {
      Pose3D newPose = new Pose3D(controlBody.getBodyFixedFrame().getTransformToWorldFrame());
      return getDeltaPose(newPose);
   }
   
   public double getDeltaPose(Pose3D newPose)
   {
      controlPose = new Pose3D(newPose);
      deltaPose = ratioPositionToOrientation * getDeltaOrientation() + getDeltaPosition();

      controlPoseOld = new Pose3D(newPose);
      return deltaPose;
   }

   private double getDeltaOrientation()
   {
      Quaternion orientationOne = new Quaternion(controlPoseOld.getOrientation());
      Quaternion orientationTwo = new Quaternion(controlPose.getOrientation());

      // orientation - linearize the angle of the rotational vector from orientationOne to orientationTwo
      Quaternion deltaOrientation = new Quaternion();
      Quaternion inverseOrientationOne = new Quaternion(orientationOne);

      inverseOrientationOne.inverse();

      deltaOrientation.multiply(inverseOrientationOne, orientationTwo);

      AxisAngle toGoal = new AxisAngle();
      Vector3D toGoalVector = new Vector3D();

      RotationVectorConversion.convertQuaternionToRotationVector(deltaOrientation, toGoalVector);
      AxisAngleConversion.convertRotationVectorToAxisAngle(toGoalVector, toGoal);

      AxisAngle toWayPoint = new AxisAngle(toGoal);
      double fullAngle = toWayPoint.getAngle();

      return fullAngle;
   }

   private double getDeltaPosition()
   {
      Point3D positionOne = new Point3D(controlPoseOld.getPosition());
      Point3D positionTwo = new Point3D(controlPose.getPosition());

      return positionOne.distance(positionTwo);
   }
}
