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

   private Pose3D desiredPose;

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
      this.controlPose = new Pose3D(controlPose);
      this.controlPoseOld = new Pose3D(controlPose);
      this.deltaPose = 0.0;
   }

   public void setDesiredPose(Pose3D desiredPose)
   {
      this.desiredPose = desiredPose;
   }

   public double getError()
   {
      tempErrorPosition = Pose3DMovementCalculator.getDeltaPosition(controlPose, desiredPose);
      tempErrorOrientation = Pose3DMovementCalculator.getDeltaOrientation(controlPose, desiredPose);
      
      return ratioPositionToOrientation * Pose3DMovementCalculator.getDeltaOrientation(controlPose, desiredPose)
            + Pose3DMovementCalculator.getDeltaPosition(controlPose, desiredPose);
   }

   public double getDeltaPose(RigidBody controlBody)
   {
      Pose3D newPose = new Pose3D(controlBody.getBodyFixedFrame().getTransformToWorldFrame());
      return getDeltaPose(newPose);
   }

   public double getDeltaPose(Pose3D newPose)
   {
      controlPose = new Pose3D(newPose);
      deltaPose = ratioPositionToOrientation * Pose3DMovementCalculator.getDeltaOrientation(controlPoseOld, controlPose)
            + Pose3DMovementCalculator.getDeltaPosition(controlPoseOld, controlPose);

      controlPoseOld = new Pose3D(newPose);
      return deltaPose;
   }

   public double tempErrorPosition = 0.0;
   public double tempErrorOrientation = 0.0;

   static class Pose3DMovementCalculator
   {
      static double getDeltaOrientation(Pose3D controlPoseOne, Pose3D controlPoseTwo)
      {
         Quaternion orientationOne = new Quaternion(controlPoseOne.getOrientation());
         Quaternion orientationTwo = new Quaternion(controlPoseTwo.getOrientation());

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

         if(fullAngle > Math.PI)
            fullAngle = 2.0*Math.PI - fullAngle;
         else if(fullAngle < -Math.PI)
            fullAngle = 2.0*Math.PI + fullAngle;
         
         return fullAngle;
      }

      static double getDeltaPosition(Pose3D controlPoseOne, Pose3D controlPoseTwo)
      {
         Point3D positionOne = new Point3D(controlPoseOne.getPosition());
         Point3D positionTwo = new Point3D(controlPoseTwo.getPosition());

         return positionOne.distance(positionTwo);
      }
   }
}
