package us.ihmc.manipulation.planning.rrt.constrainedplanning.pushDoor;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.manipulation.planning.trajectory.EndEffectorPose;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.transformables.Pose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class PushDoorPose implements EndEffectorPose
{
   static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   private PushDoor pushDoor;
   private double doorRotationAngle;
   private double pitchAngle;
   
   public PushDoorPose(PushDoor pushDoor, double doorRotationAngle, double pitchAngle)
   {
      this.pushDoor = pushDoor;
      this.doorRotationAngle = doorRotationAngle;
      this.pitchAngle = pitchAngle;
   }
   
   public void setPitchAngle(double pitchAngle)
   {
      this.pitchAngle = pitchAngle;
   }

   @Override
   public Pose getEndEffectorPose()
   {  
      FramePose dummyFramePose = new FramePose(pushDoor.getDoorAxis());      
      dummyFramePose.changeFrame(worldFrame);
      
      Point3D translation = new Point3D(dummyFramePose.getPosition());
      RotationMatrix orientation = new RotationMatrix(dummyFramePose.getOrientation());
      
      RigidBodyTransform endEffectorRigidBody = new RigidBodyTransform(orientation, translation);
      
      endEffectorRigidBody.appendYawRotation(doorRotationAngle);
      
      endEffectorRigidBody.appendTranslation(0.0, pushDoor.getRadius(), pushDoor.getKnobHeight());
      
      endEffectorRigidBody.appendPitchRotation(pitchAngle);
      
      return new Pose(endEffectorRigidBody);
   }
}
