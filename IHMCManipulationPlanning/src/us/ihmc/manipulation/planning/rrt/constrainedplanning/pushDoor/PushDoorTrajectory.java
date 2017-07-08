package us.ihmc.manipulation.planning.rrt.constrainedplanning.pushDoor;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.manipulation.planning.trajectory.EndEffectorTrajectory;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.transformables.Pose;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.waypoints.EuclideanTrajectoryPointCalculator;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class PushDoorTrajectory extends EndEffectorTrajectory
{
   static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   private PushDoor pushDoor;   
   private double openingAngle = 0;   
   private double pitchAngle = 0;
   
   public PushDoorTrajectory(PushDoor pushDoor, double trajectoryTime, double openingAngle)
   {
      this.pushDoor = pushDoor;
      this.trajectoryTime = trajectoryTime;
      this.openingAngle = openingAngle;
   }
   
   public void setPitchAngle(double pitchAngle)
   {
      this.pitchAngle = pitchAngle;
   }   

   @Override
   public Pose getEndEffectorPose(double time)
   {
      double rotationAngle = time/trajectoryTime * openingAngle;
      
      PushDoorPose pushDoorPose = new PushDoorPose(pushDoor, getRobotSide(), rotationAngle, pitchAngle);
      
      return pushDoorPose.getEndEffectorPose();
   }

   @Override
   public HandTrajectoryMessage getEndEffectorTrajectoryMessage(ReferenceFrame midFeetFrame)
   {
      double firstTime = 5.0;
      int numberOfViaPoints = 10;
      HandTrajectoryMessage endEffectorTrajectoryMessage = new HandTrajectoryMessage(getRobotSide(), numberOfViaPoints);
      
      endEffectorTrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrame(worldFrame);
      endEffectorTrajectoryMessage.getFrameInformation().setDataReferenceFrame(worldFrame);
      
      EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();   
      
      for(int i=0;i<numberOfViaPoints;i++)
      {
         double time = trajectoryTime*((double)(i)/((double)(numberOfViaPoints)-1) );
         Pose pose = getEndEffectorPose(time);
         FramePoint framePoint = new FramePoint(midFeetFrame, pose.getPoint());
         framePoint.changeFrame(midFeetFrame);
         
         euclideanTrajectoryPointCalculator.appendTrajectoryPoint(framePoint.getPoint());
      }
      euclideanTrajectoryPointCalculator.computeTrajectoryPointTimes(firstTime, trajectoryTime);
      euclideanTrajectoryPointCalculator.computeTrajectoryPointVelocities(true);
      RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();
      
      for(int i=0;i<numberOfViaPoints;i++)
      {
         double time = trajectoryTime*((double)(i)/((double)(numberOfViaPoints)-1) );
         
         Point3D desiredPosition = new Point3D();
         Vector3D desiredLinearVelocity = new Vector3D();
         trajectoryPoints.get(i).get(desiredPosition, desiredLinearVelocity);
         Pose pose = getEndEffectorPose(time);
         FramePoint framePoint = new FramePoint(worldFrame, pose.getPoint());
         framePoint.changeFrame(midFeetFrame);
         
         Point3D point = new Point3D(pose.getPoint());
         Quaternion orientation = new Quaternion(pose.getOrientation());
         
         if(getRobotSide() == RobotSide.RIGHT)
         {
            orientation.appendRollRotation(-Math.PI*0.5);
         }
         else
         {
            orientation.appendRollRotation(Math.PI*0.5);   
         }
         
         time = time + firstTime;
         endEffectorTrajectoryMessage.setTrajectoryPoint(i, time, point, orientation, desiredLinearVelocity, new Vector3D(), worldFrame);
      }      
      
      return endEffectorTrajectoryMessage;
   }
}
