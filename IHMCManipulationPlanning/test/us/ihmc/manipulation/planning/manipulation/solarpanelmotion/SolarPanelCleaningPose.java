package us.ihmc.manipulation.planning.manipulation.solarpanelmotion;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.robotics.geometry.transformables.Pose;
import us.ihmc.robotics.robotSide.RobotSide;

public class SolarPanelCleaningPose
{
   private SolarPanel solarPanel;
   private double uCoordinate;
   private double vCoordinate;
   private double wCoordinate;
   private Pose pose;
   
   public SolarPanelCleaningPose(SolarPanel solarPanel, double u, double v, double w)
   {
      this.solarPanel = solarPanel;
      this.uCoordinate = u;
      this.vCoordinate = v;
      this.wCoordinate = w;
      this.pose = getPose(uCoordinate, vCoordinate, wCoordinate);
   }
   
   public Pose getPose()
   {
      return this.pose;
   }
   
   private Pose getPose(double u, double v, double w)
   {
      Pose pose = new Pose();
      
      RigidBodyTransform poseTransform = solarPanel.getRigidBodyTransform();
      
      Point3D translation;
      translation = new Point3D(solarPanel.getSizeX()/2, solarPanel.getSizeY()/2, 0);
      poseTransform.appendTranslation(translation);
      
      poseTransform.appendPitchRotation(Math.PI);
      poseTransform.appendYawRotation(-Math.PI/2);      
      
      Point3D uvwCoordinate;
      uvwCoordinate = new Point3D(u, v, w);
      poseTransform.appendTranslation(uvwCoordinate);
      
      pose.setPosition(poseTransform.getTranslationVector());
      pose.setOrientation(poseTransform.getRotationMatrix());
      
      return pose;
   }
   
   public void setZRotation(double zRotation)
   {
      RigidBodyTransform appendTransform = new RigidBodyTransform();
      appendTransform.setIdentity();
      appendTransform.appendYawRotation(zRotation);
            
      pose.appendTransform(appendTransform);
   }
   
   public HandTrajectoryMessage getHandTrajectoryMessage(double motionTime)
   {      
      RigidBodyTransform handPoseTransform = new RigidBodyTransform(this.pose.getOrientation(), this.pose.getPosition());
      
      handPoseTransform.appendPitchRotation(-Math.PI/2);
      handPoseTransform.appendRollRotation(Math.PI/2);
      
      Point3D positionToWorld = new Point3D(this.pose.getPosition());
      
      Quaternion orientationToWorld = new Quaternion(handPoseTransform.getRotationMatrix());
            
      PrintTools.info(""+positionToWorld.getX()+" "+positionToWorld.getY()+" "+positionToWorld.getZ()+" ");
      
      HandTrajectoryMessage handMessage = new HandTrajectoryMessage(RobotSide.RIGHT, motionTime, positionToWorld, orientationToWorld);
      
      
      
      return handMessage;
   }
}
