package us.ihmc.manipulation.planning.manipulation.solarpanelmotion;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanel;
import us.ihmc.robotics.geometry.transformables.Pose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class SolarPanelCleaningPose
{
   private SolarPanel solarPanel;
   private double uCoordinate;
   private double vCoordinate;
   private double wCoordinate;
   private double zRotation;
   private Pose pose;
   
   private boolean isValidPose = true;
   
   public SolarPanelCleaningPose(SolarPanel solarPanel)
   {
      this.solarPanel = solarPanel;
      this.uCoordinate = 0;
      this.vCoordinate = 0;
      this.wCoordinate = 0;
      this.zRotation = 0;
      this.pose = getPose(uCoordinate, vCoordinate, wCoordinate, this.zRotation);
   }
   
   public SolarPanelCleaningPose(SolarPanelCleaningPose cleaningPose)
   {
      this.solarPanel = cleaningPose.solarPanel;
      this.uCoordinate = cleaningPose.uCoordinate;
      this.vCoordinate = cleaningPose.vCoordinate;
      this.wCoordinate = cleaningPose.wCoordinate;
      this.zRotation = cleaningPose.zRotation;
      this.pose = getPose(uCoordinate, vCoordinate, wCoordinate, this.zRotation);
   }
   
   public SolarPanelCleaningPose(SolarPanel solarPanel, double u, double v, double w)
   {
      this.solarPanel = solarPanel;
      this.uCoordinate = u;
      this.vCoordinate = v;
      this.wCoordinate = w;
      this.zRotation = 0;
      this.pose = getPose(uCoordinate, vCoordinate, wCoordinate, this.zRotation);
   }   
   
   public SolarPanelCleaningPose(SolarPanel solarPanel, double u, double v, double w, double zRotation)
   {
      this.solarPanel = solarPanel;
      this.uCoordinate = u;
      this.vCoordinate = v;
      this.wCoordinate = w;
      this.zRotation = zRotation;
      this.pose = getPose(uCoordinate, vCoordinate, wCoordinate, this.zRotation);
   } 
   
   public void setUcoordinate(double u)
   {
      this.uCoordinate = u;
      this.pose = getPose(uCoordinate, vCoordinate, wCoordinate, this.zRotation);
   }
   
   public void setVcoordinate(double v)
   {
      this.vCoordinate = v;
      this.pose = getPose(uCoordinate, vCoordinate, wCoordinate, this.zRotation);
   }
   
   public void setWcoordinate(double w)
   {
      this.wCoordinate = w;
      this.pose = getPose(uCoordinate, vCoordinate, wCoordinate, this.zRotation);
   }
   
   public void setUVWCoordinate(double u, double v, double w)
   {
      this.uCoordinate = u;
      this.vCoordinate = v;
      this.wCoordinate = w;
      this.pose = getPose(uCoordinate, vCoordinate, wCoordinate, this.zRotation);
   }
   
   public Pose getPose()
   {
      return this.pose;
   }
      
   private Pose getPose(double u, double v, double w, double zRotation)
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
      
      poseTransform.appendYawRotation(zRotation);
      
      pose.setPosition(poseTransform.getTranslationVector());
      pose.setOrientation(poseTransform.getRotationMatrix());
      
      return pose;
   }
      
   public void setZRotation(double zRotation)
   {
      this.zRotation = zRotation;
      
      this.pose = getPose(uCoordinate, vCoordinate, wCoordinate, this.zRotation);
   }
   
   public HandTrajectoryMessage getHandTrajectoryMessage(double motionTime)
   {      
      RigidBodyTransform handPoseTransform = new RigidBodyTransform(this.pose.getOrientation(), this.pose.getPosition());
      
      handPoseTransform.appendPitchRotation(-Math.PI/2);
      handPoseTransform.appendRollRotation(Math.PI/2);
      
      Point3D positionToWorld = new Point3D(this.pose.getPosition());
      
      Quaternion orientationToWorld = new Quaternion(handPoseTransform.getRotationMatrix());
            
      PrintTools.info(""+positionToWorld.getX()+" "+positionToWorld.getY()+" "+positionToWorld.getZ()+" ");
      
      HandTrajectoryMessage handMessage = new HandTrajectoryMessage(RobotSide.RIGHT, motionTime, positionToWorld, orientationToWorld, ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
            
      return handMessage;
   }
      
   public Point3D getDesiredHandPosition()
   {
      return new Point3D(this.pose.getPosition());
   }
   
   public Quaternion getDesiredHandOrientation()
   {
      RigidBodyTransform handPoseTransform = new RigidBodyTransform(this.pose.getOrientation(), this.pose.getPosition());
      
      handPoseTransform.appendPitchRotation(-Math.PI/2);
      handPoseTransform.appendRollRotation(Math.PI/2);
      
      return new Quaternion(handPoseTransform.getRotationMatrix());      
   }
   
   public double getU()
   {
      return uCoordinate;
   }
   
   public double getV()
   {
      return vCoordinate;
   }
   
   public double getW()
   {
      return wCoordinate;
   }
   
   public double getZRotation()
   {
      return zRotation;
   }   
   
   public void setValidity(boolean isValid)
   {
      isValidPose = isValid;
   }
   
   public boolean isValid()
   {
      return isValidPose;
   }
}
