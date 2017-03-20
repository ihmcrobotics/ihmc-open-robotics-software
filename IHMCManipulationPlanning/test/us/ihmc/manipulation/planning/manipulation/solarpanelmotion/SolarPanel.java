package us.ihmc.manipulation.planning.manipulation.solarpanelmotion;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.geometry.transformables.Pose;

public class SolarPanel
{
   private double sizeX;
   private double sizeY;
   private double sizeZ = 0.05;
   private Pose centerPose;
      
   public SolarPanel(Pose pose, double sizeX, double sizeY)
   {
      this.sizeX = sizeX;
      this.sizeY = sizeY;
      this.centerPose = pose;
   }
   
   public Pose getCenterPose()
   {
      return centerPose;
   }
   
   public double getSizeX()
   {
      return sizeX;
   }
   
   public double getSizeY()
   {
      return sizeY;
   }
   
   public double getSizeZ()
   {
      return sizeZ;
   }
   
   public RigidBodyTransform getRigidBodyTransform()
   {
      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform(centerPose.getOrientation(), centerPose.getPosition());
      
      return rigidBodyTransform;
   }
   
}
