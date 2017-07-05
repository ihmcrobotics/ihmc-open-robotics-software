package us.ihmc.manipulation.planning.solarpanelmotion;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class SolarPanel
{
   private double sizeX;
   private double sizeY;
   private double sizeZ = 0.05;
   private Pose3D centerPose;
    
   public SolarPanel()
   {
      this.centerPose = new Pose3D();
      this.sizeX = 0;
      this.sizeY = 0;      
   }
   
   public SolarPanel(Pose3D pose, double sizeX, double sizeY)
   {
      this.sizeX = sizeX;
      this.sizeY = sizeY;
      this.centerPose = pose;      
   }
   
   public Pose3D getCenterPose()
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
