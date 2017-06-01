package us.ihmc.manipulation.planning.solarpanelmotion;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.transformables.Pose;

public class SolarPanel
{
   private double sizeU;
   private double sizeV;
   private double sizeZ = 0.05;
   private Pose centerPose;
    
   public SolarPanel()
   {
      this.centerPose = new Pose();
      this.sizeU = 0;
      this.sizeV = 0;      
   }
   
   public SolarPanel(Pose pose, double sizeU, double sizeV)
   {
      this.sizeU = sizeU;
      this.sizeV = sizeV;
      this.centerPose = pose;      
   }
   
   public Pose getCenterPose()
   {
      return centerPose;
   }
   
   public double getSizeU()
   {
      return sizeU;
   }
   
   public double getSizeV()
   {
      return sizeV;
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
