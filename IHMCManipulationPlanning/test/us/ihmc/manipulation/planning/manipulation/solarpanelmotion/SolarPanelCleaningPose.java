package us.ihmc.manipulation.planning.manipulation.solarpanelmotion;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.transformables.Pose;

public class SolarPanelCleaningPose
{
   // Euler angle is from X-Y'-Z'' intrinsic rotation.
   private Point3D location;
   private double rotationX;
   private double rotationY;
   private double rotationZ = Double.NaN;

   private Quaternion quaternion;
   private Pose pose;

   public SolarPanelCleaningPose()
   {
      this.location = new Point3D();
      this.rotationX = 0;
      this.rotationY = 0;
      this.quaternion = new Quaternion();      
   }
   
   public SolarPanelCleaningPose(Point3D location, double rotationX, double rotationY)
   {
      this.location = location;
      this.rotationX = rotationX;
      this.rotationY = rotationY;
      this.quaternion = new Quaternion();      
   }
   
   public SolarPanelCleaningPose(Point3D location, double rotationX, double rotationY, double rotationZ)
   {
      this.location = location;
      this.rotationX = rotationX;
      this.rotationY = rotationY;
      this.rotationZ = rotationZ;
      getQuaternion();  
   }

   public Point3D getLocation()
   {
      return location;
   }

   public void setRotationX(double rotationX)
   {
      this.rotationX = rotationX;
   }
   
   public void setRotationY(double rotationY)
   {
      this.rotationY = rotationY;
   }
   
   public void setRotationZ(double rotationZ)
   {
      this.rotationZ = rotationZ;
   }

   public void getQuaternion()
   {
      if (Double.isNaN(this.rotationZ) == true)
      {
         PrintTools.info("please setRotationZ");
      }
      else
      {
         quaternion = new Quaternion();
         quaternion.appendRollRotation(this.rotationX);
         quaternion.appendPitchRotation(this.rotationY);
         quaternion.appendYawRotation(this.rotationZ);
      }
   }
   
   public Quaternion getRotation()
   {
      getQuaternion();
      return quaternion;
   }

   public Pose getPose()
   {
      this.pose = new Pose(this.location, getRotation());
      
      return pose;
   }
   
   public Pose getPose(double rotationZ)
   {
      setRotationZ(rotationZ);
      return getPose();
   }
}
