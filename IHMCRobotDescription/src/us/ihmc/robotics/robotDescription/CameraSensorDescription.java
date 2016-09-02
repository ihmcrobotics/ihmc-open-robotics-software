package us.ihmc.robotics.robotDescription;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;

public class CameraSensorDescription extends SensorDescription
{
   private double fieldOfView;
   private double clipNear;
   private double clipFar;

   public CameraSensorDescription(String name, Vector3d offsetFromJoint)
   {
      super(name, offsetFromJoint);
   }

   public CameraSensorDescription(String name, RigidBodyTransform transformToJoint)
   {
      super(name, transformToJoint);
   }

   public CameraSensorDescription(String name, RigidBodyTransform transformToJoint, double fieldOfView, double clipNear, double clipFar)
   {
      super(name, transformToJoint);

      this.setFieldOfView(fieldOfView);

      this.setClipNear(clipNear);
      this.setClipFar(clipFar);
   }

   public double getFieldOfView()
   {
      return fieldOfView;
   }

   public void setFieldOfView(double fieldOfView)
   {
      this.fieldOfView = fieldOfView;
   }

   public double getClipNear()
   {
      return clipNear;
   }

   public void setClipNear(double clipNear)
   {
      this.clipNear = clipNear;
   }

   public double getClipFar()
   {
      return clipFar;
   }

   public void setClipFar(double clipFar)
   {
      this.clipFar = clipFar;
   }

}
