package us.ihmc.robotics.robotDescription;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;

public class CameraSensorDescription extends SensorDescription
{
   private double fieldOfView;
   private double clipNear;
   private double clipFar;

   private int imageWidth;
   private int imageHeight;

   public CameraSensorDescription(String name, Vector3D offsetFromJoint)
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

   public int getImageWidth()
   {
      return imageWidth;
   }

   public void setImageWidth(int imageWidth)
   {
      this.imageWidth = imageWidth;
   }

   public int getImageHeight()
   {
      return imageHeight;
   }

   public void setImageHeight(int imageHeight)
   {
      this.imageHeight = imageHeight;
   }

}
