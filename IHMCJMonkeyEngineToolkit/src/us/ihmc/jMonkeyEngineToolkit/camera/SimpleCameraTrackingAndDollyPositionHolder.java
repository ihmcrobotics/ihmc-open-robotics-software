package us.ihmc.jMonkeyEngineToolkit.camera;

import javax.vecmath.Point3d;

public class SimpleCameraTrackingAndDollyPositionHolder implements CameraTrackingAndDollyPositionHolder
{
   private final Point3d trackingPosition = new Point3d();
   private final Point3d dollyPosition = new Point3d();
   private double fieldOfView = 1.0;
   
   public void getTrackingPosition(Point3d trackingPositionToPack)
   {
      trackingPositionToPack.set(trackingPosition);
   }

   public void getDollyPosition(Point3d dollyPositionToPack)
   {
      dollyPositionToPack.set(dollyPosition);
   }

   public double getTrackingX()
   {
      return trackingPosition.getX();
   }

   public double getTrackingY()
   {
      return trackingPosition.getY();
   }

   public double getTrackingZ()
   {
      return trackingPosition.getZ();
   }

   public double getDollyX()
   {
      return dollyPosition.getX();
   }

   public double getDollyY()
   {
      return dollyPosition.getY();
   }

   public double getDollyZ()
   {
      return dollyPosition.getZ();
   }

   public double getFieldOfView()
   {
      return fieldOfView;
   }
   
   public void setTrackingPosition(Point3d trackingPosition)
   {
      this.trackingPosition.set(trackingPosition);
   }

   public void setDollyPosition(Point3d dollyPosition)
   {
      this.dollyPosition.set(dollyPosition);
   }
   
   public void setTrackingX(double cameraTrackingX)
   {
      this.trackingPosition.setX(cameraTrackingX);
   }
   
   public void setTrackingY(double cameraTrackingY)
   {
      this.trackingPosition.setY(cameraTrackingY);
   }
   
   public void setTrackingZ(double cameraTrackingZ)
   {
      this.trackingPosition.setZ(cameraTrackingZ);
   }
   
   public void setDollyX(double cameraDollyX)
   {
      this.dollyPosition.setX(cameraDollyX);
   }
   
   public void setDollyY(double cameraDollyY)
   {
      this.dollyPosition.setY(cameraDollyY);
   }
   
   public void setDollyZ(double cameraDollyZ)
   {
      this.dollyPosition.setZ(cameraDollyZ);
   }
   
   public void setFieldOfView(double fieldOfView)
   {
      this.fieldOfView = fieldOfView;
   }

   public void closeAndDispose()
   {      
   }


}
