package us.ihmc.graphics3DAdapter.camera;

import javax.vecmath.Point3d;

public interface CameraTrackingAndDollyPositionHolder
{
   public abstract void getCameraTrackingPosition(Point3d trackPositionToPack);
   
   public abstract void getCameraDollyPosition(Point3d dollyPositionToPack);
   
   public abstract double getTrackingX();

   public abstract double getTrackingY();

   public abstract double getTrackingZ();

   public abstract double getDollyX();
 
   public abstract double getDollyY();

   public abstract double getDollyZ();
   
   public abstract double getFieldOfView();


}
