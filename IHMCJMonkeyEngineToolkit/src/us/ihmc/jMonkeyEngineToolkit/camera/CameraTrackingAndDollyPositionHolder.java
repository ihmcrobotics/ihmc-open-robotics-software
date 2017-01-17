package us.ihmc.jMonkeyEngineToolkit.camera;

import javax.vecmath.Point3d;

public interface CameraTrackingAndDollyPositionHolder
{
   public abstract void getTrackingPosition(Point3d trackPositionToPack);
   
   public abstract void getDollyPosition(Point3d dollyPositionToPack);
   
   public abstract double getTrackingX();

   public abstract double getTrackingY();

   public abstract double getTrackingZ();

   public abstract double getDollyX();
 
   public abstract double getDollyY();

   public abstract double getDollyZ();
   
   public abstract double getFieldOfView();

   public abstract void closeAndDispose();


}
