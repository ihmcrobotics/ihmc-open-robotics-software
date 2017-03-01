package us.ihmc.jMonkeyEngineToolkit.camera;

import us.ihmc.euclid.tuple3D.Point3D;

public interface CameraTrackingAndDollyPositionHolder
{
   public abstract void getTrackingPosition(Point3D trackPositionToPack);
   
   public abstract void getDollyPosition(Point3D dollyPositionToPack);
   
   public abstract double getTrackingX();

   public abstract double getTrackingY();

   public abstract double getTrackingZ();

   public abstract double getDollyX();
 
   public abstract double getDollyY();

   public abstract double getDollyZ();
   
   public abstract double getFieldOfView();

   public abstract void closeAndDispose();


}
