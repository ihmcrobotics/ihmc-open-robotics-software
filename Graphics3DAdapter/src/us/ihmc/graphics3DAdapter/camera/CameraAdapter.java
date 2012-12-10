package us.ihmc.graphics3DAdapter.camera;

import java.awt.Canvas;


public interface CameraAdapter
{

   public abstract Canvas getCanvas();
   public abstract CaptureDevice getCaptureDevice();

   public abstract CameraController getCameraController();

   public abstract void setClipDistances(double near, double far);

   public abstract void setFieldOfView(double fieldOfView);

   public abstract void destroy();
   public abstract void registerAsKeyListener();


}