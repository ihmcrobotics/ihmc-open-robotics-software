package us.ihmc.graphics3DAdapter.camera;

import java.awt.Canvas;
import java.util.ArrayList;

import javax.vecmath.Point3d;

import us.ihmc.graphics3DAdapter.CameraAdapter;
import us.ihmc.graphics3DAdapter.ContextSwitchedListener;
import us.ihmc.graphics3DAdapter.Graphics3DFrameListener;


public abstract class ViewportAdapter
{
   private ArrayList<Graphics3DFrameListener> frameListeners = new ArrayList<Graphics3DFrameListener>();

   public abstract Canvas getCanvas();

   public abstract CaptureDevice getCaptureDevice();

   public abstract void setCameraController(CameraController cameraController);

   public abstract CameraController getCameraController();

   public abstract CameraAdapter getCamera();

   public abstract double getFieldOfView();

   public abstract double getPhysicalWidth();

   public abstract double getPhysicalHeight();

   public abstract void setupOffscreenView(int width, int height);

   public abstract Point3d getWorldCoordinatesFromScreenCoordinates(float f, float g, double z);

   public abstract void addContextSwitchedListener(ContextSwitchedListener contextSwitchedListener);

   public void addFrameListener(Graphics3DFrameListener frameListener)
   {
      frameListeners.add(frameListener);
   }

   public ArrayList<Graphics3DFrameListener> getFrameListeners()
   {
      return frameListeners;
   }
}
