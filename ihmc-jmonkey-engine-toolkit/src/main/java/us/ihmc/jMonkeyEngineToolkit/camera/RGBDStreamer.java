package us.ihmc.jMonkeyEngineToolkit.camera;

import java.awt.image.BufferedImage;

import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface RGBDStreamer
{
   public long getTimeStamp();

   public Point3DReadOnly getCameraPosition();

   public QuaternionReadOnly getCameraOrientation();

   public double getFieldOfView();

   /**
    * Called every time a new RGBD image is available
    * 
    * @param image 2D image data from the camera. This image is re-used after this call
    * @param coordinates Point coordinates. Ordered from Left to Right, Top to Bottom. Invalid coordinates are set to Float.NaN. This array is re-used after this call
    * @param colors A-RGB values of the coordinates.  This array is re-used after this call
    * @param timeStamp
    * @param cameraPosition
    * @param cameraOrientation
    * @param fov TODO
    */
   public void updateRBGD(BufferedImage image, Point3D32[] coordinates, int[] colors, long timeStamp, Point3DReadOnly cameraPosition, QuaternionReadOnly cameraOrientation, double fov);

   /**
    * Function to check if a new frame should be captured and provided to the RGBD streamer
    * 
    * @return true if a new frame can be received
    */
   public boolean isReadyForNewData();


   /**
    * Get the nearest distance from the camera for measured points
    * 
    * Return 0 to disable near clipping. Note: The graphics engine might do its own clipping
    * 
    * @return Distance in meters
    */
   public double getNearClip();

   /**
    * Get the farthest distance form the camera for returned measured points
    * 
    * Return Double.POSITIVE_INFINITY to disable far clipping. Note: The graphics engine might do its own clipping
    * 
    * @return Distance in meters
    */
   public double getFarClip();

}
