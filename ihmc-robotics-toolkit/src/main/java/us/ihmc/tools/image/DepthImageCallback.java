package us.ihmc.tools.image;

import java.awt.image.BufferedImage;

import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

/**
 * Interface to consume images.
 *
 */
public interface DepthImageCallback
{
   /**
    * Check if the consumer is ready to receive images. 
    * 
    * Do not call onNewFrame if this is false.
    */
   public boolean isAvailable();
   
   
   /**
    * Callback for new image
    * 
    * 
    * @param image The image captured at timestamp as BufferedImage
    * @param coordinates Point cloud with 3D coordinates of the points. Ordered Left to Right, Top to Bottom. Invalid coordinates are set to Float.NaN.
    * @param colors Colors corresponding to the 3D coordinates
    * @param timeStamp timestamp in ns
    * @param cameraPosition position of the camera
    * @param cameraOrientation orientation of the camera
    * @param fov field of view
    */
   public void onNewImage(BufferedImage image, Point3D32[] coordinates, int[] colors, long timeStamp, Point3DReadOnly cameraPosition, QuaternionReadOnly cameraOrientation, double fov);
   
   
   /**
    * Dispose of the consumer and release all underlying resources.
    */
   public void dispose();
   
}
