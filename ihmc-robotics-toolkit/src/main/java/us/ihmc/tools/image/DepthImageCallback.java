package us.ihmc.tools.image;

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
    * @param image The image captured at timestamp as DepthImage
    * @param timeStamp timestamp in ns
    * @param cameraPosition position of the camera
    * @param cameraOrientation orientation of the camera
    * @param fov field of view
    */
   public void onNewImage(DepthImage image, long timeStamp, Point3DReadOnly cameraPosition, QuaternionReadOnly cameraOrientation, double fov);
   
   
   /**
    * Dispose of the consumer and release all underlying resources.
    */
   public void dispose();
   
}
