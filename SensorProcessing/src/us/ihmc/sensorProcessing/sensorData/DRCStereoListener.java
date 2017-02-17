package us.ihmc.sensorProcessing.sensorData;

import us.ihmc.euclid.transform.RigidBodyTransform;

/**
 * Listens for stereo images
 *
 * @author Peter Abeles
 */
public interface DRCStereoListener
{
   /**
    *
    * @param rigidBodyTransform 
    * @param robotSide TODO
    * @param image
    * @param timestamp Time stamp in nano-seconds
    */
//   public void newImageAvailable(RobotSide robotSide , BufferedImage image, long timestamp, IntrinsicParameters intrinsicParameters);

   public void newImageAvailable(CameraData data, RigidBodyTransform rigidBodyTransform);

}
