package us.ihmc.sensorProcessing.sensorData;

import java.awt.image.BufferedImage;

import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import boofcv.struct.calib.IntrinsicParameters;

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
