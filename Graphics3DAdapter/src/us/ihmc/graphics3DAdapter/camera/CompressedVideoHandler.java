package us.ihmc.graphics3DAdapter.camera;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;


import us.ihmc.graphics3DAdapter.camera.VideoSettings.VideoCompressionKey;

public abstract class CompressedVideoHandler
{
   public abstract void newVideoPacketAvailable(long timeStamp, byte[] data, Point3d position, Quat4d orientation, double fieldOfView,
         VideoCompressionKey videoCompressionKey);

   public abstract void addNetStateListener(CompressedVideoDataServer compressedVideoDataServer);
   
   public abstract boolean isConnected();
}
