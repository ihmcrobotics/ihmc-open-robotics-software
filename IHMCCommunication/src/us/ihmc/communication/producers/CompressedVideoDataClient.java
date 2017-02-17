package us.ihmc.communication.producers;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.communication.net.NetStateListener;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public interface CompressedVideoDataClient extends NetStateListener
{

   public abstract void consumeObject(byte[] data, Point3D position, Quaternion orientation, IntrinsicParameters intrinsicParameters);

}
