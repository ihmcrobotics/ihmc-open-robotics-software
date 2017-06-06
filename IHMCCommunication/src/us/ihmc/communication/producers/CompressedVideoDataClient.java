package us.ihmc.communication.producers;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.communication.net.NetStateListener;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface CompressedVideoDataClient extends NetStateListener
{

   public abstract void consumeObject(byte[] data, Point3DReadOnly position, QuaternionReadOnly orientation, IntrinsicParameters intrinsicParameters);

}
