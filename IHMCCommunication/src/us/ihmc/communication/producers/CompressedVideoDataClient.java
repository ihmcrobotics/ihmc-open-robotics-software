package us.ihmc.communication.producers;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.net.NetStateListener;
import boofcv.struct.calib.IntrinsicParameters;

public interface CompressedVideoDataClient extends NetStateListener
{

   public abstract void consumeObject(byte[] data, Point3d position, Quat4d orientation, IntrinsicParameters intrinsicParameters);

}
