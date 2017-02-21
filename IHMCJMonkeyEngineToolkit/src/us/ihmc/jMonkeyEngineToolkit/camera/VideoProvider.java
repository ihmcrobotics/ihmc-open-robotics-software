package us.ihmc.jMonkeyEngineToolkit.camera;

import java.awt.image.BufferedImage;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public interface VideoProvider
{
   public BufferedImage getBufferedImage();

   public IntrinsicParameters getIntrinsicParameters();

   public Quaternion getCameraOrientation();

   public Point3D getCameraPosition();

}
