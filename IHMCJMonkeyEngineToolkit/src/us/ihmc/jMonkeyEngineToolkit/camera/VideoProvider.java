package us.ihmc.jMonkeyEngineToolkit.camera;

import java.awt.image.BufferedImage;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import boofcv.struct.calib.IntrinsicParameters;

public interface VideoProvider
{
   public BufferedImage getBufferedImage();

   public IntrinsicParameters getIntrinsicParameters();

   public Quat4d getCameraOrientation();

   public Point3d getCameraPosition();

}
