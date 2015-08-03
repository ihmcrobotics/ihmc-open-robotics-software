package us.ihmc.communication.producers;

import java.awt.image.BufferedImage;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.robotics.robotSide.RobotSide;

public interface VideoDataServer
{
   public abstract void updateImage(RobotSide robotSide, BufferedImage bufferedImage, long timeStamp, Point3d cameraPosition, Quat4d cameraOrientation, IntrinsicParameters intrinsicParameters);

   public abstract void close();

   public abstract boolean isConnected();
}
