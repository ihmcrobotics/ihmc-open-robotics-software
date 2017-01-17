package us.ihmc.jMonkeyEngineToolkit.camera;

import java.awt.image.BufferedImage;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.robotics.robotSide.RobotSide;

public interface RenderedSceneHandler
{
   void close();

   void updateImage(RobotSide left, BufferedImage bufferedImage, long timeStamp, Point3d cameraPosition, Quat4d cameraOrientation,
         IntrinsicParameters intrinsicParameters);

   boolean isReadyForNewData();
}
