package us.ihmc.jMonkeyEngineToolkit.camera;

import java.awt.image.BufferedImage;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;

public interface RenderedSceneHandler
{
   void close();

   void updateImage(RobotSide left, BufferedImage bufferedImage, long timeStamp, Point3DReadOnly cameraPosition, QuaternionReadOnly cameraOrientation,
         IntrinsicParameters intrinsicParameters);

   boolean isReadyForNewData();
}
