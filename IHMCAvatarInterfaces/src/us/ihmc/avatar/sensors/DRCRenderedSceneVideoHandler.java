package us.ihmc.avatar.sensors;

import java.awt.image.BufferedImage;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.LocalVideoPacket;
import us.ihmc.humanoidRobotics.communication.producers.RawVideoDataServer;
import us.ihmc.jMonkeyEngineToolkit.camera.RenderedSceneHandler;
import us.ihmc.robotics.robotSide.RobotSide;

public class DRCRenderedSceneVideoHandler extends RawVideoDataServer implements RenderedSceneHandler
{

   public DRCRenderedSceneVideoHandler(ObjectCommunicator objectCommunicator)
   {
      super(objectCommunicator);
   }

   @Override
   public boolean isReadyForNewData()
   {
      return isConnected();
   }

   @Override
   public void updateImage(RobotSide left, BufferedImage bufferedImage, long timeStamp, Point3DReadOnly cameraPosition, QuaternionReadOnly cameraOrientation,
         IntrinsicParameters intrinsicParameters)
   {
      LocalVideoPacket videoPacket = new LocalVideoPacket(timeStamp, bufferedImage, intrinsicParameters);
      objectCommunicator.consumeObject(videoPacket);
   }
}
