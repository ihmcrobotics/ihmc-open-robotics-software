package us.ihmc.avatar.sensors;

import java.awt.image.BufferedImage;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.communication.net.ObjectCommunicator;
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
   public void updateImage(RobotSide left, BufferedImage bufferedImage, long timeStamp, Point3d cameraPosition, Quat4d cameraOrientation,
         IntrinsicParameters intrinsicParameters)
   {
      LocalVideoPacket videoPacket = new LocalVideoPacket(timeStamp, bufferedImage, intrinsicParameters);
      objectCommunicator.consumeObject(videoPacket);
   }
}
