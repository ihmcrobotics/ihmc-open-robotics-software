package us.ihmc.darpaRoboticsChallenge.networkProcessor.camera;

import java.awt.image.BufferedImage;
import java.io.IOException;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.darpaRoboticsChallenge.networkProcessor.messages.controller.RobotPoseData;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.state.RobotPoseBuffer;
import us.ihmc.darpaRoboticsChallenge.networking.bandwidthControl.DRCNetworkProcessorNetworkingManager;
import us.ihmc.graphics3DAdapter.camera.CompressedVideoDataServer;
import us.ihmc.graphics3DAdapter.camera.VideoPacket;
import us.ihmc.graphics3DAdapter.camera.VideoSettings;
import us.ihmc.utilities.net.NetStateListener;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.utilities.net.ObjectConsumer;

public abstract class CameraDataReceiver
{
   
   private final RobotPoseBuffer robotPoseBuffer;
   private final CompressedVideoDataServer compressedVideoDataServer;
   
   private final Point3d cameraPosition = new Point3d();
   private final Quat4d cameraOrientation = new Quat4d();
   private final Vector3d tempVector = new Vector3d();
   
   public CameraDataReceiver(RobotPoseBuffer robotPoseBuffer, VideoSettings videoSettings, final DRCNetworkProcessorNetworkingManager networkingManager)
   {
      this.robotPoseBuffer = robotPoseBuffer;
      ObjectCommunicator videoConsumer = new ObjectCommunicator()
      {
         
         public void consumeObject(Object object)
         {
            if(object instanceof VideoPacket)
            {
               networkingManager.sendVideoPacket((VideoPacket) object);
            }
         }
         
         public boolean isConnected()
         {
            return networkingManager.isConnected();
         }
         
         public void close()
         {
            networkingManager.close();
         }
         
         public void attachStateListener(NetStateListener stateListener)
         {
            networkingManager.attachStateListener(stateListener);
         }
         
         public <T> void attachListener(Class<T> clazz, ObjectConsumer<T> listener)
         {
            
         }

         public void connect() throws IOException
         {
         }
      };
      compressedVideoDataServer = new CompressedVideoDataServer(videoSettings, videoConsumer);
      
   }

   public void updateImage(BufferedImage bufferedImage, long timeStamp, double fov)
   {
      RobotPoseData robotPoseData = robotPoseBuffer.floorEntry(timeStamp);
      if(robotPoseData == null)
      {
         return;
      }
      
      robotPoseData.getCameraPose().get(cameraOrientation, tempVector);
      cameraPosition.set(tempVector);
      compressedVideoDataServer.updateImage(bufferedImage, timeStamp, cameraPosition, cameraOrientation, fov);
   }

}
