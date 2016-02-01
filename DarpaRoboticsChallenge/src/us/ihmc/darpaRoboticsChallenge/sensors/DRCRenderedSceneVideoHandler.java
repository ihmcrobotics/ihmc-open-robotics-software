package us.ihmc.darpaRoboticsChallenge.sensors;

import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.graphics3DAdapter.camera.RenderedSceneHandler;
import us.ihmc.humanoidRobotics.communication.producers.RawVideoDataServer;

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

}
