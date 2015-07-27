package us.ihmc.darpaRoboticsChallenge.sensors;

import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.communication.producers.RawVideoDataServer;
import us.ihmc.graphics3DAdapter.camera.RenderedSceneHandler;

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
