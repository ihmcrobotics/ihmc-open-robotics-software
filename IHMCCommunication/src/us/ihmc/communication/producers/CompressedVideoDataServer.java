package us.ihmc.communication.producers;

import us.ihmc.utilities.VideoDataServer;


public interface CompressedVideoDataServer extends VideoDataServer
{

   public void setVideoControlSettings(VideoControlSettings object);

}
