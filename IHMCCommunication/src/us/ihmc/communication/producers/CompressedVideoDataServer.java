package us.ihmc.communication.producers;

import us.ihmc.proprietaryUtilities.VideoDataServer;


public interface CompressedVideoDataServer extends VideoDataServer
{

   public void setVideoControlSettings(VideoControlSettings object);

}
