package us.ihmc.communication.producers;

import us.ihmc.communication.interfaces.Connectable;
import us.ihmc.communication.net.ConnectionStateListener;
import us.ihmc.communication.video.CompressedVideoCallback;

public interface CompressedVideoHandler extends Connectable, CompressedVideoCallback
{
   public void addNetStateListener(ConnectionStateListener compressedVideoDataServer);
}
