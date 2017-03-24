package us.ihmc.robotDataLogger.gui;

import java.nio.ByteBuffer;

public interface GUICaptureHandler
{
   public void receivedFrame(ByteBuffer data);
}
