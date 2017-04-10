package us.ihmc.robotDataLogger.guiRecorder;

import java.nio.ByteBuffer;

public interface GUICaptureHandler
{
   public void receivedFrame(ByteBuffer data);
}
