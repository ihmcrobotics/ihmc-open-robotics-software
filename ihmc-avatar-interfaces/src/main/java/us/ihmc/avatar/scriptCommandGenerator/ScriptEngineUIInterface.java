package us.ihmc.avatar.scriptCommandGenerator;

import java.nio.file.Path;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public interface ScriptEngineUIInterface
{
   public void startRecordingScript(Path scriptDirectory, String baseFileName, ReferenceFrame scriptingFrame);

   public void stopRecordingScript();
}
