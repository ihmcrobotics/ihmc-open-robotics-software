package us.ihmc.humanoidBehaviors.behaviors.scripts.engine;

import java.nio.file.Path;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public interface ScriptEngineUIInterface
{
   public void startRecordingScript(Path scriptDirectory, String baseFileName, ReferenceFrame scriptingFrame);

   public void stopRecordingScript();
}
