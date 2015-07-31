package us.ihmc.humanoidBehaviors.behaviors.scripts.engine;

import us.ihmc.robotics.geometry.ReferenceFrame;

public interface ScriptEngineUIInterface
{
   public void startRecordingScript(String filename, ReferenceFrame scriptingFrame);

   public void stopRecordingScript();
}
