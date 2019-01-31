package us.ihmc.robotEnvironmentAwareness.updaters;

import us.ihmc.messager.Messager;

public class REAStereoVisionUpdater
{
   private final Messager reaMessager;
   private final REAStereoVisionBuffer reaStereoVisionBuffer;

   public REAStereoVisionUpdater(REAStereoVisionBuffer stereoVisionBufferUpdater, Messager reaMessager)
   {
      this.reaMessager = reaMessager;
      this.reaStereoVisionBuffer = stereoVisionBufferUpdater;
   }

   public void update()
   {
      
   }
}
