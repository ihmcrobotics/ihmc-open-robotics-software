package us.ihmc.avatar.logProcessor;

import us.ihmc.yoVariables.variable.YoDouble;

/** Work in progress, will likely be needed for future manipulation publications. For log data post-processing. */
public class SCS2LogJointTracker
{
   private final YoDouble jointPosition;

   public SCS2LogJointTracker(YoDouble jointPosition)
   {
      this.jointPosition = jointPosition;
   }

   public void update()
   {

   }
}
