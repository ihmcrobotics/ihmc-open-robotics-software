package us.ihmc.avatar;

import us.ihmc.wholeBodyController.DRCOutputProcessor;

public class BarrierSchedulerTools
{
   /**
    * Creates a runnable to update the provided output processor that can be attached to either the scheduler or
    * controller thread depending on the thread setup.
    *
    * @param drcOutputProcessor to update.
    * @param controllerThread to provide timing information.
    * @return Runnable that updates the output processor.
    */
   public static Runnable createProcessorUpdater(DRCOutputProcessor drcOutputProcessor, AvatarControllerThread controllerThread)
   {
      return new Runnable()
      {
         boolean initialized = false;

         @Override
         public void run()
         {
            if (!controllerThread.getHumanoidRobotContextData().getControllerRan())
               return;
            if (!initialized)
               drcOutputProcessor.initialize();
            initialized = true;
            drcOutputProcessor.processAfterController(controllerThread.getHumanoidRobotContextData().getTimestamp());
         }
      };
   }
}
