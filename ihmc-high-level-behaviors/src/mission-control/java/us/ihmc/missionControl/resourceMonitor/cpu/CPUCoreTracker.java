package us.ihmc.missionControl.resourceMonitor.cpu;

public class CPUCoreTracker
{
   private final CPUCorePropertyTracker timeInUserModeTracker = new CPUCorePropertyTracker();
   private final CPUCorePropertyTracker timeNiceUserModeTracker = new CPUCorePropertyTracker();
   private final CPUCorePropertyTracker timeInKernelCodeTracker = new CPUCorePropertyTracker();
   private final CPUCorePropertyTracker idleTimeTracker = new CPUCorePropertyTracker();
   private final CPUCorePropertyTracker ioWaitTimeTracker = new CPUCorePropertyTracker();
   private final CPUCorePropertyTracker interruptTimeTracker = new CPUCorePropertyTracker();
   private final CPUCorePropertyTracker softIRQTimeTracker = new CPUCorePropertyTracker();

   int historySize = 10;

   boolean firstUpdate = true;

   float percentUsage;

   public void update(double timeInUserMode,
                      double timeNiceUserMode,
                      double timeInKernelCode,
                      double idleTime,
                      double ioWaitTime,
                      double interruptTime,
                      double softIRQTime)
   {
      historySize = 3;
      timeInUserModeTracker.addIncrement(historySize, timeInUserMode);
      timeNiceUserModeTracker.addIncrement(historySize, timeNiceUserMode);
      timeInKernelCodeTracker.addIncrement(historySize, timeInKernelCode);
      idleTimeTracker.addIncrement(historySize, idleTime);
      ioWaitTimeTracker.addIncrement(historySize, ioWaitTime);
      interruptTimeTracker.addIncrement(historySize, interruptTime);
      softIRQTimeTracker.addIncrement(historySize, softIRQTime);

      if (firstUpdate)
      {
         firstUpdate = false;
         percentUsage = 0.0f;
      }
      else
      {
         float idleTimeAverage = (float) idleTimeTracker.getAverage();
         float everythingButIdle = (float) (timeInUserModeTracker.getAverage() + timeNiceUserModeTracker.getAverage() + timeInKernelCodeTracker.getAverage()
                                            + ioWaitTimeTracker.getAverage() + interruptTimeTracker.getAverage() + softIRQTimeTracker.getAverage());
         percentUsage = (float) (100.0 * (everythingButIdle / (idleTimeAverage + everythingButIdle)));
      }
   }

   public float getPercentUsage()
   {
      return percentUsage;
   }
}
