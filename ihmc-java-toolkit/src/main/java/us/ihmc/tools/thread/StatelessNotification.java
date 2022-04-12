package us.ihmc.tools.thread;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;

public class StatelessNotification
{
   public synchronized void blockingWait()
   {
      ExceptionTools.handle(() -> this.wait(), DefaultExceptionHandler.RUNTIME_EXCEPTION);
   }

   /** THREAD 2 ACCESS BELOW THIS POINT */

   public synchronized void notifyOtherThread()
   {
      this.notifyAll(); // if wait has been called, notify it
   }
}
