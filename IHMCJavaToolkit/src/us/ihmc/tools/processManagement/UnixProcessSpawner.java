package us.ihmc.tools.processManagement;

import us.ihmc.commons.PrintTools;

/**
 * @author Igor Kalkov <a href="mailto:ikalkov@ihmc.us">(ikalkov@ihmc.us)</a>
 */
public abstract class UnixProcessSpawner extends ProcessSpawner
{
   public UnixProcessSpawner(boolean killChildProcessesOnShutdown)
   {
      super(killChildProcessesOnShutdown);
   }

   @Override
   public void kill(Process process)
   {
      if (UnixProcessKiller.isUnixProcess(process))
      {
         PrintTools.info("Attempting to SigInt process");
         attemptSigInt(process);

         if (hasProcessExited(process))
         {
            return;
         }
      }

      PrintTools.info("SigInt failed, escalting to SigTerm");
      attemptDestroy(process);
   }

   private void attemptSigInt(Process process)
   {
      Object o = waitForUnixProcessSigInt(process);
      synchronized (o)
      {
         try
         {

            o.wait(5000);
         }
         catch (InterruptedException e)
         {
            e.printStackTrace();
         }
      }
   }

   private void attemptDestroy(Process process)
   {
      Object o = waitForProcessDestroy(process);

      synchronized (o)
      {
         try
         {
            o.wait(2000);
         }
         catch (InterruptedException e)
         {
            e.printStackTrace();
         }
      }
   }

   private Object waitForUnixProcessSigInt(final Process process)
   {
      final Object monitorObject = new Object();

      new Thread(new Runnable()
      {
         @Override public void run()
         {
            UnixProcessKiller.killSigIntUnixProcess(process);
            try
            {
               process.waitFor();
               synchronized (monitorObject)
               {
                  monitorObject.notifyAll();
               }
            }
            catch (InterruptedException e)
            {
               // TODO Auto-generated catch block
               e.printStackTrace();
            }
         }
      }).start();

      return monitorObject;
   }

   private Object waitForProcessDestroy(final Process process)
   {
      final Object monitorObject = new Object();

      new Thread(new Runnable()
      {
         @Override public void run()
         {
            process.destroy();
            try
            {
               process.waitFor();
               synchronized (monitorObject)
               {
                  monitorObject.notifyAll();
               }
            }
            catch (InterruptedException e)
            {
               e.printStackTrace();
            }
         }
      }).start();

      return monitorObject;
   }

   private boolean hasProcessExited(Process process)
   {
      try
      {
         process.exitValue();
      }
      catch (IllegalThreadStateException e)
      {
         return false;
      }

      return true;
   }
}
