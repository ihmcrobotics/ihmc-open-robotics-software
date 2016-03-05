package us.ihmc.simulationconstructionset.gui;

import javax.swing.SwingUtilities;

public class EventDispatchThreadHelper
{
   public static void checkThatInEventDispatchThread()
   {
      if (!SwingUtilities.isEventDispatchThread())
      {
         throw new RuntimeException("Should only call this method from the Swing Event Dispatch Thread");
      }
   }
   
   public static void invokeAndWait(Runnable runnable)
   {
      if (SwingUtilities.isEventDispatchThread())
      {
         runnable.run();
      }
      else
      {
         try
         {
            SwingUtilities.invokeAndWait(runnable);
         }
         catch (Exception e)
         {
            throw new RuntimeException(e);
         }
      }
   }

   public static void justRun(Runnable runnable)
   {
      runnable.run();
   }

   public static void invokeLater(Runnable runnable)
   {
      if (SwingUtilities.isEventDispatchThread())
      {
         runnable.run();
      }
      else
      {
         SwingUtilities.invokeLater(runnable);
      }
   }
}
