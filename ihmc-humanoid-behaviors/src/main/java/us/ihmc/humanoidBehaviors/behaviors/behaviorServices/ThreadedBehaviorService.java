package us.ihmc.humanoidBehaviors.behaviors.behaviorServices;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.ros2.Ros2Node;

public abstract class ThreadedBehaviorService extends BehaviorService
{   
   private boolean running = false;
   private boolean paused = false;
   private final String threadName;
   
   public ThreadedBehaviorService(String robotName, String threadName, Ros2Node ros2Node)
   {
      super(robotName, threadName, ros2Node);
      
      this.threadName = threadName;
   }
   
   @Override
   public void run()
   {
      if (!running)
      {
         running = true;
         paused = false;
         
         ThreadTools.startAThread(new Run(), threadName);
      }
      else if (paused)
      {
         paused = false;
      }
   }
   
   private class Run implements Runnable
   {
      @Override
      public void run()
      {
         while (running)
         {
            if (!paused)
            {
               doThreadAction();
            }
            else
            {
               ThreadTools.sleep(300L);
            }
         }
      }
   }
   
   @Override
   public void pause()
   {
      paused = true;
   }
   
   @Override
   public void destroy()
   {
      running = false;
   }
   
   public abstract void doThreadAction();
   
   public abstract void initialize();
}
