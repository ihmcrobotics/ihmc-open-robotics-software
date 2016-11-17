package us.ihmc.humanoidBehaviors.behaviors.behaviorServices;

import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.tools.thread.ThreadTools;

public abstract class ThreadedBehaviorService extends BehaviorService implements Runnable
{   
   private boolean running = false;
   private boolean paused = false;
   private final String threadName;
   
   public ThreadedBehaviorService(String threadName, CommunicationBridgeInterface communicationBridge)
   {
      super(threadName, communicationBridge);
      
      this.threadName = threadName;
   }
   
   @Override
   public void initialize()
   {
      if (!running)
      {
         running = true;
         paused = false;
         
         ThreadTools.startAThread(this, threadName);
      }
   }
   
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
   
   @Override
   public void pause()
   {
      paused = true;
   }
   
   @Override
   public void resume()
   {
      paused = false;
   }
   
   @Override
   public void stop()
   {
      running = false;
   }
   
   public abstract void doThreadAction();
}
