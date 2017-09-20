package us.ihmc.tools.io;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.TimeUnit;

public class TimerBasedOutputStream extends OutputStream
{
   private ByteArrayOutputStream baos = new ByteArrayOutputStream();
   private CountdownTimer countdownTimer;
   private ConcurrentLinkedQueue<String> outputQueue = new ConcurrentLinkedQueue<>();
   
   public TimerBasedOutputStream(int timerPeriod, TimeUnit timeUnit)
   {
      countdownTimer = new CountdownTimer(timerPeriod, timeUnit);
   }

   @Override
   public void write(int b) throws IOException
   {
      synchronized(baos)
      {
         baos.write(b);
         countdownTimer.reset();
      }
   }
   
   public boolean hasNewOutput()
   {
      return outputQueue.peek() != null;
   }
   
   public String getString()
   {
      return outputQueue.poll();
   }
   
   private class CountdownTimer
   {
      private final Timer timer = new Timer();
      private final int period;
      private final TimeUnit timeUnit;
      
      private TimerTask task;
      
      public CountdownTimer(int period, TimeUnit timeUnit)
      {
         this.period = period;
         this.timeUnit = timeUnit;
      }
      
      public void start()
      {
         synchronized(baos)
         {
            task = new TimerTask()
            {
               @Override
               public void run()
               {
                  synchronized(baos)
                  {
                     outputQueue.add(baos.toString());
                     baos.reset();
                  }
               }
            };
            
            timer.schedule(task, timeUnit.toMillis(period));
         }
      }
      
      public void reset()
      {
         if(task != null)
            task.cancel();
         
         start();
      }
   }
}
