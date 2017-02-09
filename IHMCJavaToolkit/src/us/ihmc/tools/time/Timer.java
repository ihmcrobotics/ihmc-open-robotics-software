package us.ihmc.tools.time;

public class Timer
{
   private double currentTime;
   private double lastTime;
   private double deltaTime;
   private double startTime;
   private long numLaps;
   private double deltaSum;
   
   public Timer start()
   {
      reset();
      
      return this;
   }
   
   public void resetLap()
   {
      lastTime = now();
   }

   public void reset()
   {
      startTime = now();
      lastTime = startTime;
      
      numLaps = 0;
      deltaSum = 0.0;
   }

   public double lap()
   {
      currentTime = now();
      deltaTime = currentTime - lastTime;
      lastTime = currentTime;

      // for average lap
      numLaps++;
      deltaSum += deltaTime;
      
      return deltaTime;
   }
   
   public double averageLap()
   {
      return deltaSum / numLaps;
   }
   
   public double totalElapsed()
   {
      return now() - startTime;
   }
   
   public double lapElapsed()
   {
      return now() - lastTime;
   }
   
   private double now()
   {
      return System.nanoTime() / 1e9;
   }
}
