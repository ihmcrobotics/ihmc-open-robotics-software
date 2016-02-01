package us.ihmc.tools.time;

public class Timer
{
   private double currentTime = 0.0;
   private double lastTime = 0.0;
   private double deltaTime = 0.0;
   private double startTime = 0.0;
   private long numLaps = 0;
   private double deltaSum = 0.0;
   
   public Timer start()
   {
      startTime = cpu();
      lastTime = startTime;
      
      return this;
   }
   
   public void resetLap()
   {
      lastTime = cpu();
   }

   public void reset()
   {
      start();
      
      deltaSum = 0.0;
      numLaps = 0;
   }

   public double lap()
   {
      currentTime = cpu();
      deltaTime = currentTime - lastTime;
      lastTime = currentTime;

      // for average lap
      numLaps++;
      deltaSum += deltaTime;
      
      return deltaTime;
   }
   
   public double averageLap()
   {
      double averageLap = Double.NaN;
      
      if (numLaps > 0)
         averageLap = deltaSum / numLaps;
      
      return averageLap;
   }
   
   public double totalElapsed()
   {
      return cpu() - startTime;
   }
   
   public double lapElapsed()
   {
      return cpu() - lastTime;
   }
   
   public static double cpu()
   {
      return System.nanoTime() / 1e9;
   }
}
