package us.ihmc.tools.time;

public class Timer
{
   private final TimeProvider timeProvider;
   
   protected double currentTime = 0.0;
   protected double lastTime = 0.0;
   protected double deltaTime = 0.0;
   protected double startTime = 0.0;
   protected long numLaps = 0;
   protected double deltaSum = 0.0;
   
   public Timer()
   {
      timeProvider = new CpuTimeProvider();
   }
   
   public Timer(TimeProvider timeProvider)
   {
      this.timeProvider = timeProvider;
   }
   
   public Timer start()
   {
      startTime = timeProvider.now();
      lastTime = startTime;
      
      return this;
   }
   
   public void resetLap()
   {
      lastTime = timeProvider.now();
   }

   public void reset()
   {
      start();
      
      deltaSum = 0.0;
      numLaps = 0;
   }

   public double lap()
   {
      currentTime = timeProvider.now();
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
      return timeProvider.now() - startTime;
   }
   
   public double lapElapsed()
   {
      return timeProvider.now() - lastTime;
   }
}
