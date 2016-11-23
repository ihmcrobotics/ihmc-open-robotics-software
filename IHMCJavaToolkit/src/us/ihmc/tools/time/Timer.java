package us.ihmc.tools.time;

public class Timer
{
   private final TimeProvider timeProvider;
   
   protected double currentTime;
   protected double lastTime;
   protected double deltaTime;
   protected double startTime;
   protected long numLaps;
   protected double deltaSum;
   
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
      reset();
      
      return this;
   }
   
   public void resetLap()
   {
      lastTime = timeProvider.now();
   }

   public void reset()
   {
      startTime = timeProvider.now();
      lastTime = startTime;
      
      numLaps = 0;
      deltaSum = 0.0;
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
      return deltaSum / numLaps;
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
