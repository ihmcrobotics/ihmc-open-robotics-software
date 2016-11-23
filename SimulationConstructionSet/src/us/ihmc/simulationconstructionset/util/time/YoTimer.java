package us.ihmc.simulationconstructionset.util.time;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.tools.time.Timer;

public class YoTimer extends Timer
{
   private DoubleYoVariable currentTime;
   private DoubleYoVariable lastTime;
   private DoubleYoVariable deltaTime;
   private DoubleYoVariable startTime;
   private LongYoVariable numLaps;
   private DoubleYoVariable deltaSum;
   
   public YoTimer(String name, DoubleYoVariable timeYoVariable, YoVariableRegistry registry)
   {
      this(timeYoVariable);
      
      currentTime = new DoubleYoVariable(name + "CurrentTime", registry);
      lastTime = new DoubleYoVariable(name + "LastTime", registry);
      deltaTime = new DoubleYoVariable(name + "DeltaTime", registry);
      startTime = new DoubleYoVariable(name + "StartTime", registry);
      numLaps = new LongYoVariable(name + "NumLaps", registry);
      deltaSum = new DoubleYoVariable(name + "DeltaSum", registry);
   }
   
   public YoTimer(DoubleYoVariable timeYoVariable)
   {
      super(new YoVariableTimeProvider(timeYoVariable));
   }
   
   @Override
   public YoTimer start()
   {
      super.start();
      updateYoVariables();
      return this;
   }
   
   @Override
   public void resetLap()
   {
      super.resetLap();
      updateYoVariables();
   }

   @Override
   public void reset()
   {
      super.reset();
      updateYoVariables();
   }

   @Override
   public double lap()
   {
      double lap = super.lap();
      updateYoVariables();
      return lap;
   }
   
   @Override
   public double averageLap()
   {
      double averageLap = super.lap();
      updateYoVariables();
      return averageLap;
   }
   
   @Override
   public double totalElapsed()
   {
      double totalElapsed = super.totalElapsed();
      updateYoVariables();
      return totalElapsed;
   }
   
   @Override
   public double lapElapsed()
   {
      double lapElapsed = super.lapElapsed();
      updateYoVariables();
      return lapElapsed;
   }
   
   private void updateYoVariables()
   {
      if (currentTime != null)
      {
         currentTime.set(super.currentTime);
         lastTime.set(super.lastTime);
         deltaTime.set(super.deltaTime);
         startTime.set(super.startTime);
         numLaps.set(super.numLaps);
         deltaSum.set(super.deltaSum);
      }
   }
}
