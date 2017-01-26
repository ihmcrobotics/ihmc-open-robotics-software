package us.ihmc.simulationconstructionset.scripts;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;


public class TimerScript implements Script
{

   private final YoVariableRegistry registry;
   private final BooleanYoVariable startTimer;
   private final BooleanYoVariable timerEnabled;
   private final DoubleYoVariable timeElapsed;
   private final DoubleYoVariable startTime;
   
   public TimerScript(String name, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(name);
      startTimer = new BooleanYoVariable("startTimer", registry);
      timerEnabled = new BooleanYoVariable("timerEnabled", registry);
      timeElapsed = new DoubleYoVariable("timeElapsed", registry);
      startTime = new DoubleYoVariable("startTime", registry);
      parentRegistry.addChild(registry);
   }
   
   @Override
   public void doScript(double t)
   {
      if(startTimer.getBooleanValue())
      {
         startTimer.set(false);
         startTime.set(t);
         timerEnabled.set(true);
      }
      
      if(timerEnabled.getBooleanValue())
         timeElapsed.set(t - startTime.getDoubleValue());
   }
   
   public void startTimer()
   {
      startTimer.set(true);
   }

   public DoubleYoVariable getTimeElapsed()
   {
      return timeElapsed;
   }
}
