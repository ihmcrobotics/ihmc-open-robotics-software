package us.ihmc.simulationconstructionset.scripts;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.DoubleYoVariable;


public class TimerScript implements Script
{

   private final YoVariableRegistry registry;
   private final YoBoolean startTimer;
   private final YoBoolean timerEnabled;
   private final DoubleYoVariable timeElapsed;
   private final DoubleYoVariable startTime;
   
   public TimerScript(String name, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(name);
      startTimer = new YoBoolean("startTimer", registry);
      timerEnabled = new YoBoolean("timerEnabled", registry);
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
