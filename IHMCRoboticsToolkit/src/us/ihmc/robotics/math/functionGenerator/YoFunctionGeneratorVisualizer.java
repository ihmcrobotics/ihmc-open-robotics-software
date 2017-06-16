package us.ihmc.robotics.math.functionGenerator;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.DoubleYoVariable;
import us.ihmc.yoVariables.variable.EnumYoVariable;
import us.ihmc.robotics.robotController.RobotController;

public class YoFunctionGeneratorVisualizer implements RobotController
{
   private YoVariableRegistry registry = new YoVariableRegistry("YoFunGenViz");
   
   private DoubleYoVariable valueCheck;
   
   private YoFunctionGenerator yoFunctionGenerator;
   private DoubleYoVariable time;
   
   private DoubleYoVariable resetTime;
   private DoubleYoVariable maxSweepFreq;
   private DoubleYoVariable amplitude;

   private YoBoolean hasBeenReset;

   private final EnumYoVariable<YoFunctionGeneratorMode> mode;

   
   public YoFunctionGeneratorVisualizer(YoFunctionGenerator yoFunctionGenerator)
   {
      this.yoFunctionGenerator = yoFunctionGenerator;
      
      mode = EnumYoVariable.create("Mode", YoFunctionGeneratorMode.class, registry);
      
      resetTime = new DoubleYoVariable("resetTime", registry);
      
      resetTime.set(20.0);
      
      maxSweepFreq = new DoubleYoVariable("maxSweepFreq", registry);
      maxSweepFreq.set(60.0);
      
      amplitude = new DoubleYoVariable("amplitude", registry);
      amplitude.set(1.0);
      
      valueCheck = new DoubleYoVariable("valueCheck", registry);
      
      hasBeenReset = new YoBoolean("hasBeenReset", registry);
      hasBeenReset.set(true);
   }
   
   @Override
   public void initialize()
   {
      
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return null;
   }

   @Override
   public String getDescription()
   {
      return null;
   }
   
   public void setTimeVariable(DoubleYoVariable time)
   {
      this.time = time;
   }
   
   
   @Override
   public void doControl()
   {
      if (!hasBeenReset.getBooleanValue() && !mode.getEnumValue().equals(YoFunctionGeneratorMode.OFF) && yoFunctionGenerator.getMode().equals(YoFunctionGeneratorMode.OFF))
      {
         mode.set(YoFunctionGeneratorMode.OFF);
         hasBeenReset.set(true);
      }
      
      if (!yoFunctionGenerator.getMode().equals(YoFunctionGeneratorMode.OFF))
         hasBeenReset.set(false);
      
      yoFunctionGenerator.setMode(mode.getEnumValue());
      yoFunctionGenerator.setResetTime(resetTime.getDoubleValue());
      yoFunctionGenerator.setChirpFrequencyMaxHz(maxSweepFreq.getDoubleValue());
      yoFunctionGenerator.setAmplitude(amplitude.getDoubleValue());
      
      valueCheck.set(this.yoFunctionGenerator.getValue(time.getDoubleValue()));
      
      try
      {
         Thread.sleep(1);
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
      }
   }
}


