package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

   public class FirstOrderFilteredYoVariable extends DoubleYoVariable
   {
      
      public enum FirstOrderFilterType
      {
         LOW_PASS, HIGH_PASS
      }
      
      private boolean hasBeenCalled = false;
      
      private double filterInputOld;
      private double filterUpdateTimeOld;
      
      private final DoubleYoVariable cutoffFrequency_Hz;

      private final DoubleYoVariable yoTime;
      private double dt;
      
      private FirstOrderFilterType highOrLowPass;
            
      public FirstOrderFilteredYoVariable(String name, String description, double cutoffFrequency_Hz, DoubleYoVariable yoTime, FirstOrderFilterType highOrLowPass, YoVariableRegistry registry)
      {
         super(name, description, registry);
         
         String cutoffFrequencyName;
         switch (highOrLowPass)
         {
         case LOW_PASS:
            cutoffFrequencyName = name + "_LowPassCutoff_Hz";
            break;
         case HIGH_PASS:
            cutoffFrequencyName = name + "_HighPassCutoff_Hz";
            break;
         default:
            throw new RuntimeException("Must Specify Filter Type as Low or High Pass.  Current Specification : " + highOrLowPass);
         }

         this.cutoffFrequency_Hz = new DoubleYoVariable(cutoffFrequencyName, registry);
         this.cutoffFrequency_Hz.set(cutoffFrequency_Hz);

         this.yoTime = yoTime;
         
         this.highOrLowPass = highOrLowPass;
      }
      
      public FirstOrderFilteredYoVariable(String name, String description, double cutoffFrequency_Hz, double DT, FirstOrderFilterType highOrLowPass, YoVariableRegistry registry)
      {
         this(name, description, cutoffFrequency_Hz, null, highOrLowPass, registry);
         this.dt = DT;
      }
      
      private double computeLowPassUpdate(double filterInput, double dt)
      {         
         double alpha = computeAlpha( dt, cutoffFrequency_Hz.getDoubleValue());
         
         double ret = alpha * this.getDoubleValue() + (1.0-alpha) * filterInput;

         return ret;
      }
      
      private double computeHighPassUpdate(double filterInput, double dt)
      {
         double alpha = computeAlpha(dt, cutoffFrequency_Hz.getDoubleValue());
         
         double ret = alpha * (this.getDoubleValue() + filterInput - filterInputOld);
         return ret;
      }

      private double computeAlpha(double dt, double cutoffFrequencyHz)
      {
         if (cutoffFrequencyHz <= 0.0)
         {
            throw new RuntimeException("Cutoff Frequency must be greater than zero.  Cutoff = " + cutoffFrequencyHz);
         }
         
         double cutoff_radPerSec = cutoffFrequencyHz * 2.0 * Math.PI;
         double RC = 1.0 / cutoff_radPerSec;
         double alpha = RC / (RC + dt);  // alpha decreases with increasing cutoff frequency
         
         if (alpha <= 0 || alpha >= 1.0 && dt != 0.0)
         {
            throw new RuntimeException("Alpha value must be between 0 and 1.  Alpha = " + alpha);
         }
         
         return alpha;
      }
      
      public void reset()
      {
         hasBeenCalled = false;
      }
      
      public void setCutoffFrequencyHz(double cutoffHz)
      {
         this.cutoffFrequency_Hz.set(cutoffHz);
      }
      
      public void update(double filterInput)
      {
         if (!hasBeenCalled)
         {
            hasBeenCalled = true;
            
            filterInputOld = 0.0;
            filterUpdateTimeOld = 0.0;
            
            this.set(filterInput);
         }
         else
         {
            if ( yoTime != null )
            {
               double timeSinceLastUpdate = yoTime.getDoubleValue() - filterUpdateTimeOld;
               
               if (timeSinceLastUpdate > 0.0)
               {
                  dt = timeSinceLastUpdate;
               }
               else
               {
                  reset();
//                  throw new RuntimeException("Computed step size, DT must be greater than zero.   DT = " + dt + ".  Current time = " + yoTime.getDoubleValue() + ", previous update time = " + filterUpdateTimeOld);
               }
            }
            
            double filterOutput;

            switch (highOrLowPass)
            {
            case LOW_PASS:
               filterOutput = computeLowPassUpdate(filterInput, dt);
               break;
            case HIGH_PASS:
               filterOutput = computeHighPassUpdate(filterInput, dt);
               break;

            default:
               filterOutput = filterInput;
               break;
            }
            
            this.set(filterOutput);
         }
         
         filterInputOld = filterInput;
         
         if (yoTime != null)
         {
            filterUpdateTimeOld = yoTime.getDoubleValue();
         }
      }
   }