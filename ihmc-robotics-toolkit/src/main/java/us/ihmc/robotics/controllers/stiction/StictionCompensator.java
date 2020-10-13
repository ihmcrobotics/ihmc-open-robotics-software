package us.ihmc.robotics.controllers.stiction;

import us.ihmc.robotics.math.filters.GlitchFilteredYoInteger;
import us.ihmc.robotics.math.filters.RateLimitedYoVariable;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class StictionCompensator
{
   private final DoubleProvider desiredTorqueStictionLimitFactor;

   private final YoDouble stictionCompensationLimit;
   private final DoubleProvider movingVelocityThreshold;
   private final DoubleProvider movingAccelerationThreshold;
   private final DoubleProvider acceleratingThreshold;
   private final DoubleProvider stictionCompensationRate;
   private final RateLimitedYoVariable stictionCompensation;

   private final StictionModel stictionModel;

   private double currentPosition, desiredPosition;
   private double currentVelocity, desiredVelocity;
   private double desiredAcceleration;
   private double desiredTorque;

   enum StictionActionMode
   {
      Moving(0), Accelerating(1), Braking(2), Stopped(3);

      private final int index;

      private StictionActionMode(int index)
      {
         this.index = index;
      }

      public static StictionActionMode[] values = values();

      public static StictionActionMode getMode(int index)
      {
         return values[index];
      }

      public int index()
      {
         return index;
      }

      public boolean active()
      {
         switch (this)
         {
         case Moving:
         case Accelerating:
            return true;
         default:
            return false;
         }
      }
   }

   private final YoEnum<StictionActionMode> stictionActionMode;
   private final GlitchFilteredYoInteger stictionActionModeIndex;
   private final YoInteger windowSize;
   private final DoubleProvider minTimeInMode;
   private final YoDouble timeInCurrentMode;

   private final double controlDt;

   public StictionCompensator(String prefix, StictionModel stictionModel, double controlDT, YoRegistry parentRegistry)
   {
      this.stictionModel = stictionModel;
      this.controlDt = controlDT;

      YoRegistry registry = new YoRegistry(prefix + getClass().getSimpleName());

      movingVelocityThreshold = new DoubleParameter(prefix + "_MovingVelocityThreshold", registry, 1e-2);
      movingAccelerationThreshold = new DoubleParameter(prefix + "_MovingAccelerationThreshold", registry, 1e-1);
      acceleratingThreshold = new DoubleParameter(prefix + "_AcceleratingThreshold", registry, 1);
      desiredTorqueStictionLimitFactor = new DoubleParameter(prefix + "_DesiredTorqueStictionLimitFactor", registry, 2.0);

      stictionCompensationLimit = new YoDouble(prefix + "_StictionCompensationLimit", registry);
      stictionCompensationRate = new DoubleParameter(prefix + "_StictionCompensationRate", registry, 10.0);
      stictionCompensation = new RateLimitedYoVariable(prefix + "_StictionCompensation", registry, stictionCompensationRate, controlDT);

      stictionActionMode = new YoEnum<>(prefix + "_StictionActionMode", registry, StictionActionMode.class);
      windowSize = new YoInteger(prefix + "_StictionActionModeWindowSize", registry);
      stictionActionModeIndex = new GlitchFilteredYoInteger(prefix + "_StictionActionModeIndex", windowSize, registry);
      minTimeInMode = new DoubleParameter(prefix + "_MinTimeInMode", registry, 5e-2);
      timeInCurrentMode = new YoDouble(prefix + "_TimeInCurrentMode", registry);


      stictionActionModeIndex.set(StictionActionMode.Stopped.index());
      stictionActionMode.set(StictionActionMode.Stopped);

      parentRegistry.addChild(registry);
   }

   public void setPositions(double currentPosition, double desiredPosition)
   {
      this.currentPosition = currentPosition;
      this.desiredPosition = desiredPosition;
   }

   public void setVelocities(double currentVelocity, double desiredVelocity)
   {
      this.currentVelocity = currentVelocity;
      this.desiredVelocity = desiredVelocity;
   }

   public void setDesiredAcceleration(double desiredAcceleration)
   {
      this.desiredAcceleration = desiredAcceleration;
   }

   public void setDesiredTorque(double desiredTorque)
   {
      this.desiredTorque = desiredTorque;
   }

   public void resetStictionCompensation()
   {
      this.stictionCompensationLimit.set(0.0);
      this.stictionCompensation.set(0.0);
   }

   public double computeStictionCompensation()
   {
      updateActionMode();

      if (stictionActionMode.getEnumValue().active())
      {
         double torqueSign = Math.signum(desiredTorque);

         double stictionMagnitude = stictionModel.getStictionMagnitude();
         stictionCompensationLimit.set(Math.min(torqueSign * desiredTorque * desiredTorqueStictionLimitFactor.getValue(), stictionMagnitude));
         stictionCompensation.update(torqueSign * stictionCompensationLimit.getDoubleValue());
      }
      else
      {
         stictionCompensationLimit.set(0.0);
         stictionCompensation.update(stictionCompensationLimit.getDoubleValue());
      }

      return stictionCompensation.getDoubleValue();
   }

   private void updateActionMode()
   {
      timeInCurrentMode.add(controlDt);
      updateWindowSize();

      StictionActionMode estimatedCurrentMode = estimateCurrentActionMode();
      if (timeInCurrentMode.getDoubleValue() > minTimeInMode.getValue() && estimatedCurrentMode != stictionActionMode.getEnumValue())
      {
         stictionActionMode.set(estimatedCurrentMode);
         timeInCurrentMode.set(0.0);
      }
   }

   private void updateWindowSize()
   {
      windowSize.set((int) (0.25 * minTimeInMode.getValue() / controlDt));
   }

   private StictionActionMode estimateCurrentActionMode()
   {
      stictionActionModeIndex.update(estimateCurrentActionModeIndex());
      return StictionActionMode.getMode(stictionActionModeIndex.getValue());
   }

   private int estimateCurrentActionModeIndex()
   {
      if (Math.abs(desiredVelocity) < movingVelocityThreshold.getValue() && Math.abs(desiredAcceleration) < movingAccelerationThreshold.getValue())
         return StictionActionMode.Stopped.index();

      if (Math.abs(desiredAcceleration) > acceleratingThreshold.getValue())
      {
         double velocitySign = Math.signum(desiredVelocity);
         double accelerationSign = Math.signum(desiredAcceleration);

         if (velocitySign * accelerationSign > 0)
            return StictionActionMode.Accelerating.index();
         else
            return StictionActionMode.Braking.index();
      }

      return StictionActionMode.Moving.index();
   }

   public double getStictionCompensation()
   {
      return stictionCompensation.getDoubleValue();
   }

   // for testing
   public double getDesiredTorque()
   {
      return desiredTorque;
   }
}
