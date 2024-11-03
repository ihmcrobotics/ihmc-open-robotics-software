package us.ihmc.robotics.stateMachine.core;

import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This defines the interface for the internal clock used in a {@link StateMachine}.
 * <p>
 * A clock is used to easily keep track of the time of the last state change and the time spent in a
 * state.
 * </p>
 * <p>
 * Three implementations are provided via state constructors:
 * <ul>
 * <li>{@link #dummyClock()}: Provides a default implementation when the absolute time information
 * is not available.
 * <li>{@link #clock(DoubleProvider)}: Provides a clock implementation backed by doubles.
 * <li>{@link #yoClock(DoubleProvider, String, YoRegistry)}: Provides a clock implementation
 * backed by {@code YoDouble}.
 * </ul>
 * </p>
 * 
 * @author Sylvain
 */
public interface StateMachineClock
{
   /**
    * Invoked by the state machine to notify that the active state is changing.
    */
   void notifyStateChanged();

   /**
    * Gets the absolute time value, or {@link Double#NaN} if time information is unavailable.
    * 
    * @return the absolute time.
    */
   double getTime();

   /**
    * Gets the time spent in the active, or {@link Double#NaN} if time information is unavailable.
    * 
    * @return the time spent in the current state.
    */
   double getTimeInCurrentState();

   /**
    * Gets the absolute time value when the last state change occurred, or {@link Double#NaN} if time
    * information is unavailable.
    * 
    * @return the time of the last state change.
    */
   double getTimeOfLastStateChange();

   /**
    * Dummy implementation of a clock for when the time information is unavailable.
    * 
    * @return the dummy clock.
    */
   public static StateMachineClock dummyClock()
   {
      return new StateMachineClock()
      {
         @Override
         public void notifyStateChanged()
         {
         }

         @Override
         public double getTime()
         {
            return Double.NaN;
         }

         @Override
         public double getTimeInCurrentState()
         {
            return Double.NaN;
         }

         @Override
         public double getTimeOfLastStateChange()
         {
            return Double.NaN;
         }
      };
   }

   /**
    * Clock implementation with time information backed by doubles.
    * 
    * @param timeProvider the variable used to obtain the time information.
    * @return the clock.
    */
   public static StateMachineClock clock(DoubleProvider timeProvider)
   {
      return new StateMachineClock()
      {
         double timeOfLastStateChange = Double.NaN;

         @Override
         public void notifyStateChanged()
         {
            timeOfLastStateChange = timeProvider.getValue();
         }

         @Override
         public double getTime()
         {
            return timeProvider.getValue();
         }

         @Override
         public double getTimeInCurrentState()
         {
            return timeProvider.getValue() - timeOfLastStateChange;
         }

         @Override
         public double getTimeOfLastStateChange()
         {
            return timeOfLastStateChange;
         }
      };
   }

   /**
    * Clock implementation with time information backed by {@link YoDouble}s.
    * 
    * @param timeProvider the variable used to obtain the time information.
    * @param namePrefix the prefix used to create the {@code YoDouble}s.
    * @param registry the registry to which the {@code YoDouble}s are registered.
    * @return the clock.
    */
   public static StateMachineClock yoClock(DoubleProvider timeProvider, String namePrefix, YoRegistry registry)
   {
      YoDouble timeOfLastStateChange = new YoDouble(namePrefix + "SwitchTime", "Time at which the last state change occured.", registry);
      YoDouble timeInCurrentState = new YoDouble(namePrefix + "StateTime", "Time relative to the start of the current state.", registry);
      timeOfLastStateChange.setToNaN();
      timeInCurrentState.setToNaN();

      return new StateMachineClock()
      {
         @Override
         public void notifyStateChanged()
         {
            timeOfLastStateChange.set(timeProvider.getValue());
         }

         @Override
         public double getTime()
         {
            return timeProvider.getValue();
         }

         @Override
         public double getTimeInCurrentState()
         {
            double t = timeProvider.getValue() - timeOfLastStateChange.getDoubleValue();
            timeInCurrentState.set(t);
            return t;
         }

         @Override
         public double getTimeOfLastStateChange()
         {
            return timeOfLastStateChange.getDoubleValue();
         }
      };
   }
}
