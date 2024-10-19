package us.ihmc.yoVariables.filters;

import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This is a yo variable whose rate of change is clamped to some maximum value. To use it, either create this variable passing in the non-limited yo variable to
 * track and then call {@link #update()}, or unlink it keep it unlinked from a specific yo variable and call {@link #update(double)}. This variable when then
 * track the specified variable or input, but will not change faster than the rate provided by {@code maxRateVariable}.
 */
public class RateLimitedYoVariable extends YoDouble
{
   private final DoubleProvider maxRateVariable;

   private final DoubleProvider unlimitedPosition;
   private final YoBoolean limited;

   private final double dt;

   private final YoBoolean hasBeenCalled;

   /**
    * Constructs this variable with no double to track. The value contained in this yo variable will track the desired reference, but will limit the maximum
    * rate of change to {@code maxRate}.
    * <p>
    * The maximum rate of change is enforced as a maximum step size every time {@link #update(double)} is called. The maximum step size
    * can be calculated as {@code maxRate} * {@code dt}.
    * </p>
    * <p>
    * To use this variable after using this constructor, you must call {@link #update(double)}.  Calling {@link #update()} will result in a null pointer
    * exception.
    * </p>
    * <p>
    * A known edge case is if {@link #update(double)} is called more than once per control update. In this case, the maximum rate is enforced each time
    * {@link #update()}  called, rather than per each control update. Avoid doing this.
    * </p>
    *
    * @param name     name of this variable.
    * @param registry registry to add this variable to.
    * @param maxRate  maximum rate of change this value can experience every time {@link #update(double)} is called.
    * @param dt       expected time change since between calls of {@link #update(double)}.
    */
   public RateLimitedYoVariable(String name, YoRegistry registry, double maxRate, double dt)
   {
      this(name, registry, maxRate, null, dt);
   }

   /**
    * Constructs this variable with no double to track. The value contained in this yo variable will track the desired reference, but will limit the maximum
    * rate of change to {@code maxRateVariable}.
    * <p>
    * The maximum rate of change is enforced as a maximum step size every time  {@link #update(double)} is called. The maximum step size
    * can be calculated as {@code maxRateVariable} * {@code dt}.
    * </p>
    * <p>
    * To use this variable after using this constructor, you must call {@link #update(double)}.  Calling {@link #update()} will result in a null pointer
    * exception.
    * </p>
    * <p>
    * A known edge case is if {@link #update(double)} is called more than once per control update. In this case, the maximum rate is enforced each time
    * {@link #update()}  called, rather than per each control update. Avoid doing this.
    * </p>
    *
    * @param name            name of this variable.
    * @param registry        registry to add this variable to.
    * @param maxRateVariable maximum rate of change this value can experience every time {@link #update(double)} is called.
    * @param dt              expected time change since between calls of {@link #update(double)}.
    */
   public RateLimitedYoVariable(String name, YoRegistry registry, DoubleProvider maxRateVariable, double dt)
   {
      this(name, registry, maxRateVariable, null, dt);
   }

   /**
    * Constructs this variable to track {@code positionVariable}. The value contained in this yo variable will track the desired reference passed in at
    * construction, but will limit the maximum
    * rate of change to {@code maxRate}.
    * <p>
    * The maximum rate of change is enforced as a maximum step size every time {@link #update()} is called. The maximum step size
    * can be calculated as {@code maxRate} * {@code dt}.
    * </p>
    * <p>
    * To use this variable after using this constructor, you should call {@link #update()}.  Calling {@link #update(double)} will not track the variable
    * provided
    * by {@code positionVariable}.
    * </p>
    * <p>
    * A known edge case is if {@link #update()} is called more than once per control update. In this case, the maximum rate is enforced each time
    * {@link #update()}  called, rather than per each control update. Avoid doing this.
    * </p>
    *
    * @param name             name of this variable.
    * @param registry         registry to add this variable to.
    * @param maxRate          maximum rate of change this value can experience every time {@link #update()} is called.
    * @param positionVariable varible provider that this variable will track.
    * @param dt               expected time change since between calls of {@link #update()}.
    */
   public RateLimitedYoVariable(String name, YoRegistry registry, double maxRate, DoubleProvider positionVariable, double dt)
   {
      this(name, registry, VariableTools.createMaxRateYoDouble(name, "", maxRate, registry), positionVariable, dt);
   }

   /**
    * Constructs this variable to track {@code positionVariable}. The value contained in this yo variable will track the desired reference passed in at
    * construction, but will limit the maximum
    * rate of change to {@code maxRateVariable}.
    * <p>
    * The maximum rate of change is enforced as a maximum step size every time {@link #update()} is called. The maximum step size
    * can be calculated as {@code maxRateVariable} * {@code dt}.
    * </p>
    * <p>
    * To use this variable after using this constructor, you should call {@link #update()}.  Calling {@link #update(double)} will not track the variable
    * provided
    * by {@code positionVariable}.
    * </p>
    * <p>
    * A known edge case is if {@link #update()} is called more than once per control update. In this case, the maximum rate is enforced each time
    * {@link #update()}  called, rather than per each control update. Avoid doing this.
    * </p>
    *
    * @param name              name of this variable.
    * @param registry          registry to add this variable to.
    * @param maxRateVariable   maximum rate of change this value can experience every time {@link #update()} is called.
    * @param unlimitedPosition varible provider that this variable will track.
    * @param dt                expected time change since between calls of {@link #update()}.
    */
   public RateLimitedYoVariable(String name, YoRegistry registry, DoubleProvider maxRateVariable, DoubleProvider unlimitedPosition, double dt)
   {
      super(name, registry);

      this.hasBeenCalled = VariableTools.createHasBeenCalledYoBoolean(name, "", registry);
      this.limited = VariableTools.createLimitedCalledYoBoolean(name, "", registry);

      this.unlimitedPosition = unlimitedPosition;
      this.maxRateVariable = maxRateVariable;

      this.dt = dt;

      reset();
   }

   /**
    * Resets this variable. On the next time {@link #update()} or {@link #update(double)} is called, it will automatically be set to the variable to track,
    * rather than experiencing any rate limiting.
    */
   public void reset()
   {
      hasBeenCalled.set(false);
   }

   /**
    * Updates the value contained in this yo variable to track the value contained in {@code unlimitedPosition}. If the {@code unlimitedPosition} can be
    * achieved with a rate of change
    * less that provided by {@code maxRateVariable}, then the value stored in this variable and returned by {@link #getValue()} will match
    * {@code unlimitedPosition}.
    * Otherwise, it will step towards the new position at maximum rate. This can be computed using the following pseudo-code:
    * <p>
    * <li>error = currentPosition - getValue()</li>
    * <li>if abs(error) < maxRate * dt</li>
    * <li>  set(currentPosition)</li>
    * <li>else</li>
    * <li>  set(currentPosition + sign(error) * maxRate * dt)</li>
    * </p>
    *
    * @throws NullPointerException if this class was constructed with no {@code unlimitedPosition} variable to track.
    */
   public void update()
   {
      if (unlimitedPosition == null)
      {
         throw new NullPointerException(
               getClass().getSimpleName() + " must be constructed with a non null " + "position variable to call update(), otherwise use update(double)");
      }

      update(unlimitedPosition.getValue());
   }

   /**
    * Updates the value contained in this yo variable to track {@code currentPosition}. If the {@code currentPosition} can be achieved with a rate of change
    * less that provided by {@code maxRateVariable}, then the value stored in this variable and returned by {@link #getValue()} will match
    * {@code currentPosition}.
    * Otherwise, it will step towards the new position at maximum rate. This can be computed using the following pseudo-code:
    * <p>
    * <li>error = currentPosition - getValue()</li>
    * <li>if abs(error) < maxRate * dt</li>
    * <li>  set(currentPosition)</li>
    * <li>else</li>
    * <li>  set(currentPosition + sign(error) * maxRate * dt)</li>
    * </p>
    *
    * @param currentPosition position to try and achieve within rate limits.
    */
   public void update(double currentPosition)
   {
      if (!hasBeenCalled.getBooleanValue())
      {
         hasBeenCalled.set(true);
         set(currentPosition);
      }

      if (maxRateVariable.getValue() < 0)
         throw new RuntimeException("The maxRate parameter in the RateLimitedYoVariable cannot be negative.");

      double difference = currentPosition - getDoubleValue();
      if (Math.abs(difference) > maxRateVariable.getValue() * dt)
      {
         difference = Math.signum(difference) * maxRateVariable.getValue() * dt;
         this.limited.set(true);
      }
      else
         this.limited.set(false);

      set(getDoubleValue() + difference);
   }
}