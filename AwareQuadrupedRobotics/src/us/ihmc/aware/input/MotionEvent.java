package us.ihmc.aware.input;

public class MotionEvent
{
   private final InputDeviceAxis axis;
   private final double value;

   public MotionEvent(InputDeviceAxis axis, double value)
   {
      this.axis = axis;
      this.value = value;
   }

   public InputDeviceAxis getAxis()
   {
      return axis;
   }

   public double getValue()
   {
      return value;
   }
}
