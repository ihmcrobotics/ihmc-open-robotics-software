package us.ihmc.aware.config;

public class DoubleProperty extends Property
{
   private double value;

   protected DoubleProperty(String path, double defaultValue)
   {
      super(path);
      this.value = defaultValue;
   }

   public double get()
   {
      return value;
   }

   public void set(double value)
   {
      this.value = value;
      notifyChangedListeners();
   }

   @Override
   public String dump()
   {
      return getPath() + "=" + value;
   }
}

