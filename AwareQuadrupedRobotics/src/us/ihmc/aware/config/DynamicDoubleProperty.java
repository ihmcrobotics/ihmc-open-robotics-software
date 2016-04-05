package us.ihmc.aware.config;

public class DynamicDoubleProperty extends DynamicProperty
{
   private double value;

   public DynamicDoubleProperty(DynamicPropertyRegistry registry, String path, double defaultValue)
   {
      super(registry, path);
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

