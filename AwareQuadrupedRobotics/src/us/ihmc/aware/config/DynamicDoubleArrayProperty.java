package us.ihmc.aware.config;

public class DynamicDoubleArrayProperty extends DynamicProperty
{
   private double[] value;

   public DynamicDoubleArrayProperty(DynamicPropertyRegistry registry, String path, double... defaultValue)
   {
      super(registry, path);
      this.value = defaultValue;
   }

   public void set(double[] value)
   {
      this.value = value;
   }

   public void set(int idx, double value)
   {
      this.value[idx] = value;
   }

   @Override
   public String dump()
   {
      return null;
   }
}
