package us.ihmc.robotics.dataStructures.parameter;

public class BooleanParameter extends Parameter
{
   private boolean value;

   private BooleanParameter() // no-arg for serialization
   {
      super(null);
      this.value = false;
   }

   BooleanParameter(String path, boolean defaultValue)
   {
      super(path);
      this.value = defaultValue;
   }

   public boolean get()
   {
      return value;
   }

   public void set(boolean value)
   {
      this.value = value;
      super.notifyChangeListeners();
   }

   @Override
   public boolean tryLoadValue(String value)
   {
      this.value = Boolean.parseBoolean(value);
      return true;
   }

   @Override
   String dumpValue()
   {
      return Boolean.toString(value);
   }
}

