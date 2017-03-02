package us.ihmc.robotics.dataStructures.parameter;

public class IntegerParameter extends Parameter
{
   private int value;

   private IntegerParameter() // no-arg for serialization
   {
      super(null);
      this.value =  0;
   }

   IntegerParameter(String path, int defaultValue)
   {
      super(path);
      this.value = defaultValue;
   }

   public int get()
   {
      return value;
   }

   public void set(int value)
   {
      this.value = value;
      super.notifyChangeListeners();
   }

   @Override
   public boolean tryLoadValue(String value)
   {
      try
      {
         this.value = Integer.parseInt(value);
         return true;
      } catch (NumberFormatException e)
      {
         return false;
      }
   }

   @Override
   String dumpValue()
   {
      return Integer.toString(value);
   }
}

