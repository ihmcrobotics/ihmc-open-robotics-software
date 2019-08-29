package us.ihmc.tools.property;

public class DoubleStoredPropertyKey extends StoredPropertyKey<Double>
{
   public DoubleStoredPropertyKey(int id, String titleCasedName)
   {
      super(Double.class, id, titleCasedName);
   }

   public DoubleStoredPropertyKey(int id, String titleCasedName, double defaultValue)
   {
      super(Double.class, id, titleCasedName, defaultValue);
   }
}
