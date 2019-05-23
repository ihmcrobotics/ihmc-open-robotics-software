package us.ihmc.tools.property;

public class DoubleStoredPropertyKey extends StoredPropertyKey<Double>
{
   public DoubleStoredPropertyKey(int id, String titleCasedName)
   {
      super(Double.class, id, titleCasedName);
   }
}
