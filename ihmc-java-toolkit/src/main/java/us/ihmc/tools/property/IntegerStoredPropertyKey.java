package us.ihmc.tools.property;

public class IntegerStoredPropertyKey extends StoredPropertyKey<Integer>
{
   private int lowerBound = Integer.MIN_VALUE;
   private int upperBound = Integer.MAX_VALUE;
   private int[] validValues = null;

   public IntegerStoredPropertyKey(int id, String titleCasedName)
   {
      super(Integer.class, id, titleCasedName);
   }

   public IntegerStoredPropertyKey(int id, String titleCasedName, int defaultValue)
   {
      super(Integer.class, id, titleCasedName, defaultValue);
   }

   public void setValidValues(int[] validValues)
   {
      this.validValues = validValues;
   }

   public int[] getValidValues()
   {
      return validValues;
   }

   public void setLowerBound(int lowerBound)
   {
      this.lowerBound = lowerBound;
   }

   public int getLowerBound()
   {
      return lowerBound;
   }

   public void setUpperBound(int upperBound)
   {
      this.upperBound = upperBound;
   }

   public int getUpperBound()
   {
      return upperBound;
   }

   public boolean hasLowerBound()
   {
      return lowerBound != Integer.MIN_VALUE;
   }

   public boolean hasUpperBound()
   {
      return upperBound != Integer.MAX_VALUE;
   }

   public boolean hasSpecifiedValidValues()
   {
      return validValues != null;
   }
}
