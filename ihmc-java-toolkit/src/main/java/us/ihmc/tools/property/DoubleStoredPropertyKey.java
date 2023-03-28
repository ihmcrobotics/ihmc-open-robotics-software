package us.ihmc.tools.property;

public class DoubleStoredPropertyKey extends StoredPropertyKey<Double>
{
   private double lowerBound = Double.NaN;
   private double upperBound = Double.NaN;

   public DoubleStoredPropertyKey(int id, String titleCasedName)
   {
      super(Double.class, id, titleCasedName);
   }

   public DoubleStoredPropertyKey(int id, String titleCasedName, double defaultValue)
   {
      super(Double.class, id, titleCasedName, defaultValue);
   }

   public void setLowerBound(double lowerBound)
   {
      this.lowerBound = lowerBound;
   }

   public double getLowerBound()
   {
      return lowerBound;
   }

   public void setUpperBound(double upperBound)
   {
      this.upperBound = upperBound;
   }

   public double getUpperBound()
   {
      return upperBound;
   }

   public boolean hasLowerBound()
   {
      return !Double.isNaN(lowerBound);
   }

   public boolean hasUpperBound()
   {
      return !Double.isNaN(upperBound);
   }
}
