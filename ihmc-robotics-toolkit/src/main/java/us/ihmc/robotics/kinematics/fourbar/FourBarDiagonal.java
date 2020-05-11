package us.ihmc.robotics.kinematics.fourbar;

public class FourBarDiagonal
{
   private final String name;

   private double length;
   private double lengthDot;
   private double lengthDDot;
   private double maxLength;
   private FourBarVertex start, end;
   private FourBarDiagonal other;

   FourBarDiagonal(String name)
   {
      this.name = name;
   }

   void setup(FourBarVertex start, FourBarVertex end, FourBarDiagonal other)
   {
      this.start = start;
      this.end = end;
      this.other = other;
   }

   protected void updateMaxLength()
   {
      FourBarTools.updateMaxLength(this);
   }

   public void setToNaN()
   {
      length = Double.NaN;
      lengthDot = Double.NaN;
      lengthDDot = Double.NaN;
      maxLength = Double.NaN;
   }

   public void setLength(double length)
   {
      this.length = length;
   }

   public void setLengthDot(double lengthDot)
   {
      this.lengthDot = lengthDot;
   }

   public void setLengthDDot(double lengthDDot)
   {
      this.lengthDDot = lengthDDot;
   }

   public void setMaxLength(double maxLength)
   {
      this.maxLength = maxLength;
   }

   public String getName()
   {
      return name;
   }

   public double getLength()
   {
      return length;
   }

   public double getLengthDot()
   {
      return lengthDot;
   }

   public double getLengthDDot()
   {
      return lengthDDot;
   }

   public double getMaxLength()
   {
      if (Double.isNaN(maxLength))
         updateMaxLength();
      return maxLength;
   }

   public FourBarVertex getStart()
   {
      return start;
   }

   public FourBarVertex getEnd()
   {
      return end;
   }

   public FourBarDiagonal getOther()
   {
      return other;
   }

   @Override
   public String toString()
   {
      return getName();
   }
}