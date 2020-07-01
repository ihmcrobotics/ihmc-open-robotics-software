package us.ihmc.robotics.kinematics.fourbar;

import java.util.Objects;

/**
 * Represents one of the two diagonals of a four bar linkage.
 * <p>
 * The length of a diagonal depends on the state of the four bar linkage.
 * </p>
 */
public class FourBarDiagonal
{
   private final String name;
   private double length;
   private double lengthDot;
   private double lengthDDot;
   private FourBarVertex start, end;
   private FourBarDiagonal other;

   /**
    * Creates a new diagonal given a human readable name.
    * 
    * @param name the name of this diagonal.
    */
   FourBarDiagonal(String name)
   {
      this.name = name;
   }

   /**
    * Initializes the data structure of the four bar linkage.
    * 
    * @param start the vertex from which this diagonal starts.
    * @param end   the vertex at which this diagonal ends.
    * @param other the other diagonal of the four bar linkage.
    */
   void setup(FourBarVertex start, FourBarVertex end, FourBarDiagonal other)
   {
      this.start = start;
      this.end = end;
      this.other = other;
   }

   /**
    * Checks that the references to the other four bar elements have been set and that the configuration is consistent.
    * 
    * @throws NullPointerException if any of the references has not been set.
    * @throws IllegalStateException if a problem in the configuration is detected.
    */
   void checkProperlySetup()
   {
      Objects.requireNonNull(start);
      Objects.requireNonNull(end);
      Objects.requireNonNull(other);

      if (start.getDiagonal() != this)
         throw new IllegalStateException("Improper configuration of the four bar.");
      if (end.getDiagonal() != this)
         throw new IllegalStateException("Improper configuration of the four bar.");
      if (other.getOther() != this)
         throw new IllegalStateException("Improper configuration of the four bar.");
   }

   /**
    * Clears the internal state.
    */
   public void setToNaN()
   {
      length = Double.NaN;
      lengthDot = Double.NaN;
      lengthDDot = Double.NaN;
   }

   /**
    * Sets the current length of this diagonal.
    * 
    * @param length the new diagonal length.
    */
   void setLength(double length)
   {
      this.length = length;
   }

   /**
    * Sets the first time-derivative of this diagonal's length.
    * 
    * @param lengthDot the rate of change of the length.
    */
   void setLengthDot(double lengthDot)
   {
      this.lengthDot = lengthDot;
   }

   /**
    * Sets the second time-derivative of this diagonal's length.
    * 
    * @param lengthDDot the length second time-derivative.
    */
   public void setLengthDDot(double lengthDDot)
   {
      this.lengthDDot = lengthDDot;
   }

   /**
    * Gets the short name for this diagonal.
    * 
    * @return this diagonal's name.
    */
   public String getName()
   {
      return name;
   }

   /**
    * Gets the current length of this diagonal or {@link Double#NaN} if it has not been computed yet.
    * 
    * @return the length of this diagonal.
    */
   public double getLength()
   {
      return length;
   }

   /**
    * Gets the current value of the first time-derivative of the length of this diagonal or
    * {@link Double#NaN} if it has not been computed yet.
    * 
    * @return the value of the first time-derivative of the length of this diagonal.
    */
   public double getLengthDot()
   {
      return lengthDot;
   }

   /**
    * Gets the current value of the second time-derivative of the length of this diagonal or
    * {@link Double#NaN} if it has not been computed yet.
    * 
    * @return the value of the second time-derivative of the length of this diagonal.
    */
   public double getLengthDDot()
   {
      return lengthDDot;
   }

   /**
    * Gets the vertex from which this diagonal starts.
    * 
    * @return the start vertex.
    */
   public FourBarVertex getStart()
   {
      return start;
   }

   /**
    * Gets the vertex at which this diagonal ends.
    * 
    * @return the end vertex.
    */
   public FourBarVertex getEnd()
   {
      return end;
   }

   /**
    * Gets the other diagonal of the four bar linkage.
    * 
    * @return the other diagonal.
    */
   public FourBarDiagonal getOther()
   {
      return other;
   }

   @Override
   public String toString()
   {
      return String.format("%s [length=%f, lengthDot=%f, lengthDDot=%f]", getName(), getLength(), getLengthDot(), getLengthDDot());
   }
}