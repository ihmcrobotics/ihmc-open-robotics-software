package us.ihmc.robotics.kinematics.fourbar;

import java.util.Objects;

/**
 * Represents one of the four sides of a four bar linkage.
 * <p>
 * Each side is assumed to have a fixed length.
 * </p>
 */
public class FourBarEdge
{
   private final String name;

   private double length;

   private FourBarVertex start, end;
   private FourBarEdge next, previous;

   /**
    * Creates a new edge given a human readable name.
    * 
    * @param name the name of this edge.
    */
   FourBarEdge(String name)
   {
      this.name = name;
   }

   /**
    * Initializes the data structure of the four bar linkage.
    * 
    * @param start    the vertex from which this edge starts.
    * @param end      the vertex at which this edge ends.
    * @param previous the previous edge, i.e. before {@code start}.
    * @param next     the next edge, i.e. after {@code next}.
    */
   void setup(FourBarVertex start, FourBarVertex end, FourBarEdge previous, FourBarEdge next)
   {
      this.start = start;
      this.end = end;
      this.previous = previous;
      this.next = next;
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
      Objects.requireNonNull(next);
      Objects.requireNonNull(previous);

      if (start.getNextEdge() != this)
         throw new IllegalStateException("Improper configuration of the four bar.");
      if (end.getPreviousEdge() != this)
         throw new IllegalStateException("Improper configuration of the four bar.");
      if (next.getPrevious() != this)
         throw new IllegalStateException("Improper configuration of the four bar.");
      if (previous.getNext() != this)
         throw new IllegalStateException("Improper configuration of the four bar.");
   }

   /**
    * Clears the internal state.
    */
   public void setToNaN()
   {
      length = Double.NaN;
   }

   /**
    * Sets the length of this edge.
    * 
    * @param length this edge's length.
    */
   void setLength(double length)
   {
      this.length = length;
   }

   /**
    * Gets the short name for this edge.
    * 
    * @return this edge's name.
    */
   public String getName()
   {
      return name;
   }

   /**
    * Gets this edge's length.
    * 
    * @return the length of this edge.
    */
   public double getLength()
   {
      return length;
   }

   /**
    * Gets whether this edge is flipped, i.e. its two vertices are concave.
    * 
    * @return {@code true}
    */
   public boolean isFlipped()
   {
      return !start.isConvex() && !end.isConvex();
   }

   /**
    * Gets whether this edge is crossing another edge of the four bar linkage.
    * 
    * @return {@code true} if this edge crosses another edge, {@code false} otherwise.
    */
   public boolean isCrossing()
   {
      return next.isFlipped() || previous.isFlipped();
   }

   /**
    * Gets the vertex from which this edge starts.
    * 
    * @return the start vertex.
    */
   public FourBarVertex getStart()
   {
      return start;
   }

   /**
    * Gets the vertex at which this edge ends.
    * 
    * @return the end vertex.
    */
   public FourBarVertex getEnd()
   {
      return end;
   }

   /**
    * Gets the next edge, i.e. the edge after the end vertex.
    * 
    * @return the next edge.
    */
   public FourBarEdge getNext()
   {
      return next;
   }

   /**
    * Gets the previous edge, i.e. the edge before the start vertex.
    * 
    * @return the previous edge.
    */
   public FourBarEdge getPrevious()
   {
      return previous;
   }

   @Override
   public String toString()
   {
      return String.format("%s: [length=%f, flipped=%b, crossing=%b]", getName(), getLength(), isFlipped(), isCrossing());
   }
}