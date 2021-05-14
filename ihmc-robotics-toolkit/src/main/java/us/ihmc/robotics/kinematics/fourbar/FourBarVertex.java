package us.ihmc.robotics.kinematics.fourbar;

import java.util.Objects;

/**
 * Represents one of the four vertices of a four bar linkage.
 */
public class FourBarVertex
{
   private final String name;
   private final FourBarAngle fourBarAngle;

   private double angle;
   private double angleDot;
   private double angleDDot;

   protected double minAngle;
   protected double maxAngle;

   private boolean convex = true;

   private FourBarEdge nextEdge, previousEdge;
   private FourBarDiagonal diagonal;

   /**
    * Creates a new vertex given a human readable name.
    * 
    * @param name         the name of this vertex.
    * @param fourBarAngle the angle in the four bar which this vertex represents.
    */
   FourBarVertex(String name, FourBarAngle fourBarAngle)
   {
      this.name = name;
      this.fourBarAngle = fourBarAngle;
   }

   /**
    * Initializes the data structure of the four bar linkage.
    * 
    * @param previousEdge the previous edge, i.e. the edge that ends at this vertex.
    * @param nextEdge     the next edge, i.e. the edge that starts at this vertex.
    * @param diagonal     the diagonal that starts or ends at this vertex.
    */
   void setup(FourBarEdge previousEdge, FourBarEdge nextEdge, FourBarDiagonal diagonal)
   {
      this.nextEdge = nextEdge;
      this.previousEdge = previousEdge;
      this.diagonal = diagonal;
   }

   /**
    * Checks that the references to the other four bar elements have been set and that the
    * configuration is consistent.
    * 
    * @throws NullPointerException  if any of the references has not been set.
    * @throws IllegalStateException if a problem in the configuration is detected.
    */
   void checkProperlySetup()
   {
      Objects.requireNonNull(nextEdge);
      Objects.requireNonNull(previousEdge);
      Objects.requireNonNull(diagonal);

      if (nextEdge.getStart() != this)
         throw new IllegalStateException("Improper configuration of the four bar.");
      if (previousEdge.getEnd() != this)
         throw new IllegalStateException("Improper configuration of the four bar.");
      if (diagonal.getStart() != this && diagonal.getEnd() != this)
         throw new IllegalStateException("Improper configuration of the four bar.");
   }

   /**
    * Computes and update the lower and upper limits at this vertex.
    */
   protected void updateLimits()
   {
      FourBarTools.updateLimits(this);
   }

   /**
    * Clears the internal state.
    */
   public void setToNaN()
   {
      angle = Double.NaN;
      angleDot = Double.NaN;
      angleDDot = Double.NaN;
      minAngle = Double.NaN;
      maxAngle = Double.NaN;
   }

   /**
    * Sets the inner angle at this vertex to {@link #getMinAngle()} and zeroes the velocity and
    * acceleration.
    */
   public void setToMin()
   {
      angle = getMinAngle();
      angleDot = 0.0;
      angleDDot = 0.0;
   }

   /**
    * Sets the inner angle at this vertex to {@link #getMaxAngle()} and zeroes the velocity and
    * acceleration.
    */
   public void setToMax()
   {
      angle = getMaxAngle();
      angleDot = 0.0;
      angleDDot = 0.0;
   }

   /**
    * Sets the inner angle at this vertex.
    * 
    * @param angle the new angle.
    */
   void setAngle(double angle)
   {
      this.angle = angle;
   }

   /**
    * Sets the value of the first time-derivative of the inner angle at this vertex.
    * 
    * @param angle the new angle first time-derivative.
    */
   void setAngleDot(double angleDot)
   {
      this.angleDot = angleDot;
   }

   /**
    * Sets the value of the second time-derivative of the inner angle at this vertex.
    * 
    * @param angle the new angle second time-derivative.
    */
   void setAngleDDot(double angleDDot)
   {
      this.angleDDot = angleDDot;
   }

   /**
    * Sets the lower bound for the inner angle at this vertex.
    * 
    * @param minAngle the minimum angle.
    */
   void setMinAngle(double minAngle)
   {
      this.minAngle = minAngle;
   }

   /**
    * Sets the upper bound for the inner angle at this vertex.
    * 
    * @param maxAngle the maximum angle.
    */
   void setMaxAngle(double maxAngle)
   {
      this.maxAngle = maxAngle;
   }

   /**
    * Indicates whether the four bar linkage is concave at this vertex.
    * 
    * @param convex whether the four bar linkage is convex at this vertex.
    */
   void setConvex(boolean convex)
   {
      this.convex = convex;
   }

   /**
    * Gets the short name for this vertex.
    * 
    * @return this vertex's name.
    */
   public String getName()
   {
      return name;
   }

   /**
    * Gets the angle in the four bar which this vertex holds onto.
    * 
    * @return the angle in the four bar which this vertex holds onto.
    */
   public FourBarAngle getFourBarAngle()
   {
      return fourBarAngle;
   }

   /**
    * Gets the current inner angle at this vertex or {@link Double#NaN} if it has not been computed
    * yet.
    * <p>
    * Note: the sign of the angle depends on convex property of this vertex:
    * <ul>
    * <li>if {@code this.isConvex() == true}: the angle is positive and defined in [0; <i>pi</i>].
    * <li>if {@code this.isConvex() == false}: the angle is negative and defined in [-<i>pi</i>; 0].
    * </ul>
    * </p>
    * 
    * @return the inner angle at this vertex.
    */
   public double getAngle()
   {
      return angle;
   }

   /**
    * Gets the current value for the first time-derivative of the inner angle at this vertex or
    * {@link Double#NaN} if it has not been computed yet.
    * 
    * @return the value for the first time-derivative of the inner angle at this vertex.
    */
   public double getAngleDot()
   {
      return angleDot;
   }

   /**
    * Gets the current value for the second time-derivative of the inner angle at this vertex or
    * {@link Double#NaN} if it has not been computed yet.
    * 
    * @return the value for the second time-derivative of the inner angle at this vertex.
    */
   public double getAngleDDot()
   {
      return angleDDot;
   }

   /**
    * Gets the lower bound for the inner angle at this vertex
    * 
    * @return the minimum angle for this vertex.
    */
   public double getMinAngle()
   {
      if (Double.isNaN(minAngle))
         updateLimits();
      return minAngle;
   }

   /**
    * Gets the upper bound for the inner angle at this vertex
    * 
    * @return the maximum angle for this vertex.
    */
   public double getMaxAngle()
   {
      if (Double.isNaN(maxAngle))
         updateLimits();
      return maxAngle;
   }

   /**
    * Indicates whether the four bar linkage is convex or concave at this vertex.
    * 
    * @return {@code true} if the four bar linkage is convex at this vertex, {@code false} otherwise.
    */
   public boolean isConvex()
   {
      return convex;
   }

   /**
    * Gets the next edge, i.e. the edge that starts at this vertex.
    * 
    * @return the next edge.
    */
   public FourBarEdge getNextEdge()
   {
      return nextEdge;
   }

   /**
    * Gets the previous edge, i.e. the edge that ends at this vertex.
    * 
    * @return the next edge.
    */
   public FourBarEdge getPreviousEdge()
   {
      return previousEdge;
   }

   /**
    * Gets the diagonal that starts or ends at this vertex.
    * 
    * @return the diagonal associated with this vertex.
    */
   public FourBarDiagonal getDiagonal()
   {
      return diagonal;
   }

   /**
    * Quick access to get the end vertex of the next edge.
    * 
    * @return the next vertex.
    */
   public FourBarVertex getNextVertex()
   {
      return nextEdge.getEnd();
   }

   /**
    * Quick access to get the start vertex of the previous edge.
    * 
    * @return the previous vertex.
    */
   public FourBarVertex getPreviousVertex()
   {
      return previousEdge.getStart();
   }

   /**
    * Gets the vertex that is on the other end of the diagonal associated with this vertex.
    * 
    * @return the opposite vertex.
    */
   public FourBarVertex getOppositeVertex()
   {
      return nextEdge.getNext().getEnd();
   }

   @Override
   public String toString()
   {
      return String.format("%s: [angle=%f, angleDot=%f, angleDDot=%f, convex=%b]", getName(), getAngle(), getAngleDot(), getAngleDDot(), isConvex());
   }
}