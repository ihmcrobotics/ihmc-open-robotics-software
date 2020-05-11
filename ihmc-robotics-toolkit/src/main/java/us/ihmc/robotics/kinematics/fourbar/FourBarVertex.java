package us.ihmc.robotics.kinematics.fourbar;

public class FourBarVertex
{
   private final String name;

   private double angle;
   private double angleDot;
   private double angleDDot;

   protected double minAngle;
   protected double maxAngle;

   private boolean convex = true;

   private FourBarEdge nextEdge, previousEdge;
   private FourBarDiagonal diagonal;

   FourBarVertex(String name)
   {
      this.name = name;
   }

   void setup(FourBarEdge previousEdge, FourBarEdge nextEdge, FourBarDiagonal diagonal)
   {
      this.nextEdge = nextEdge;
      this.previousEdge = previousEdge;
      this.diagonal = diagonal;
   }

   protected void updateLimits()
   {
      FourBarTools.updateLimits(this);
   }

   public void setToNaN()
   {
      angle = Double.NaN;
      angleDot = Double.NaN;
      angleDDot = Double.NaN;
      minAngle = Double.NaN;
      maxAngle = Double.NaN;
   }

   public void setToMin()
   {
      angle = getMinAngle();
      angleDot = 0.0;
      angleDDot = 0.0;
   }

   public void setToMax()
   {
      angle = getMaxAngle();
      angleDot = 0.0;
      angleDDot = 0.0;
   }

   public void setAngle(double angle)
   {
      this.angle = angle;
   }

   public void setAngleDot(double angleDot)
   {
      this.angleDot = angleDot;
   }

   public void setAngleDDot(double angleDDot)
   {
      this.angleDDot = angleDDot;
   }

   public void setMinAngle(double minAngle)
   {
      this.minAngle = minAngle;
   }

   public void setMaxAngle(double maxAngle)
   {
      this.maxAngle = maxAngle;
   }

   void setConvex(boolean convex)
   {
      this.convex = convex;
   }

   public String getName()
   {
      return name;
   }

   public double getAngle()
   {
      return angle;
   }

   public double getAngleDot()
   {
      return angleDot;
   }

   public double getAngleDDot()
   {
      return angleDDot;
   }

   public double getMinAngle()
   {
      if (Double.isNaN(minAngle))
         updateLimits();
      return minAngle;
   }

   public double getMaxAngle()
   {
      if (Double.isNaN(maxAngle))
         updateLimits();
      return maxAngle;
   }

   public boolean isConvex()
   {
      return convex;
   }

   public FourBarEdge getNextEdge()
   {
      return nextEdge;
   }

   public FourBarEdge getPreviousEdge()
   {
      return previousEdge;
   }

   public FourBarDiagonal getDiagonal()
   {
      return diagonal;
   }

   public FourBarVertex getNextVertex()
   {
      return nextEdge.getEnd();
   }

   public FourBarVertex getPreviousVertex()
   {
      return previousEdge.getStart();
   }

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