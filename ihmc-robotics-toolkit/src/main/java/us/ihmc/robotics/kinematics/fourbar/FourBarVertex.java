package us.ihmc.robotics.kinematics.fourbar;

public class FourBarVertex
{
   private final String name;

   private double angle;
   private double angleDot;
   private double angleDDot;

   private double minAngle;
   private double maxAngle;

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
      angle = 0.0;
      angleDDot = 0.0;
   }

   public void setToMax()
   {
      angle = getMaxAngle();
      angle = 0.0;
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
         FourBarTools.updateLimits(this);
      return minAngle;
   }

   public double getMaxAngle()
   {
      if (Double.isNaN(maxAngle))
         FourBarTools.updateLimits(this);
      return maxAngle;
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
      return getName();
   }
}