package us.ihmc.robotics.kinematics.fourbar;

public class FourBarEdge
{
   private final String name;

   private double length;
   private FourBarVertex start, end;
   private FourBarEdge nextEdge, previous;

   FourBarEdge(String name)
   {
      this.name = name;
   }

   void setup(FourBarVertex start, FourBarVertex end, FourBarEdge previous, FourBarEdge next)
   {
      this.start = start;
      this.end = end;
      this.previous = previous;
      this.nextEdge = next;
   }

   public void setToNaN()
   {
      length = Double.NaN;
   }

   void setLength(double length)
   {
      this.length = length;
   }

   public String getName()
   {
      return name;
   }

   public double getLength()
   {
      return length;
   }

   public FourBarVertex getStart()
   {
      return start;
   }

   public FourBarVertex getEnd()
   {
      return end;
   }

   public FourBarEdge getNext()
   {
      return nextEdge;
   }

   public FourBarEdge getPrevious()
   {
      return previous;
   }

   @Override
   public String toString()
   {
      return getName();
   }
}