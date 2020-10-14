package us.ihmc.ihmcPerception.lineSegmentDetector;

public class LineMatch
{
   double[] prevLine, curLine;

   private double angleCost;
   private double lengthCost;
   private double midPointCost;

   LineMatch(double[] l1, double[] l2)
   {
      this.prevLine = l1;
      this.curLine = l2;
   }

   LineMatch(double[] l1, double[] l2, double angleCost, double midPointCost, double lengthCost)
   {
      this.prevLine = l1;
      this.curLine = l2;
      this.angleCost = angleCost;
      this.midPointCost = midPointCost;
      this.lengthCost = lengthCost;
   }
}
