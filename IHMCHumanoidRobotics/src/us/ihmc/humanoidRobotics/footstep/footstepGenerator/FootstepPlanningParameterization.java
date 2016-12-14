package us.ihmc.humanoidRobotics.footstep.footstepGenerator;

import java.util.List;

public abstract class FootstepPlanningParameterization
{
   protected List<FootstepOffset> offsetList;
   protected double footWidth;
   protected double footLength;
   protected double cropWidth;
   protected double cropLength;
   protected double maxSupportPolygonArea;
   protected double minSupportPolygonArea;
   protected double xAnkleOffset;
   protected double yAnkleOffset;
   
   //used for straight line footstep path generation in the UI (clicking in the window)
   //defaults work with atlas and val
   protected double straightstepLength = 0.4; 
   protected double straightStepWidth = 0.25;
   protected double reverseStepLength = 0.15;
   protected double reverseStepWidth = 0.25;
   protected double shuffleStepLength = 0.25;
   protected double shuffleStepWidth = 0.21;

   public List<FootstepOffset> getOffsets(FootstepPlanState currentState)
   {
      return offsetList;
   }

   public abstract FootstepOffset getSidestep(FootstepPlanState currentState);

   public abstract boolean withinReachForNext(double xdiff, double ydiff, double thetadiff);

   public abstract double getMaxStepUp();

   public abstract double getMinStepDown();

   public abstract double getMaxStepDistance();

   public abstract double getDangerDistance();
   
   public double getMaxSupportPolygonArea()
   {
      return maxSupportPolygonArea;
   }

   public class FootstepOffset
   {
      public double dx;
      public double dy;
      public double dtheta;
      
      public FootstepOffset(double dx, double dy, double dtheta)
      {
         this.dx = dx;
         this.dy = dy;
         this.dtheta = dtheta;
      }
   }

   public double getStraightstepLength()
   {
      return straightstepLength;
   }

   public double getStraightStepWidth()
   {
      return straightStepWidth;
   }

   public double getReverseStepLength()
   {
      return reverseStepLength;
   }

   public double getReverseStepWidth()
   {
      return reverseStepWidth;
   }

   public double getShuffleStepLength()
   {
      return shuffleStepLength;
   }

   public double getShuffleStepWidth()
   {
      return shuffleStepWidth;
   }

   public double getCropLength()
   {
      return cropLength;
   }

   public double getCropWidth()
   {
      return cropWidth;
   }

   public double getFootLength()
   {
      return footLength;
   }

   public void setFootLength(double footLength)
   {
      this.footLength = footLength;
   }

   public double getFootWidth()
   {
      return footWidth;
   }

   public void setFootWidth(double footWidth)
   {
      this.footWidth = footWidth;
   }

   public double getMinSupportPolygonArea()
   {
      return minSupportPolygonArea;
   }

   public void setMinSupportPolygonArea(double minSupportPolygonArea)
   {
      this.minSupportPolygonArea = minSupportPolygonArea;
   }
}
