package us.ihmc.humanoidRobotics.footstep.footsepGenerator;

import java.util.List;

public abstract class FootstepPlanningParameterization
{

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

   protected List<FootstepOffset> offsetList;
   public double footWidth;
   public double footLength;
   public double cropWidth;
   public double cropLength;
   public double maxSupportPolygonArea;
   public double minSupportPolygonArea;
   public double xAnkleOffset;
   public double yAnkleOffset;

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
}
