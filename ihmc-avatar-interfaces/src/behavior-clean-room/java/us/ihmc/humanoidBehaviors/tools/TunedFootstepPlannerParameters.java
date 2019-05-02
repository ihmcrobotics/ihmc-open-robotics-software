package us.ihmc.humanoidBehaviors.tools;

import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.SettableFootstepPlannerParameters;

public class TunedFootstepPlannerParameters
{
   private double cliffClearance    ;
   private double cliffHeight       ;
   private double maxStepLength     ;
   private double maxStepWidth      ;
   private double maxStepYaw        ;
   private double maxStepZ          ;
   private double maxXYWiggle       ;
   private double maxYawWiggle      ;
   private double minFootholdPercent;
   private double minStepLength     ;
   private double minStepWidth      ;
   private double minStepYaw        ;
   private double minSurfaceIncline ;
   private double minXClearance     ;
   private double minYClearance     ;
   private double wiggleInsideDelta ;

   public void setFromFootstepPlannerParameters(FootstepPlannerParameters footstepPlannerParameters)
   {
      cliffClearance      = footstepPlannerParameters.getMinimumDistanceFromCliffBottoms();
      cliffHeight         = footstepPlannerParameters.getCliffHeightToAvoid()             ;
      maxStepLength       = footstepPlannerParameters.getMaximumStepReach()               ;
      maxStepWidth        = footstepPlannerParameters.getMaximumStepWidth()               ;
      maxStepYaw          = footstepPlannerParameters.getMaximumStepYaw()                 ;
      maxStepZ            = footstepPlannerParameters.getMaximumStepZ()                   ;
      maxXYWiggle         = footstepPlannerParameters.getMaximumXYWiggleDistance()        ;
      maxYawWiggle        = footstepPlannerParameters.getMaximumYawWiggle()               ;
      minFootholdPercent  = footstepPlannerParameters.getMinimumFootholdPercent()         ;
      minStepLength       = footstepPlannerParameters.getMinimumStepLength()              ;
      minStepWidth        = footstepPlannerParameters.getMinimumStepWidth()               ;
      minStepYaw          = footstepPlannerParameters.getMinimumStepYaw()                 ;
      minSurfaceIncline   = footstepPlannerParameters.getMinimumSurfaceInclineRadians()   ;
      minXClearance       = footstepPlannerParameters.getMinXClearanceFromStance()        ;
      minYClearance       = footstepPlannerParameters.getMinYClearanceFromStance()        ;
      wiggleInsideDelta   = footstepPlannerParameters.getWiggleInsideDelta()              ;
   }

   public void packFootstepPlannerParameters(SettableFootstepPlannerParameters footstepPlannerParameters)
   {
      footstepPlannerParameters.setMinimumDistanceFromCliffBottoms(       cliffClearance      );
      footstepPlannerParameters.setCliffHeightToAvoid(                    cliffHeight         );
      footstepPlannerParameters.setMaximumStepReach(                      maxStepLength       );
      footstepPlannerParameters.setMaximumStepWidth(                      maxStepWidth        );
      footstepPlannerParameters.setMaximumStepYaw(                        maxStepYaw          );
      footstepPlannerParameters.setMaximumStepZ(                          maxStepZ            );
      footstepPlannerParameters.setMaximumXYWiggleDistance(               maxXYWiggle         );
      footstepPlannerParameters.setMaximumYawWiggle(                      maxYawWiggle        );
      footstepPlannerParameters.setMinimumFootholdPercent(                minFootholdPercent  );
      footstepPlannerParameters.setMinimumStepLength(                     minStepLength       );
      footstepPlannerParameters.setMinimumStepWidth(                      minStepWidth        );
      footstepPlannerParameters.setMinimumStepYaw(                        minStepYaw          );
      footstepPlannerParameters.setMinimumSurfaceInclineRadians(          minSurfaceIncline   );
      footstepPlannerParameters.setMinXClearanceFromStance(               minXClearance       );
      footstepPlannerParameters.setMinYClearanceFromStance(               minYClearance       );
      footstepPlannerParameters.setWiggleInsideDelta(                     wiggleInsideDelta   );
   }

   public double getCliffClearance()
   {
      return cliffClearance;
   }

   public void setCliffClearance(double cliffClearance)
   {
      this.cliffClearance = cliffClearance;
   }

   public double getCliffHeight()
   {
      return cliffHeight;
   }

   public void setCliffHeight(double cliffHeight)
   {
      this.cliffHeight = cliffHeight;
   }

   public double getMaxStepLength()
   {
      return maxStepLength;
   }

   public void setMaxStepLength(double maxStepLength)
   {
      this.maxStepLength = maxStepLength;
   }

   public double getMaxStepWidth()
   {
      return maxStepWidth;
   }

   public void setMaxStepWidth(double maxStepWidth)
   {
      this.maxStepWidth = maxStepWidth;
   }

   public double getMaxStepYaw()
   {
      return maxStepYaw;
   }

   public void setMaxStepYaw(double maxStepYaw)
   {
      this.maxStepYaw = maxStepYaw;
   }

   public double getMaxStepZ()
   {
      return maxStepZ;
   }

   public void setMaxStepZ(double maxStepZ)
   {
      this.maxStepZ = maxStepZ;
   }

   public double getMaxXYWiggle()
   {
      return maxXYWiggle;
   }

   public void setMaxXYWiggle(double maxXYWiggle)
   {
      this.maxXYWiggle = maxXYWiggle;
   }

   public double getMaxYawWiggle()
   {
      return maxYawWiggle;
   }

   public void setMaxYawWiggle(double maxYawWiggle)
   {
      this.maxYawWiggle = maxYawWiggle;
   }

   public double getMinFootholdPercent()
   {
      return minFootholdPercent;
   }

   public void setMinFootholdPercent(double minFootholdPercent)
   {
      this.minFootholdPercent = minFootholdPercent;
   }

   public double getMinStepLength()
   {
      return minStepLength;
   }

   public void setMinStepLength(double minStepLength)
   {
      this.minStepLength = minStepLength;
   }

   public double getMinStepWidth()
   {
      return minStepWidth;
   }

   public void setMinStepWidth(double minStepWidth)
   {
      this.minStepWidth = minStepWidth;
   }

   public double getMinStepYaw()
   {
      return minStepYaw;
   }

   public void setMinStepYaw(double minStepYaw)
   {
      this.minStepYaw = minStepYaw;
   }

   public double getMinSurfaceIncline()
   {
      return minSurfaceIncline;
   }

   public void setMinSurfaceIncline(double minSurfaceIncline)
   {
      this.minSurfaceIncline = minSurfaceIncline;
   }

   public double getMinXClearance()
   {
      return minXClearance;
   }

   public void setMinXClearance(double minXClearance)
   {
      this.minXClearance = minXClearance;
   }

   public double getMinYClearance()
   {
      return minYClearance;
   }

   public void setMinYClearance(double minYClearance)
   {
      this.minYClearance = minYClearance;
   }

   public double getWiggleInsideDelta()
   {
      return wiggleInsideDelta;
   }

   public void setWiggleInsideDelta(double wiggleInsideDelta)
   {
      this.wiggleInsideDelta = wiggleInsideDelta;
   }
}
