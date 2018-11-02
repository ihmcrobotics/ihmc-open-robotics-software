package us.ihmc.footstepPlanning.ui.components;

import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.MessagerPlannerParameters;

public class SettableMessagerPlannerParameters implements MessagerPlannerParameters
{
   private double idealFootstepWidth;
   private double idealFootstepLength;

   private double maxStepReach;
   private double maxStepYaw;
   private double minStepWidth;
   private double minStepLength;
   private double minStepYaw;
   private double maxStepZ;
   private double minFootholdPercent;
   private double minSurfaceIncline;
   private double maxStepWidth;

   public SettableMessagerPlannerParameters(FootstepPlannerParameters footstepPlannerParameters)
   {
      set(footstepPlannerParameters);
   }
   
   public SettableMessagerPlannerParameters(MessagerPlannerParameters footstepPlannerParameters)
   {
      set(footstepPlannerParameters);
   }

   public void set(FootstepPlannerParameters footstepPlannerParameters)
   {
      this.idealFootstepWidth = footstepPlannerParameters.getIdealFootstepWidth();
      this.idealFootstepLength = footstepPlannerParameters.getIdealFootstepLength();
      this.maxStepReach = footstepPlannerParameters.getMaximumStepReach();
      this.maxStepYaw = footstepPlannerParameters.getMaximumStepYaw();
      this.minStepWidth = footstepPlannerParameters.getMinimumStepWidth();
      this.minStepLength = footstepPlannerParameters.getMinimumStepLength();
      this.minStepYaw = footstepPlannerParameters.getMinimumStepYaw();
      this.maxStepZ = footstepPlannerParameters.getMaximumStepZ();
      this.maxStepWidth = footstepPlannerParameters.getMaximumStepWidth();
      this.minFootholdPercent = footstepPlannerParameters.getMinimumFootholdPercent();
      this.minSurfaceIncline = footstepPlannerParameters.getMinimumSurfaceInclineRadians();
   }
   
   public void set(MessagerPlannerParameters footstepPlannerParameters)
   {
      this.idealFootstepWidth = footstepPlannerParameters.getIdealFootstepWidth();
      this.idealFootstepLength = footstepPlannerParameters.getIdealFootstepLength();
      this.maxStepReach = footstepPlannerParameters.getMaximumStepReach();
      this.maxStepYaw = footstepPlannerParameters.getMaximumStepYaw();
      this.minStepWidth = footstepPlannerParameters.getMinimumStepWidth();
      this.minStepLength = footstepPlannerParameters.getMinimumStepLength();
      this.minStepYaw = footstepPlannerParameters.getMinimumStepYaw();
      this.maxStepZ = footstepPlannerParameters.getMaximumStepZ();
      this.maxStepWidth = footstepPlannerParameters.getMaximumStepWidth();
      this.minFootholdPercent = footstepPlannerParameters.getMinimumFootholdPercent();
      this.minSurfaceIncline = footstepPlannerParameters.getMinimumSurfaceInclineRadians();
   }

   public void setIdealFootstepWidth(double idealFootstepLength)
   {
      this.idealFootstepLength = idealFootstepLength;
   }

   public void setIdealFootstepLength(double idealFootstepLength)
   {
      this.idealFootstepLength = idealFootstepLength;
   }

   public void setMaximumStepReach(double maxStepReach)
   {
      this.maxStepReach = maxStepReach;
   }

   public void setMaximumStepYaw(double maxStepYaw)
   {
      this.maxStepYaw = maxStepYaw;
   }

   public void setMinimumStepWidth(double minStepWidth)
   {
      this.minStepWidth = minStepWidth;
   }

   public void setMinimumStepLength(double minStepLength)
   {
      this.minStepLength = minStepLength;
   }

   public void setMinimumStepYaw(double minStepYaw)
   {
      this.minStepYaw = minStepYaw;
   }

   public void setMaximumStepZ(double maxStepZ)
   {
      this.maxStepZ = maxStepZ;
   }

   public void setMaximumStepWidth(double maxStepWidth)
   {
      this.maxStepWidth = maxStepWidth;
   }

   public void setMinimumFootholdPercent(double minFootholdPercent)
   {
      this.minFootholdPercent = minFootholdPercent;
   }

   public void setMinimumSurfaceInclineRadians(double minSurfaceIncline)
   {
      this.minSurfaceIncline = minSurfaceIncline;
   }

   @Override
   public double getIdealFootstepWidth()
   {
      return idealFootstepWidth;
   }

   @Override
   public double getIdealFootstepLength()
   {
      return idealFootstepLength;
   }

   @Override
   public double getMaximumStepReach()
   {
      return maxStepReach;
   }

   @Override
   public double getMaximumStepYaw()
   {
      return maxStepYaw;
   }

   @Override
   public double getMinimumStepWidth()
   {
      return minStepWidth;
   }

   @Override
   public double getMinimumStepLength()
   {
      return minStepLength;
   }

   @Override
   public double getMinimumStepYaw()
   {
      return minStepYaw;
   }

   @Override
   public double getMaximumStepZ()
   {
      return maxStepZ;
   }

   @Override
   public double getMaximumStepWidth()
   {
      return maxStepWidth;
   }

   @Override
   public double getMinimumFootholdPercent()
   {
      return minFootholdPercent;
   }

   @Override
   public double getMinimumSurfaceInclineRadians()
   {
      return minSurfaceIncline;
   }
}
