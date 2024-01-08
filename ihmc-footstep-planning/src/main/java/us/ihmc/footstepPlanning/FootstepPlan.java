package us.ihmc.footstepPlanning;

import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.Collections;

public class FootstepPlan implements FootstepPlanReadOnly
{
   private final ArrayList<PlannedFootstep> footsteps = new ArrayList<>();
   private double finalTransferSplitFraction = -1.0;
   private double finalTransferWeightDistribution = -1.0;

   public FootstepPlan()
   {

   }

   /** deep copy */
   public FootstepPlan(FootstepPlanReadOnly otherFootstepPlan)
   {
      set(otherFootstepPlan);
   }

   public void set(FootstepPlanReadOnly otherFootstepPlan)
   {
      for (int i = 0; i < otherFootstepPlan.getNumberOfSteps(); i++)
      {
         footsteps.add(new PlannedFootstep(otherFootstepPlan.getFootstep(i)));
      }

      this.finalTransferSplitFraction = otherFootstepPlan.getFinalTransferSplitFraction();
      this.finalTransferWeightDistribution = otherFootstepPlan.getFinalTransferWeightDistribution();
   }

   @Override
   public int getNumberOfSteps()
   {
      return footsteps.size();
   }

   @Override
   public PlannedFootstep getFootstep(int footstepIndex)
   {
      return footsteps.get(footstepIndex);
   }

   public void addFootstep(PlannedFootstep footstep)
   {
      footsteps.add(footstep);
   }

   public PlannedFootstep addFootstep(RobotSide robotSide, FramePose3DReadOnly soleFramePose)
   {
      PlannedFootstep simpleFootstep = new PlannedFootstep(robotSide, soleFramePose);
      footsteps.add(simpleFootstep);
      return simpleFootstep;
   }

   @Override
   public double getFinalTransferSplitFraction()
   {
      return finalTransferSplitFraction;
   }

   @Override
   public double getFinalTransferWeightDistribution()
   {
      return finalTransferWeightDistribution;
   }

   public void setFinalTransferSplitFraction(double finalTransferSplitFraction)
   {
      this.finalTransferSplitFraction = finalTransferSplitFraction;
   }

   public void setFinalTransferWeightDistribution(double finalTransferWeightDistribution)
   {
      this.finalTransferWeightDistribution = finalTransferWeightDistribution;
   }

   public void reverse()
   {
      Collections.reverse(footsteps);
   }

   public void clear()
   {
      footsteps.clear();
      finalTransferSplitFraction = -1.0;
      finalTransferWeightDistribution = -1.0;
   }

   public void remove(int footstepIndex)
   {
      footsteps.remove(footstepIndex);
   }

   public String toString()
   {
      StringBuilder builder = new StringBuilder();
      builder.append("Footstep Plan: ").append("Steps: ").append(getNumberOfSteps()).append("\n");

      for (PlannedFootstep footstep : footsteps)
      {
         builder.append("\t").append("[Step Side: ")
                .append(footstep.getRobotSide())
                .append(", Position: ")
                .append(footstep.getFootstepPose().getPosition())
                .append(footstep.getFootstepPose().getOrientation())
               .append("]\n");
      }
      builder.append("\n");
      return builder.toString();
   }
}
