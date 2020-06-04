package us.ihmc.footstepPlanning;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.Collections;

public class FootstepPlan implements FootstepPlanReadOnly
{
   private final ArrayList<PlannedFootstep> footsteps = new ArrayList<>();
   private double finalTransferSplitFraction = -1.0;
   private double finalTransferWeightDistribution = -1.0;

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

   public PlannedFootstep addFootstep(RobotSide robotSide, FramePose3D soleFramePose)
   {
      PlannedFootstep simpleFootstep = new PlannedFootstep(robotSide, soleFramePose);
      footsteps.add(simpleFootstep);
      return simpleFootstep;
   }

   public double getFinalTransferSplitFraction()
   {
      return finalTransferSplitFraction;
   }

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
}
