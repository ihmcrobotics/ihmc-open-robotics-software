package us.ihmc.exampleSimulations.lipmWalker;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.humanoidRobotics.footstep.Footstep;

import java.util.ArrayList;

public class ICPCalculator
{
   private final FramePoint3D icp_d_eos_i = new FramePoint3D();
   private final FramePoint3D vrp_d_i = new FramePoint3D();
   private double omega;

   private double stepTime;

   private ArrayList<Footstep> virtualRepellentPoints;
   private int nStep;

   public void update(ArrayList<Footstep> steps, Footstep supportStep, double stepTime, double omega)
   {

      virtualRepellentPoints = steps;
      nStep = virtualRepellentPoints.size();
      this.omega = omega;
      this.stepTime = stepTime;
      for (int i = steps.size() - 1; i >= 0; --i)
      {
         // set icp_d_eos_i to vrp
         if (i == steps.size() - 1)
         {
            icp_d_eos_i.set(steps.get(i).getFootstepPose().getTranslation());
         }
         else
         {
            if (i == 0)
            {
               vrp_d_i.set(supportStep.getFootstepPose().getTranslation());
            }
            else
               vrp_d_i.set(steps.get(i - 1).getFootstepPose().getTranslation());

            icp_d_eos_i.set(get_icp_d_ini_i(vrp_d_i, omega, stepTime, icp_d_eos_i));
         }
      }
      icp_d_eos_i.set(get_icp_d_ini_i(vrp_d_i, omega, stepTime, icp_d_eos_i));
   }

   private FramePoint3D calculate_eos_icp(int stepNumber)
   {
      FramePoint3D endOfStepDesiredCPPosition = new FramePoint3D();
      FramePoint3D virtualRepellentPointPosition = new FramePoint3D();
      virtualRepellentPoints.get(stepNumber).getPosition(virtualRepellentPointPosition);
      if (stepNumber == nStep)
      {
         return new FramePoint3D(virtualRepellentPoints.get(nStep - 1).getFootstepPose().getTranslation());
      }
      else
      {
         double exp_wt = Math.exp(-omega * stepTime);
         FramePoint3D diff = new FramePoint3D();
         diff.sub(calculate_eos_icp(stepNumber + 1), virtualRepellentPoints.get(stepNumber).getFootstepPose().getTranslation());
         diff.scale(exp_wt);
         diff.add(virtualRepellentPoints.get(stepNumber).getFootstepPose().getTranslation());
         return diff;
      }
   }

   private FramePoint3D get_icp_d_ini_i(FramePoint3D vrp_d_i, double omega, double stepDuration, FramePoint3D icp_d_eos_i)
   {
      double exp_wt = Math.exp(-omega * stepDuration);
      FramePoint3D diff = new FramePoint3D();
      diff.sub(icp_d_eos_i, vrp_d_i);
      diff.scale(exp_wt);
      diff.add(vrp_d_i);
      return diff;
   }

   public FramePoint3D get_icp_desired_at_t(double t)
   {
      double exp_wt = Math.exp(omega * (t - stepTime));
      FramePoint3D diff = new FramePoint3D();
      diff.sub(icp_d_eos_i, vrp_d_i);
      diff.scale(exp_wt);
      diff.add(vrp_d_i);
      return diff;
   }

   public FramePoint3D get_icp_desired_velocity_at_t(double t)
   {
      double exp_wt = Math.exp(omega * (t - stepTime));
      FramePoint3D diff = new FramePoint3D();
      diff.sub(icp_d_eos_i, vrp_d_i);
      diff.scale(exp_wt);
      diff.scale(omega);
      return diff;
   }

   public FramePoint3D getIcp_d_eos_i()
   {
      return icp_d_eos_i;
   }

   public FramePoint3D getVrp_d_i()
   {
      return vrp_d_i;
   }
}
