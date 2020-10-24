package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.robotics.math.trajectories.Trajectory3D;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.List;

public class JumpingMomentumRateControlModuleInput
{
   private double omega0;
   private double timeInState;
   private List<Trajectory3D> vrpTrajectories;

   public void setOmega0(double omega0)
   {
      this.omega0 = omega0;
   }

   public double getOmega0()
   {
      return omega0;
   }

   public void setTimeInState(double timeInState)
   {
      this.timeInState = timeInState;
   }

   public double getTimeInState()
   {
      return timeInState;
   }

   public void setVrpTrajectories(List<Trajectory3D> vrpTrajectories)
   {
      this.vrpTrajectories = vrpTrajectories;
   }

   public List<Trajectory3D> getVrpTrajectories()
   {
      return vrpTrajectories;
   }

   public void set(JumpingMomentumRateControlModuleInput other)
   {
      omega0 = other.omega0;
      timeInState = other.timeInState;
      vrpTrajectories = other.vrpTrajectories;
   }

   @Override
   public boolean equals(Object obj)
   {
      if (obj == this)
      {
         return true;
      }
      else if (obj instanceof JumpingMomentumRateControlModuleInput)
      {
         JumpingMomentumRateControlModuleInput other = (JumpingMomentumRateControlModuleInput) obj;
         if (Double.compare(omega0, other.omega0) != 0)
            return false;
         if (Double.compare(timeInState, other.timeInState) != 0)
            return false;
         if (vrpTrajectories.size() != other.vrpTrajectories.size())
            return false;
         for (int i = 0; i < vrpTrajectories.size(); i++)
         {
            if (!vrpTrajectories.get(i).equals(other.vrpTrajectories.get(i)))
               return false;
         }
         return true;
      }
      else
      {
         return false;
      }
   }
}
