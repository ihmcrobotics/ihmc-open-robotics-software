package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.math.trajectories.core.Polynomial3D;
import us.ihmc.robotics.math.trajectories.interfaces.Polynomial3DBasics;
import us.ihmc.robotics.math.trajectories.interfaces.Polynomial3DReadOnly;

import java.util.List;

public class JumpingMomentumRateControlModuleInput
{
   private double omega0;
   private double timeInState;
   private double timeAtStartOfSignal;
   private boolean minimizeAngularMomentumRate;
   private boolean inFlight;
   private final RecyclingArrayList<Polynomial3DBasics> vrpTrajectories = new RecyclingArrayList<>(() -> new Polynomial3D(6));
   private final RecyclingArrayList<SettableContactStateProvider> contactStateProviders = new RecyclingArrayList<>(SettableContactStateProvider::new);
   private final FrameVector3D desiredLinearMomentumRateOfChange = new FrameVector3D();
   private final FrameVector3D desiredAngularMomentumRateOfChange = new FrameVector3D();

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

   public void setMinimizeAngularMomentumRate(boolean minimizeAngularMomentumRate)
   {
      this.minimizeAngularMomentumRate = minimizeAngularMomentumRate;
   }

   public boolean getMinimizeAngularMomentumRate()
   {
      return minimizeAngularMomentumRate;
   }

   public void setInFlight(boolean inFlight)
   {
      this.inFlight = inFlight;
   }

   public boolean getInFlight()
   {
      return inFlight;
   }

   public void setDesiredLinearMomentumRateOfChange(FrameVector3DReadOnly desiredLinearMomentumRateOfChange)
   {
      this.desiredLinearMomentumRateOfChange.set(desiredLinearMomentumRateOfChange);
   }

   public FrameVector3DReadOnly getDesiredLinearMomentumRateOfChange()
   {
      return desiredLinearMomentumRateOfChange;
   }

   public void setDesiredAngularMomentumRateOfChange(FrameVector3DReadOnly desiredAngularMomentumRateOfChange)
   {
      this.desiredAngularMomentumRateOfChange.set(desiredAngularMomentumRateOfChange);
   }

   public FrameVector3DReadOnly getDesiredAngularMomentumRateOfChange()
   {
      return desiredAngularMomentumRateOfChange;
   }

   public void setVrpTrajectories(List<? extends Polynomial3DReadOnly> vrpTrajectories)
   {
      this.vrpTrajectories.clear();
      for (int i = 0; i < vrpTrajectories.size(); i++)
         this.vrpTrajectories.add().set(vrpTrajectories.get(i));
   }

   public List<? extends Polynomial3DReadOnly> getVrpTrajectories()
   {
      return vrpTrajectories;
   }

   public void setContactStateProviders(List<? extends ContactStateProvider> contactStateProviders)
   {
      this.contactStateProviders.clear();
      for (int i = 0; i < contactStateProviders.size(); i++)
         this.contactStateProviders.add().set(contactStateProviders.get(i));
   }

   public List<? extends ContactStateProvider> getContactStateProviders()
   {
      return contactStateProviders;
   }

   public void set(JumpingMomentumRateControlModuleInput other)
   {
      omega0 = other.omega0;
      timeInState = other.timeInState;
      inFlight = other.inFlight;
      setVrpTrajectories(other.vrpTrajectories);
      setContactStateProviders(other.contactStateProviders);
      minimizeAngularMomentumRate = other.minimizeAngularMomentumRate;
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
         if (minimizeAngularMomentumRate != other.minimizeAngularMomentumRate)
            return false;
         if (inFlight != other.inFlight)
            return false;
         if (vrpTrajectories.size() != other.vrpTrajectories.size())
            return false;
         for (int i = 0; i < vrpTrajectories.size(); i++)
         {
            if (!vrpTrajectories.get(i).equals(other.vrpTrajectories.get(i)))
               return false;
         }
         if (contactStateProviders.size() != other.contactStateProviders.size())
            return false;
         for (int i = 0; i < contactStateProviders.size(); i++)
         {
            if (!contactStateProviders.get(i).equals(other.contactStateProviders.get(i)))
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
