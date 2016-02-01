package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.SpineTorques;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.screwTheory.CompositeRigidBodyInertia;
import us.ihmc.robotics.screwTheory.Wrench;

public interface SpineLungingControlModule extends SpineControlModule
{
	public abstract void setSpineTorquesForZeroQdd();
	
	public abstract void setSpineTorquesForDeltaCmp();
	
	public abstract void setSpineTorquesForDeltaCmpUsingID();
	
	public abstract void setSpineTorquesForGravityCancel();
	
   public abstract void setDesiredDeltaCmp(FrameVector2d deltaCmp);
   
   public abstract void getSpineTorques(SpineTorques spineTorquesToPack);
   
   public abstract CompositeRigidBodyInertia getTotalUpperBodyMoI();
   
   public abstract void computeTotalUpperBodyMoIProjectedAbout(FrameVector2d projectionAxis);
   
   public abstract void computeTotalWrenchExertedOnPelvis(Wrench totalUpperBodyWrench);
   
   public abstract double getTotalUpperBodyMoIProjected();
   
   public abstract void updateTotalUpperBodyMoI();
   
   public abstract FrameVector2d computeTorqueVectorForDeltaCMP(FrameVector2d deltaCMP, boolean returnActuatorTorqueContributionOnly);
 
   public abstract double computeMaxCmpDisplacement();
   
}

