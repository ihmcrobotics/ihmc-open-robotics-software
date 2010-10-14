package us.ihmc.commonWalkingControlModules.sensors;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;

public interface VirtualFootSwitchInterface
{
   public abstract boolean hasFootHitGround();
   public abstract void setUpGUI(SimulationConstructionSet scs);
}
