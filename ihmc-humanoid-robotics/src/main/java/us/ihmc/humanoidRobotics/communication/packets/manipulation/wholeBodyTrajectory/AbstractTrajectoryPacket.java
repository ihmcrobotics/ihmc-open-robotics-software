package us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory;

import us.ihmc.communication.packets.Packet;

public abstract class AbstractTrajectoryPacket extends Packet<AbstractTrajectoryPacket>
{
   public TaskRegion taskNodeRegion;
//   protected SideDependentList<ConfigurationBuildOrder> configurationBuildOrders;
//   protected SideDependentList<SelectionMatrix6D> controllableSelectionMatrices;
//   protected double trajectoryTime;
   
   public AbstractTrajectoryPacket()
   {
      
   }

   @Override
   public boolean epsilonEquals(AbstractTrajectoryPacket other, double epsilon)
   {
      return false;
   }

}
