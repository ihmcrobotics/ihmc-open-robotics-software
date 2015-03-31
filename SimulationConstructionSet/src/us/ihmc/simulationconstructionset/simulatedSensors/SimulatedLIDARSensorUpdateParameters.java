package us.ihmc.simulationconstructionset.simulatedSensors;

import us.ihmc.communication.packetCommunicator.PacketCommunicatorMock;

public class SimulatedLIDARSensorUpdateParameters
{
   private boolean alwaysOn;
   private int updatePEriod;

   private PacketCommunicatorMock objectCommunicator;

   public boolean isAlwaysOn()
   {
      return alwaysOn;
   }

   public void setAlwaysOn(boolean alwaysOn)
   {
      this.alwaysOn = alwaysOn;
   }

   public int getUpdatePeriodInMillis()
   {
      return updatePEriod;
   }

   public void setUpdatePeriodInMillis(int updatePeriod)
   {
      this.updatePEriod = updatePeriod;
   }

   public void setObjectCommunicator(PacketCommunicatorMock objectCommunicator)
   {
      this.objectCommunicator = objectCommunicator;
   }
   
   public PacketCommunicatorMock getObjectCommunicator()
   {
      return objectCommunicator;
   }
}
