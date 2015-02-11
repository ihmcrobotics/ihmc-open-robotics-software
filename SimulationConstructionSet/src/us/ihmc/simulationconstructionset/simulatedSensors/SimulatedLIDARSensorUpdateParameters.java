package us.ihmc.simulationconstructionset.simulatedSensors;

import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;

public class SimulatedLIDARSensorUpdateParameters
{
   private boolean alwaysOn;
   private double updateRate;

   private PacketCommunicator objectCommunicator;

   public boolean isAlwaysOn()
   {
      return alwaysOn;
   }

   public void setAlwaysOn(boolean alwaysOn)
   {
      this.alwaysOn = alwaysOn;
   }

   public double getUpdateRate()
   {
      return updateRate;
   }

   public void setUpdateRate(double updateRate)
   {
      this.updateRate = updateRate;
   }

   public void setObjectCommunicator(PacketCommunicator objectCommunicator)
   {
      this.objectCommunicator = objectCommunicator;
   }
   
   public PacketCommunicator getObjectCommunicator()
   {
      return objectCommunicator;
   }
}
