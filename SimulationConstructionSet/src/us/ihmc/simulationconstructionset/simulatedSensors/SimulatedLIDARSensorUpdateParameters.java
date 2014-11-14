package us.ihmc.simulationconstructionset.simulatedSensors;

import us.ihmc.utilities.net.ObjectCommunicator;

public class SimulatedLIDARSensorUpdateParameters
{
   private boolean alwaysOn;
   private double updateRate;

   private ObjectCommunicator objectCommunicator;

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

   public void setObjectCommunicator(ObjectCommunicator objectCommunicator)
   {
      this.objectCommunicator = objectCommunicator;
   }
   
   public ObjectCommunicator getObjectCommunicator()
   {
      return objectCommunicator;
   }
}
