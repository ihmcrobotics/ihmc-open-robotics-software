package us.ihmc.robotics.lidar;


public class SimulatedLIDARSensorUpdateParameters
{
   private boolean alwaysOn;
   private int updatePEriod;

//   private PacketCommunicator objectCommunicator;

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
//
//   public void setObjectCommunicator(PacketCommunicator objectCommunicator)
//   {
//      this.objectCommunicator = objectCommunicator;
//   }
//   
//   public PacketCommunicator getObjectCommunicator()
//   {
//      return objectCommunicator;
//   }
}
