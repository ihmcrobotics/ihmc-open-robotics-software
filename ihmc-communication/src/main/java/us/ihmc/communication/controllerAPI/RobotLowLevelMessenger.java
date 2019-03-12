package us.ihmc.communication.controllerAPI;

public interface RobotLowLevelMessenger
{
   void sendFreezeRequest();

   void sendStandRequest();

   default void sendShutdownRequest()
   {
      throw new RuntimeException("Robot shutdown request is not implemented.");
   }
}
