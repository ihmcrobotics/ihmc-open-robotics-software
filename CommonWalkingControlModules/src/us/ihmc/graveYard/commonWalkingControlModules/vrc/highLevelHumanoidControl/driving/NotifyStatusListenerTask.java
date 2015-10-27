package us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.driving;

import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.tools.taskExecutor.Task;

public class NotifyStatusListenerTask<T extends Packet> implements Task
{
   private final HumanoidGlobalDataProducer drivingStatusListener;
   private final T statusObject;
   
   public NotifyStatusListenerTask(HumanoidGlobalDataProducer statusProducer, T statusObject)
   {
      this.drivingStatusListener = statusProducer;
      this.statusObject = statusObject;
   }

   public void doTransitionIntoAction()
   {
      drivingStatusListener.queueDataToSend(statusObject);
   }

   public void doAction()
   {

   }

   public void doTransitionOutOfAction()
   {

   }

   public boolean isDone()
   {
      return true;
   }

   @Override
   public void pause()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void resume()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void stop()
   {
      // TODO Auto-generated method stub
      
   }

}
