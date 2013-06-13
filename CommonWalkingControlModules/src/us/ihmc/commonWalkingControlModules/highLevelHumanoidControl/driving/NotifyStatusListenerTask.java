package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.driving;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.taskExecutor.Task;
import us.ihmc.utilities.io.streamingData.QueueBasedStreamingDataProducer;

public class NotifyStatusListenerTask<T> implements Task
{
   private final QueueBasedStreamingDataProducer<T> drivingStatusListener;
   private final T statusObject;
   
   public NotifyStatusListenerTask(QueueBasedStreamingDataProducer<T> drivingStatusListener, T statusObject)
   {
      this.drivingStatusListener = drivingStatusListener;
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

}
