package us.ihmc.gr00t;

import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.sensors.ImageSensor;

/**
 * Generic asynchronous GR00T worker.
 * <p>
 * Robot embodiment, observation packing, prompts, action decoding, and execution are supplied by a
 * {@link Gr00tTask}. This class only owns the request-loop lifecycle and bridge diagnostics.
 */
public final class Gr00tUpdateThread
{
   private final RepeatingTaskThread thread = new RepeatingTaskThread(getClass().getSimpleName(), this::runTask).setFrequencyLimit(50.0);
   private final Gr00tTask task;
   private final Gr00tInferenceSession inference;
   private volatile String status = "Not connected to GR00T bridge";

   public Gr00tUpdateThread(Gr00tTask task,
                            ImageSensor imageSensor,
                            String gr00tBridgeHost,
                            int gr00tBridgePort)
   {
      this.task = task;
      task.setStatusConsumer(this::setStatus);
      inference = new Gr00tInferenceSession(task, imageSensor, gr00tBridgeHost, gr00tBridgePort, this::setStatus);
   }

   public void runTask()
   {
      task.updateBeforeInference();
      inference.update();
      task.updateAfterInference();
   }

   public Gr00tTask getTask()
   {
      return task;
   }

   public String getStatus()
   {
      return status;
   }

   public boolean hasImageSensor()
   {
      return inference.hasImageSensor();
   }

   public void setImageSensor(ImageSensor imageSensor)
   {
      inference.setImageSensor(imageSensor);
   }

   public void startRepeating()
   {
      thread.startRepeating();
   }

   public void destroy()
   {
      thread.blockingKill();
      inference.close();
   }

   public double getEndToEndInferenceTimeMilliseconds()
   {
      return inference.getEndToEndInferenceTimeMilliseconds();
   }

   public float getPolicyInferenceTimeMilliseconds()
   {
      return inference.getPolicyInferenceTimeMilliseconds();
   }

   public float getServerInferenceTimeMilliseconds()
   {
      return inference.getServerInferenceTimeMilliseconds();
   }

   private void setStatus(String status)
   {
      this.status = status;
      task.observeStatus(status);
   }
}
