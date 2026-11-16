package us.ihmc.gr00t;

import us.ihmc.sensors.ImageSensor;

import java.nio.DoubleBuffer;
import java.util.concurrent.CompletableFuture;
import java.util.function.Consumer;

/** Owns the one-request-at-a-time GR00T bridge lifecycle and inference timing diagnostics. */
final class Gr00tInferenceSession implements AutoCloseable
{
   static final long CONNECTION_RETRY_NANOS = 1_000_000_000L;

   private final Gr00tClient client;
   private final Gr00tTask task;
   private final Gr00tObservationSource observationSource;
   private final Consumer<String> statusUpdater;
   private volatile ImageSensor imageSensor;
   private CompletableFuture<byte[]> request;
   private Gr00tTask.Request requestContext;
   private long nextRequestAttemptNanos;
   private long requestStartNanos;
   private volatile double endToEndInferenceTimeMilliseconds;
   private volatile float policyInferenceTimeMilliseconds;
   private volatile float serverInferenceTimeMilliseconds;

   Gr00tInferenceSession(Gr00tTask task,
                         ImageSensor imageSensor,
                         String bridgeHost,
                         int bridgePort,
                         Consumer<String> statusUpdater)
   {
      this.task = task;
      this.imageSensor = imageSensor;
      this.statusUpdater = statusUpdater;
      Gr00tModelConfiguration configuration = task.getModelConfiguration();
      int imageWidth = positiveIntegerProperty("gr00t.bridge.image.width", configuration.defaultImageWidth());
      int imageHeight = positiveIntegerProperty("gr00t.bridge.image.height", configuration.defaultImageHeight());
      int chunkLength = positiveIntegerProperty("gr00t.bridge.chunk.length", configuration.defaultChunkLength());
      us.ihmc.robotics.robotSide.SideDependentList<String> imageKeys = new us.ihmc.robotics.robotSide.SideDependentList<>(
            System.getProperty("gr00t.bridge.image.left.key", configuration.defaultLeftImageKey()),
            System.getProperty("gr00t.bridge.image.right.key", configuration.defaultRightImageKey()));
      client = new Gr00tClient(bridgeHost,
                              bridgePort,
                              configuration.stateSize(),
                              configuration.actionSize(),
                              chunkLength,
                              imageWidth,
                              imageHeight,
                              imageKeys,
                              System.getProperty("gr00t.bridge.state.key", "state"),
                              System.getProperty("gr00t.bridge.prompt.key", "prompt"),
                              System.getProperty("gr00t.bridge.action.key", "actions"),
                              System.getProperty("gr00t.bridge.layout.id", configuration.layoutId()),
                              positiveIntegerProperty("gr00t.bridge.action.horizon", configuration.defaultActionHorizon()),
                              positiveDoubleProperty("gr00t.bridge.action.rate.hz", configuration.defaultActionRateHz()));
      observationSource = task.createObservationSource(client);
   }

   void update()
   {
      if (task.consumeInferenceResetRequested())
         nextRequestAttemptNanos = 0L;
      if (!task.isRunning())
         return;

      if (request != null && request.isDone())
         processCompletedRequest();

      if (request == null && task.shouldRequestInference() && isRetryDue(System.nanoTime(), nextRequestAttemptNanos))
         startRequest();
   }

   private void processCompletedRequest()
   {
      endToEndInferenceTimeMilliseconds = (System.nanoTime() - requestStartNanos) * 1.0e-6;
      if (!request.isCompletedExceptionally())
      {
         if (client.unpack(request))
         {
            policyInferenceTimeMilliseconds = client.getPolicyTimingMs();
            serverInferenceTimeMilliseconds = client.getServerTimingMs();
            DoubleBuffer actionChunk = client.getActionChunk().asDoubleBuffer();
            int realActionCount = Math.min(client.getHorizon(), actionChunk.capacity() / task.getModelConfiguration().actionSize());
            task.recordActionsReceived(realActionCount);
            if (isResponseAccepted(task, requestContext))
               task.processActionChunk(actionChunk, realActionCount);
            else
               updateStatus("Discarded an inference response for an outdated command");
         }
         else
         {
            task.discardAcceptedActionChunk();
            updateStatus("Rejected malformed GR00T response");
         }
      }
      else
      {
         updateStatus("GR00T inference request failed");
         nextRequestAttemptNanos = System.nanoTime() + CONNECTION_RETRY_NANOS;
      }
      requestContext = null;
      request = null;
   }

   private void startRequest()
   {
      requestStartNanos = System.nanoTime();
      if (!observationSource.pack(imageSensor))
      {
         updateStatus("Waiting for robot data...");
         return;
      }

      requestContext = task.getRequest();
      client.setPrompt(requestContext.prompt());
      request = client.request();
      if (request == null)
      {
         requestStartNanos = 0L;
         requestContext = null;
         nextRequestAttemptNanos = System.nanoTime() + CONNECTION_RETRY_NANOS;
         String status = "Could not connect to GR00T bridge at ws://" + client.getHost() + ":" + client.getPort();
         if (client.getLastConnectionError() != null)
            status += " (" + client.getLastConnectionError() + ")";
         updateStatus(status);
      }
      else
      {
         nextRequestAttemptNanos = 0L;
         updateStatus("Requested inference...");
      }
   }

   static boolean isRetryDue(long nowNanos, long nextAttemptNanos)
   {
      return nowNanos - nextAttemptNanos >= 0L;
   }

   static boolean isResponseAccepted(Gr00tTask task, Gr00tTask.Request request)
   {
      return request != null && task.accepts(request);
   }

   void setImageSensor(ImageSensor imageSensor)
   {
      this.imageSensor = imageSensor;
   }

   boolean hasImageSensor()
   {
      return imageSensor != null;
   }

   double getEndToEndInferenceTimeMilliseconds()
   {
      return endToEndInferenceTimeMilliseconds;
   }

   float getPolicyInferenceTimeMilliseconds()
   {
      return policyInferenceTimeMilliseconds;
   }

   float getServerInferenceTimeMilliseconds()
   {
      return serverInferenceTimeMilliseconds;
   }

   @Override
   public void close()
   {
      observationSource.close();
      client.destroy();
   }

   private void updateStatus(String status)
   {
      statusUpdater.accept(status);
   }

   private static int positiveIntegerProperty(String name, int defaultValue)
   {
      int value = Integer.parseInt(System.getProperty(name, Integer.toString(defaultValue)));
      if (value <= 0)
         throw new IllegalArgumentException(name + " must be positive, got " + value);
      return value;
   }

   private static double positiveDoubleProperty(String name, double defaultValue)
   {
      double value = Double.parseDouble(System.getProperty(name, Double.toString(defaultValue)));
      if (!Double.isFinite(value) || value <= 0.0)
         throw new IllegalArgumentException(name + " must be finite and positive, got " + value);
      return value;
   }
}
