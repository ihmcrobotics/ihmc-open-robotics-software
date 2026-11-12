package us.ihmc.gr00t;

import java.nio.DoubleBuffer;
import java.util.function.Consumer;

/** Task/embodiment plug-in consumed by the generic GR00T request loop. */
public interface Gr00tTask
{
   record Request(String prompt, long generation)
   {
   }

   Gr00tModelConfiguration getModelConfiguration();

   Gr00tObservationSource createObservationSource(Gr00tClient client);

   void setStatusConsumer(Consumer<String> statusConsumer);

   default void observeStatus(String status)
   {
   }

   void updateBeforeInference();

   void updateAfterInference();

   boolean isRunning();

   boolean shouldRequestInference();

   Request getRequest();

   boolean accepts(Request request);

   boolean consumeInferenceResetRequested();

   void recordActionsReceived(int count);

   void discardAcceptedActionChunk();

   void processActionChunk(DoubleBuffer actionChunk, int realActionCount);
}
