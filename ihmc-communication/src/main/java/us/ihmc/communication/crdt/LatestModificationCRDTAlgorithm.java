package us.ihmc.communication.crdt;

import java.time.Instant;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Implements a simple Conflict-free replicated data type using timestamps.
 * <a href="https://en.wikipedia.org/wiki/Conflict-free_replicated_data_type">Conflict-free replicated data type</a>
 */
public class LatestModificationCRDTAlgorithm<T extends LatestModificationReplicate>
{
   private final BooleanSupplier isDataModified;
   private final Runnable clearDataModified;
   private Instant lastModified = Instant.now();
   private final Consumer<Instant> publisher;
   private final Supplier<Instant> subscription;
   private final Runnable copyMessageData;

   /**
    *
    * @param data
    * @param publisher
    * @param subscription latest message or null
    */
   public LatestModificationCRDTAlgorithm(BooleanSupplier isDataModified,
                                          Runnable clearDataModified,
                                          Consumer<Instant> publisher,
                                          Supplier<Instant> subscription,
                                          Runnable copyMessageData)
   {
      this.isDataModified = isDataModified;
      this.clearDataModified = clearDataModified;
      this.publisher = publisher;
      this.subscription = subscription;
      this.copyMessageData = copyMessageData;
   }

   /**
    * Core algorithm. When our data is modified, we send it off.
    * When data comes in, if it's newer, we overrite our data with it.
    */
   public void update()
   {
      if (isDataModified.getAsBoolean())
      {
         lastModified = Instant.now();
         publisher.accept(lastModified);
         clearDataModified.run();
      }

      Instant incomingMessageDataLastModified = subscription.get();
      if (incomingMessageDataLastModified.isAfter(lastModified))
      {
         copyMessageData.run();
      }
   }
}
