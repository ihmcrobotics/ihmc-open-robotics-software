package us.ihmc.behaviors.sequence;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.tools.thread.Throttler;

public class BehaviorActionSequenceModule
{
   private volatile boolean running = true;
   private final Throttler throttler = new Throttler();
   private final double PERIOD = Conversions.hertzToSeconds(30.0);
   // TODO: Manage multiple sequences
   private BehaviorActionSequence sequence = new BehaviorActionSequence();

   public BehaviorActionSequenceModule()
   {

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "Shutdown"));
      ThreadTools.startAThread(this::actionThread, "ActionThread");
   }

   private void actionThread()
   {
      while (running)
      {
         throttler.waitAndRun(PERIOD);

         // Update sequences edited by user

         sequence.update();
      }
   }

   private void destroy()
   {
      running = false;
   }
}
