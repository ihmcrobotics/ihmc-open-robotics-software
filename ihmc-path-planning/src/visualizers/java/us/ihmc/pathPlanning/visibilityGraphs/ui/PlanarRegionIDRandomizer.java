package us.ihmc.pathPlanning.visibilityGraphs.ui;

import java.util.Random;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.robotEnvironmentAwareness.communication.APIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class PlanarRegionIDRandomizer
{
   private final Random random = new Random(68654);

   private final boolean isExecutorServiceProvided;
   private final ExecutorService executorService;

   private AtomicReference<PlanarRegionsList> planarRegionsReference = null;
   private Topic<PlanarRegionsList> dataTopic = null;
   private final REAMessager messager;

   public PlanarRegionIDRandomizer(REAMessager messager)
   {
      this(messager, null);
   }

   public PlanarRegionIDRandomizer(REAMessager messager, ExecutorService executorService)
   {
      this.messager = messager;
      isExecutorServiceProvided = executorService == null;

      if (isExecutorServiceProvided)
         this.executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));
      else
         this.executorService = executorService;
   }

   public void setTopics(Topic<Boolean> requestTopic, Topic<PlanarRegionsList> dataTopic)
   {
      this.dataTopic = dataTopic;
      planarRegionsReference = messager.createInput(dataTopic, null);
      messager.registerTopicListener(requestTopic, this::randomizeIDsOnThread);
   }

   private void randomizeIDsOnThread(boolean dummyArg)
   {
      PlanarRegionsList planarRegionsList = planarRegionsReference.get();
      if (planarRegionsList != null)
         executorService.execute(() -> this.randomizeIDs(planarRegionsList));
   }

   private void randomizeIDs(PlanarRegionsList planarRegionsList)
   {
      for (PlanarRegion region : planarRegionsList.getPlanarRegionsAsList())
         region.setRegionId(random.nextInt());

      messager.submitMessage(dataTopic, planarRegionsList);
   }

   public void start()
   {
   }

   public void stop()
   {
      if (!isExecutorServiceProvided)
         executorService.shutdownNow();
   }
}
