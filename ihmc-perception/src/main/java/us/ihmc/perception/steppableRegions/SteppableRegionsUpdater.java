package us.ihmc.perception.steppableRegions;

import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.SteppableRegionDebugImagesMessage;
import perception_msgs.msg.dds.SteppableRegionsListCollectionMessage;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

public class SteppableRegionsUpdater
{
   private final SteppableRegionsCalculationModule steppableRegionsCalculationModule;

   private final AtomicReference<HeightMapMessage> latestHeightMapMessageReference = new AtomicReference<>();
   private final AtomicReference<SteppableRegionCalculatorParametersReadOnly> latestParameters = new AtomicReference<>();
   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);

   public SteppableRegionsUpdater(SteppableRegionCalculatorParametersReadOnly steppableRegionsParameters)
   {
      steppableRegionsCalculationModule = new SteppableRegionsCalculationModule(steppableRegionsParameters);
   }

   public void submitLatestHeightMapMessage(HeightMapMessage heightMapMessage)
   {
      this.latestHeightMapMessageReference.set(heightMapMessage);
   }

   public void submitLatestSteppableRegionCalculatorParameters(SteppableRegionCalculatorParametersReadOnly latestParameters)
   {
      this.latestParameters.set(latestParameters);
   }

   public void addSteppableRegionListCollectionOutputConsumer(Consumer<SteppableRegionsListCollectionMessage> outputConsumer)
   {
      steppableRegionsCalculationModule.addSteppableRegionListCollectionOutputConsumer(outputConsumer);
   }

   public void addSteppableRegionDebugConsumer(Consumer<SteppableRegionDebugImagesMessage> outputConsumer)
   {
      steppableRegionsCalculationModule.addSteppableRegionDebugConsumer(outputConsumer);
   }

   public void computeASynch()
   {
      executorService.clearQueueAndExecute(this::compute);
   }

   public void compute()
   {
      HeightMapMessage heightMapMessage = latestHeightMapMessageReference.getAndSet(null);
      if (heightMapMessage == null)
         return;

      SteppableRegionCalculatorParametersReadOnly latestParameters = this.latestParameters.getAndSet(null);
      HeightMapData heightMapData = HeightMapMessageTools.unpackMessage(heightMapMessage);
      if (latestParameters != null)
         steppableRegionsCalculationModule.setSteppableRegionsCalculatorParameters(latestParameters);

      steppableRegionsCalculationModule.compute(heightMapData);
   }

   public void destroy()
   {
      steppableRegionsCalculationModule.destroy();
   }
}
