package us.ihmc.robotEnvironmentAwareness.polygonizer;

import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.MessagerAPIFactory.TopicTheme;
import us.ihmc.robotEnvironmentAwareness.planarRegion.IntersectionEstimationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionIntersectionCalculator;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;

public class PolygonizerManager
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final Category Root = apiFactory.createRootCategory(apiFactory.createCategoryTheme(PolygonizerManager.class.getSimpleName()));
   static
   {
      apiFactory.includeMessagerAPIs(Polygonizer.API);
   }

   private static final CategoryTheme PlanarRegion = apiFactory.createCategoryTheme("PlanarRegion");
   private static final CategoryTheme Semgentation = apiFactory.createCategoryTheme("Segmentation");
   private static final CategoryTheme Intersections = apiFactory.createCategoryTheme("Intersection");

   private static final TopicTheme Data = apiFactory.createTopicTheme("Data");
   private static final TopicTheme Reload = apiFactory.createTopicTheme("Reload");
   private static final TopicTheme Parameters = apiFactory.createTopicTheme("Parameters");

   public static final Topic<List<PlanarRegionSegmentationRawData>> PlanarRegionSemgentationData = Root.child(PlanarRegion).child(Semgentation).topic(Data);
   public static final Topic<Boolean> PlanarRegionSemgentationReload = Root.child(PlanarRegion).child(Semgentation).topic(Reload);
   public static final Topic<IntersectionEstimationParameters> IntersectionParameters = Root.child(PlanarRegion).child(Intersections).topic(Parameters);

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();

   private final AtomicReference<List<PlanarRegionSegmentationRawData>> lastSegmentationDataReceived;
   private final AtomicReference<IntersectionEstimationParameters> intersectionParameters;
   private final Messager messager;

   public PolygonizerManager(Messager messager)
   {
      this(messager, null);
   }

   public PolygonizerManager(Messager messager, ExecutorService executorService)
   {
      this.messager = messager;
      new Polygonizer(messager, executorService);

      lastSegmentationDataReceived = messager.createInput(PlanarRegionSemgentationData);
      intersectionParameters = messager.createInput(IntersectionParameters, new IntersectionEstimationParameters());
      messager.addTopicListener(PlanarRegionSemgentationData, this::handleSegmentationData);
      messager.addTopicListener(PlanarRegionSemgentationReload, this::handleReload);
   }

   private void handleSegmentationData(List<PlanarRegionSegmentationRawData> segmentationData)
   {
      PlanarRegionIntersectionCalculator.computeIntersections(segmentationData, intersectionParameters.get());
      messager.submitMessage(Polygonizer.PolygonizerInput, Polygonizer.toInputList(segmentationData));
   }

   private void handleReload(boolean value)
   {
      if (lastSegmentationDataReceived.get() != null)
      {
         lastSegmentationDataReceived.get().forEach(data -> data.clearIntersections());
         messager.submitMessage(PlanarRegionSemgentationData, lastSegmentationDataReceived.get());
      }
   }
}
