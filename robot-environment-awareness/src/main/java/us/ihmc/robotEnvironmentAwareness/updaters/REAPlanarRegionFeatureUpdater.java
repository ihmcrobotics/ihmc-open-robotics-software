package us.ihmc.robotEnvironmentAwareness.updaters;

import java.io.File;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import gnu.trove.map.hash.TIntObjectHashMap;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;
import us.ihmc.robotEnvironmentAwareness.planarRegion.CustomPlanarRegionHandler;
import us.ihmc.robotEnvironmentAwareness.planarRegion.CustomRegionMergeParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.IntersectionEstimationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionIntersectionCalculator;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationCalculator;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationNodeData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.SurfaceNormalFilterParameters;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionSegmentationDataExporter;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class REAPlanarRegionFeatureUpdater implements RegionFeaturesProvider
{
   private static final boolean EXPORT_SEGMENTATION_ON_EXCEPTION = false;

   private static final String segmentationTimeReport = "Segmentation took: ";
   private static final String intersectionsTimeReport = "Processing intersections took: ";

   private final PlanarRegionSegmentationDataExporter dataExporter = EXPORT_SEGMENTATION_ON_EXCEPTION
         ? new PlanarRegionSegmentationDataExporter(new File("DataThrowingException/Segmentation"))
         : null;

   private final TimeReporter timeReporter = new TimeReporter();

   private final PlanarRegionSegmentationCalculator segmentationCalculator = new PlanarRegionSegmentationCalculator();

   private final TIntObjectHashMap<PlanarRegion> customPlanarRegions = new TIntObjectHashMap<>();

   private PlanarRegionsList planarRegionsList = null;
   private List<LineSegment3D> planarRegionsIntersections = null;

   private AtomicReference<Boolean> isOcTreeEnabled;
   private AtomicReference<Boolean> enableSegmentation;
   private AtomicReference<Boolean> clearSegmentation;
   private AtomicReference<Boolean> enableCustomRegions;
   private AtomicReference<Boolean> clearCustomRegions;
   private AtomicReference<Boolean> enablePolygonizer;
   private AtomicReference<Boolean> clearPolygonizer;
   private AtomicReference<Boolean> enableIntersectionCalulator;
   private AtomicReference<PlanarRegionSegmentationParameters> planarRegionSegmentationParameters;
   private AtomicReference<CustomRegionMergeParameters> customRegionMergingParameters;
   private AtomicReference<ConcaveHullFactoryParameters> concaveHullFactoryParameters;
   private AtomicReference<PolygonizerParameters> polygonizerParameters;
   private AtomicReference<IntersectionEstimationParameters> intersectionEstimationParameters;
   private AtomicReference<SurfaceNormalFilterParameters> surfaceNormalFilterParameters;
   private final Messager reaMessager;

   private Topic<Boolean> ocTreeEnableTopic = REAModuleAPI.OcTreeEnable;
   private Topic<Boolean> planarRegionsSegmentationEnableTopic = REAModuleAPI.PlanarRegionsSegmentationEnable;
   private Topic<Boolean> planarRegionsSegmentationClearTopic = REAModuleAPI.PlanarRegionsSegmentationClear;
   private Topic<Boolean> customRegionsMergingEnableTopic = REAModuleAPI.CustomRegionsMergingEnable;
   private Topic<Boolean> customRegionsClearTopic = REAModuleAPI.CustomRegionsClear;
   private Topic<Boolean> planarRegionsPolygonizerEnableTopic = REAModuleAPI.PlanarRegionsPolygonizerEnable;
   private Topic<Boolean> planarRegionsPolygonizerClearTopic = REAModuleAPI.PlanarRegionsPolygonizerClear;
   private Topic<Boolean> planarRegionsIntersectionEnableTopic = REAModuleAPI.PlanarRegionsIntersectionEnable;

   private Topic<PlanarRegionSegmentationParameters> planarRegionsSegmentationParametersTopic = REAModuleAPI.PlanarRegionsSegmentationParameters;
   private Topic<CustomRegionMergeParameters> customRegionMergeParametersTopic = REAModuleAPI.CustomRegionsMergingParameters;
   private Topic<ConcaveHullFactoryParameters> planarRegionsConcaveHullParametersTopic = REAModuleAPI.PlanarRegionsConcaveHullParameters;
   private Topic<PolygonizerParameters> planarRegionsPolygonizerParametersTopic = REAModuleAPI.PlanarRegionsPolygonizerParameters;
   private Topic<IntersectionEstimationParameters> planarRegionsIntersectionParametersTopic = REAModuleAPI.PlanarRegionsIntersectionParameters;
   private Topic<SurfaceNormalFilterParameters> surfaceNormalFilterParametersTopic = REAModuleAPI.SurfaceNormalFilterParameters;

   public REAPlanarRegionFeatureUpdater(Messager reaMessager)
   {
      this(reaMessager, REAModuleAPI.RequestEntireModuleState);
   }

   public REAPlanarRegionFeatureUpdater(Messager reaMessager, Topic<Boolean> requestEntireModuleStateTopic)
   {
      this.reaMessager = reaMessager;

      reaMessager.registerTopicListener(requestEntireModuleStateTopic, (messageContent) -> sendCurrentState());
   }

   public void bindControls()
   {
      isOcTreeEnabled = reaMessager.createInput(ocTreeEnableTopic, true);
      enableSegmentation = reaMessager.createInput(planarRegionsSegmentationEnableTopic, true);
      clearSegmentation = reaMessager.createInput(planarRegionsSegmentationClearTopic, false);
      enableCustomRegions = reaMessager.createInput(customRegionsMergingEnableTopic, true);
      clearCustomRegions = reaMessager.createInput(customRegionsClearTopic, false);
      enablePolygonizer = reaMessager.createInput(planarRegionsPolygonizerEnableTopic, true);
      clearPolygonizer = reaMessager.createInput(planarRegionsPolygonizerClearTopic, false);
      enableIntersectionCalulator = reaMessager.createInput(planarRegionsIntersectionEnableTopic, false);

      planarRegionSegmentationParameters = reaMessager.createInput(planarRegionsSegmentationParametersTopic, new PlanarRegionSegmentationParameters());
      customRegionMergingParameters = reaMessager.createInput(customRegionMergeParametersTopic, new CustomRegionMergeParameters());
      concaveHullFactoryParameters = reaMessager.createInput(planarRegionsConcaveHullParametersTopic, new ConcaveHullFactoryParameters());
      polygonizerParameters = reaMessager.createInput(planarRegionsPolygonizerParametersTopic, new PolygonizerParameters());
      intersectionEstimationParameters = reaMessager.createInput(planarRegionsIntersectionParametersTopic, new IntersectionEstimationParameters());
      surfaceNormalFilterParameters = reaMessager.createInput(surfaceNormalFilterParametersTopic, new SurfaceNormalFilterParameters());
   }

   public void setPlanarRegionSegmentationParameters(PlanarRegionSegmentationParameters planarRegionSegmentationParameters)
   {
      this.planarRegionSegmentationParameters.set(planarRegionSegmentationParameters);
   }

   public void setCustomRegionMergingParameters(CustomRegionMergeParameters customRegionMergingParameters)
   {
      this.customRegionMergingParameters.set(customRegionMergingParameters);
   }

   public void setConcaveHullFactoryParameters(ConcaveHullFactoryParameters concaveHullFactoryParameters)
   {
      this.concaveHullFactoryParameters.set(concaveHullFactoryParameters);
   }

   public void setPolygonizerParameters(PolygonizerParameters polygonizerParameters)
   {
      this.polygonizerParameters.set(polygonizerParameters);
   }

   public void setSurfaceNormalFilterParameters(SurfaceNormalFilterParameters surfaceNormalFilterParameters)
   {
      this.surfaceNormalFilterParameters.set(surfaceNormalFilterParameters);
   }

   public void setOcTreeEnableTopic(Topic<Boolean> ocTreeEnableTopic)
   {
      this.ocTreeEnableTopic = ocTreeEnableTopic;
   }

   public void setPlanarRegionSegmentationEnableTopic(Topic<Boolean> planarRegionsSegmentationEnableTopic)
   {
      this.planarRegionsSegmentationEnableTopic = planarRegionsSegmentationEnableTopic;
   }

   public void setPlanarRegionSegmentationClearTopic(Topic<Boolean> planarRegionsSegmentationClearTopic)
   {
      this.planarRegionsSegmentationClearTopic = planarRegionsSegmentationClearTopic;
   }

   public void setCustomRegionsMergingEnableTopic(Topic<Boolean> customRegionsMergingEnableTopic)
   {
      this.customRegionsMergingEnableTopic = customRegionsMergingEnableTopic;
   }

   public void setCustomRegionsClearTopic(Topic<Boolean> customRegionsClearTopic)
   {
      this.customRegionsClearTopic = customRegionsClearTopic;
   }

   public void setPlanarRegionsPolygonizerEnableTopic(Topic<Boolean> planarRegionsPolygonizerEnableTopic)
   {
      this.planarRegionsPolygonizerEnableTopic = planarRegionsPolygonizerEnableTopic;
   }

   public void setPlanarRegionsPolygonizerClearTopic(Topic<Boolean> planarRegionsPolygonizerClearTopic)
   {
      this.planarRegionsPolygonizerClearTopic = planarRegionsPolygonizerClearTopic;
   }

   public void setPlanarRegionsIntersectionEnableTopic(Topic<Boolean> planarRegionsIntersectionEnableTopic)
   {
      this.planarRegionsIntersectionEnableTopic = planarRegionsIntersectionEnableTopic;
   }

   public void setPlanarRegionSegmentationParametersTopic(Topic<PlanarRegionSegmentationParameters> planarRegionsSegmentationParametersTopic)
   {
      this.planarRegionsSegmentationParametersTopic = planarRegionsSegmentationParametersTopic;
   }

   public void setCustomRegionMergeParametersTopic(Topic<CustomRegionMergeParameters> customRegionMergeParametersTopic)
   {
      this.customRegionMergeParametersTopic = customRegionMergeParametersTopic;
   }

   public void setPlanarRegionsConcaveHullFactoryParametersTopic(Topic<ConcaveHullFactoryParameters> planarRegionsConcaveHullParametersTopic)
   {
      this.planarRegionsConcaveHullParametersTopic = planarRegionsConcaveHullParametersTopic;
   }

   public void setPlanarRegionsPolygonizerParametersTopic(Topic<PolygonizerParameters> planarRegionsPolygonizerParametersTopic)
   {
      this.planarRegionsPolygonizerParametersTopic = planarRegionsPolygonizerParametersTopic;
   }

   public void setPlanarRegionsIntersectionParametersTopic(Topic<IntersectionEstimationParameters> planarRegionsIntersectionParametersTopic)
   {
      this.planarRegionsIntersectionParametersTopic = planarRegionsIntersectionParametersTopic;
   }

   public void setSurfaceNormalFilterParametersTopic(Topic<SurfaceNormalFilterParameters> surfaceNormalFilterParametersTopic)
   {
      this.surfaceNormalFilterParametersTopic = surfaceNormalFilterParametersTopic;
   }

   private void sendCurrentState()
   {
      reaMessager.submitMessage(ocTreeEnableTopic, isOcTreeEnabled.get());
      reaMessager.submitMessage(planarRegionsSegmentationEnableTopic, enableSegmentation.get());
      reaMessager.submitMessage(customRegionsMergingEnableTopic, enableCustomRegions.get());
      reaMessager.submitMessage(planarRegionsPolygonizerEnableTopic, enablePolygonizer.get());
      reaMessager.submitMessage(planarRegionsIntersectionEnableTopic, enableIntersectionCalulator.get());

      reaMessager.submitMessage(planarRegionsSegmentationParametersTopic, planarRegionSegmentationParameters.get());
      reaMessager.submitMessage(customRegionMergeParametersTopic, customRegionMergingParameters.get());
      reaMessager.submitMessage(planarRegionsConcaveHullParametersTopic, concaveHullFactoryParameters.get());
      reaMessager.submitMessage(planarRegionsPolygonizerParametersTopic, polygonizerParameters.get());
      reaMessager.submitMessage(planarRegionsIntersectionParametersTopic, intersectionEstimationParameters.get());
      reaMessager.submitMessage(surfaceNormalFilterParametersTopic, surfaceNormalFilterParameters.get());
   }

   public void loadConfiguration(FilePropertyHelper filePropertyHelper)
   {
      Boolean enableFile = filePropertyHelper.loadBooleanProperty(ocTreeEnableTopic.getName());
      if (enableFile != null)
         isOcTreeEnabled.set(enableFile);
      Boolean enableSegmentationFile = filePropertyHelper.loadBooleanProperty(planarRegionsSegmentationEnableTopic.getName());
      if (enableSegmentationFile != null)
         enableSegmentation.set(enableSegmentationFile);
      Boolean enableCustomRegionsFile = filePropertyHelper.loadBooleanProperty(customRegionsMergingEnableTopic.getName());
      if (enableCustomRegionsFile != null)
         enableCustomRegions.set(enableCustomRegionsFile);
      Boolean enablePolygonizerFile = filePropertyHelper.loadBooleanProperty(planarRegionsPolygonizerEnableTopic.getName());
      if (enablePolygonizerFile != null)
         enablePolygonizer.set(enablePolygonizerFile);
      Boolean enableIntersectionCalulatorFile = filePropertyHelper.loadBooleanProperty(planarRegionsIntersectionEnableTopic.getName());
      if (enableIntersectionCalulatorFile != null)
         enableIntersectionCalulator.set(enableIntersectionCalulatorFile);

      String planarRegionSegmentationParametersFile = filePropertyHelper.loadProperty(planarRegionsSegmentationParametersTopic.getName());
      if (planarRegionSegmentationParametersFile != null)
         planarRegionSegmentationParameters.set(PlanarRegionSegmentationParameters.parse(planarRegionSegmentationParametersFile));

      String customRegionMergingParametersFile = filePropertyHelper.loadProperty(customRegionMergeParametersTopic.getName());
      if (customRegionMergingParametersFile != null)
         customRegionMergingParameters.set(CustomRegionMergeParameters.parse(customRegionMergingParametersFile));

      String planarRegionConcaveHullFactoryParametersFile = filePropertyHelper.loadProperty(planarRegionsConcaveHullParametersTopic.getName());
      if (planarRegionConcaveHullFactoryParametersFile != null)
         concaveHullFactoryParameters.set(ConcaveHullFactoryParameters.parse(planarRegionConcaveHullFactoryParametersFile));

      String polygonizerParametersFile = filePropertyHelper.loadProperty(planarRegionsPolygonizerParametersTopic.getName());
      if (polygonizerParametersFile != null)
         polygonizerParameters.set(PolygonizerParameters.parse(polygonizerParametersFile));

      String intersectionEstimationParametersFile = filePropertyHelper.loadProperty(planarRegionsIntersectionParametersTopic.getName());
      if (intersectionEstimationParametersFile != null)
         intersectionEstimationParameters.set(IntersectionEstimationParameters.parse(intersectionEstimationParametersFile));

      String surfaceNormalFiltersParameterFile = filePropertyHelper.loadProperty(surfaceNormalFilterParametersTopic.getName());
      if (surfaceNormalFiltersParameterFile != null)
         surfaceNormalFilterParameters.set(SurfaceNormalFilterParameters.parse(surfaceNormalFiltersParameterFile));
   }

   public void saveConfiguration(FilePropertyHelper filePropertyHelper)
   {
      filePropertyHelper.saveProperty(planarRegionsSegmentationEnableTopic.getName(), enableSegmentation.get());
      filePropertyHelper.saveProperty(customRegionsMergingEnableTopic.getName(), enableCustomRegions.get());
      filePropertyHelper.saveProperty(planarRegionsPolygonizerEnableTopic.getName(), enablePolygonizer.get());
      filePropertyHelper.saveProperty(planarRegionsIntersectionEnableTopic.getName(), enableIntersectionCalulator.get());

      filePropertyHelper.saveProperty(planarRegionsSegmentationParametersTopic.getName(), planarRegionSegmentationParameters.get().toString());
      filePropertyHelper.saveProperty(customRegionMergeParametersTopic.getName(), customRegionMergingParameters.get().toString());
      filePropertyHelper.saveProperty(planarRegionsConcaveHullParametersTopic.getName(), concaveHullFactoryParameters.get().toString());
      filePropertyHelper.saveProperty(planarRegionsPolygonizerParametersTopic.getName(), polygonizerParameters.get().toString());
      filePropertyHelper.saveProperty(planarRegionsIntersectionParametersTopic.getName(), intersectionEstimationParameters.get().toString());
      filePropertyHelper.saveProperty(surfaceNormalFilterParametersTopic.getName(), surfaceNormalFilterParameters.get().toString());
   }

   public void update(NormalOcTree octree, Tuple3DReadOnly sensorPosition)
   {
      if (!isOcTreeEnabled.get())
         return;

      if (clearSegmentation.getAndSet(false))
      {
         segmentationCalculator.clear();
         return;
      }

      if (!enableSegmentation.get())
      {
         segmentationCalculator.removeDeadNodes();
         return;
      }

      segmentationCalculator.setBoundingBox(octree.getBoundingBox());
      segmentationCalculator.setParameters(planarRegionSegmentationParameters.get());
      segmentationCalculator.setSurfaceNormalFilterParameters(surfaceNormalFilterParameters.get());
      segmentationCalculator.setSensorPosition(sensorPosition);

      timeReporter.run(() -> segmentationCalculator.compute(octree.getRoot()), segmentationTimeReport);

      List<PlanarRegionSegmentationRawData> rawData = segmentationCalculator.getSegmentationRawData();

      List<PlanarRegion> unmergedCustomPlanarRegions;

      if (clearCustomRegions.getAndSet(false))
      {
         customPlanarRegions.clear();
         unmergedCustomPlanarRegions = Collections.emptyList();
      }
      else if (enableCustomRegions.get())
      {
         unmergedCustomPlanarRegions = CustomPlanarRegionHandler.mergeCustomRegionsToEstimatedRegions(customPlanarRegions.valueCollection(), rawData,
                                                                                                      customRegionMergingParameters.get());
      }
      else
      {
         unmergedCustomPlanarRegions = Collections.emptyList();
      }

      if (enableIntersectionCalulator.get())
         timeReporter.run(() -> updateIntersections(rawData), intersectionsTimeReport);

      if (clearPolygonizer.getAndSet(false))
      {
         planarRegionsList = null;
      }
      else if (enablePolygonizer.get())
      {
         timeReporter.run(() -> updatePolygons(rawData), segmentationTimeReport);
         if (planarRegionsList != null)
            unmergedCustomPlanarRegions.forEach(planarRegionsList::addPlanarRegion);
      }
   }

   public void registerCustomPlanarRegion(PlanarRegion planarRegion)
   {
      if (planarRegion.getRegionId() == PlanarRegion.NO_REGION_ID)
      {
         // ignore if region id isn't set
         return;
      }
      else if (planarRegion.isEmpty())
      {
         customPlanarRegions.remove(planarRegion.getRegionId());
      }
      else
      {
         CustomPlanarRegionHandler.performConvexDecompositionIfNeeded(planarRegion);
         customPlanarRegions.put(planarRegion.getRegionId(), planarRegion);
      }
   }

   public void clearOcTree()
   {
      segmentationCalculator.clear();
   }

   private void updatePolygons(List<PlanarRegionSegmentationRawData> rawData)
   {
      ConcaveHullFactoryParameters concaveHullFactoryParameters = this.concaveHullFactoryParameters.get();
      PolygonizerParameters polygonizerParameters = this.polygonizerParameters.get();

      if (EXPORT_SEGMENTATION_ON_EXCEPTION)
         planarRegionsList = PlanarRegionPolygonizer.createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters, dataExporter);
      else
         planarRegionsList = PlanarRegionPolygonizer.createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters);
   }

   private void updateIntersections(List<PlanarRegionSegmentationRawData> rawData)
   {
      planarRegionsIntersections = PlanarRegionIntersectionCalculator.computeIntersections(rawData, intersectionEstimationParameters.get());
   }

   @Override
   public List<PlanarRegionSegmentationNodeData> getSegmentationNodeData()
   {
      return segmentationCalculator.getSegmentationNodeData();
   }

   @Override
   public PlanarRegionsList getPlanarRegionsList()
   {
      return planarRegionsList;
   }

   @Override
   public int getNumberOfPlaneIntersections()
   {
      return planarRegionsIntersections == null ? 0 : planarRegionsIntersections.size();
   }

   @Override
   public LineSegment3D getIntersection(int index)
   {
      return planarRegionsIntersections.get(index);
   }
}
