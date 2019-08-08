package us.ihmc.robotEnvironmentAwareness.updaters;

import gnu.trove.map.hash.TIntObjectHashMap;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;
import us.ihmc.robotEnvironmentAwareness.planarRegion.*;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionSegmentationDataExporter;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.io.File;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class REAPlanarRegionFeatureUpdater implements RegionFeaturesProvider
{
   private static final boolean EXPORT_SEGMENTATION_ON_EXCEPTION = false;

   private static final String segmentationTimeReport = "Segmentation took: ";
   private static final String intersectionsTimeReport = "Processing intersections took: ";

   private final PlanarRegionSegmentationDataExporter dataExporter = EXPORT_SEGMENTATION_ON_EXCEPTION
         ? new PlanarRegionSegmentationDataExporter(new File("DataThrowingException/Segmentation"))
         : null;

   private final TimeReporter timeReporter = new TimeReporter();
   private final NormalOcTree octree;

   private final PlanarRegionSegmentationCalculator segmentationCalculator = new PlanarRegionSegmentationCalculator();

   private final TIntObjectHashMap<PlanarRegion> customPlanarRegions = new TIntObjectHashMap<>();

   private PlanarRegionsList planarRegionsList = null;
   private List<LineSegment3D> planarRegionsIntersections = null;

   private final AtomicReference<Boolean> isOcTreeEnabled;
   private final AtomicReference<Boolean> enableSegmentation;
   private final AtomicReference<Boolean> clearSegmentation;
   private final AtomicReference<Boolean> enableCustomRegions;
   private final AtomicReference<Boolean> clearCustomRegions;
   private final AtomicReference<Boolean> enablePolygonizer;
   private final AtomicReference<Boolean> clearPolygonizer;
   private final AtomicReference<Boolean> enableIntersectionCalulator;
   private final AtomicReference<PlanarRegionSegmentationParameters> planarRegionSegmentationParameters;
   private final AtomicReference<CustomRegionMergeParameters> customRegionMergingParameters;
   private final AtomicReference<ConcaveHullFactoryParameters> concaveHullFactoryParameters;
   private final AtomicReference<PolygonizerParameters> polygonizerParameters;
   private final AtomicReference<IntersectionEstimationParameters> intersectionEstimationParameters;
   private final Messager reaMessager;

   public REAPlanarRegionFeatureUpdater(NormalOcTree octree, Messager reaMessager)
   {
      this.octree = octree;
      this.reaMessager = reaMessager;

      isOcTreeEnabled = reaMessager.createInput(REAModuleAPI.OcTreeEnable, true);
      enableSegmentation = reaMessager.createInput(REAModuleAPI.PlanarRegionsSegmentationEnable, true);
      clearSegmentation = reaMessager.createInput(REAModuleAPI.PlanarRegionsSegmentationClear, false);
      enableCustomRegions = reaMessager.createInput(REAModuleAPI.CustomRegionsMergingEnable, true);
      clearCustomRegions = reaMessager.createInput(REAModuleAPI.CustomRegionsClear, false);
      enablePolygonizer = reaMessager.createInput(REAModuleAPI.PlanarRegionsPolygonizerEnable, true);
      clearPolygonizer = reaMessager.createInput(REAModuleAPI.PlanarRegionsPolygonizerClear, false);
      enableIntersectionCalulator = reaMessager.createInput(REAModuleAPI.PlanarRegionsIntersectionEnable, false);
      planarRegionSegmentationParameters = reaMessager.createInput(REAModuleAPI.PlanarRegionsSegmentationParameters, new PlanarRegionSegmentationParameters());
      customRegionMergingParameters = reaMessager.createInput(REAModuleAPI.CustomRegionsMergingParameters, new CustomRegionMergeParameters());
      concaveHullFactoryParameters = reaMessager.createInput(REAModuleAPI.PlanarRegionsConcaveHullParameters, new ConcaveHullFactoryParameters());
      polygonizerParameters = reaMessager.createInput(REAModuleAPI.PlanarRegionsPolygonizerParameters, new PolygonizerParameters());
      intersectionEstimationParameters = reaMessager.createInput(REAModuleAPI.PlanarRegionsIntersectionParameters, new IntersectionEstimationParameters());

      reaMessager.registerTopicListener(REAModuleAPI.RequestEntireModuleState, (messageContent) -> sendCurrentState());
   }

   private void sendCurrentState()
   {
      reaMessager.submitMessage(REAModuleAPI.PlanarRegionsSegmentationEnable, enableSegmentation.get());
      reaMessager.submitMessage(REAModuleAPI.CustomRegionsMergingEnable, enableCustomRegions.get());
      reaMessager.submitMessage(REAModuleAPI.PlanarRegionsPolygonizerEnable, enablePolygonizer.get());
      reaMessager.submitMessage(REAModuleAPI.PlanarRegionsIntersectionEnable, enableIntersectionCalulator.get());

      reaMessager.submitMessage(REAModuleAPI.PlanarRegionsSegmentationParameters, planarRegionSegmentationParameters.get());
      reaMessager.submitMessage(REAModuleAPI.CustomRegionsMergingParameters, customRegionMergingParameters.get());
      reaMessager.submitMessage(REAModuleAPI.PlanarRegionsConcaveHullParameters, concaveHullFactoryParameters.get());
      reaMessager.submitMessage(REAModuleAPI.PlanarRegionsPolygonizerParameters, polygonizerParameters.get());
      reaMessager.submitMessage(REAModuleAPI.PlanarRegionsIntersectionParameters, intersectionEstimationParameters.get());
   }

   public void loadConfiguration(FilePropertyHelper filePropertyHelper)
   {
      Boolean enableFile = filePropertyHelper.loadBooleanProperty(REAModuleAPI.OcTreeEnable.getName());
      if (enableFile != null)
         isOcTreeEnabled.set(enableFile);
      Boolean enableSegmentationFile = filePropertyHelper.loadBooleanProperty(REAModuleAPI.PlanarRegionsSegmentationEnable.getName());
      if (enableSegmentationFile != null)
         enableSegmentation.set(enableSegmentationFile);
      Boolean enableCustomRegionsFile = filePropertyHelper.loadBooleanProperty(REAModuleAPI.CustomRegionsMergingEnable.getName());
      if (enableCustomRegionsFile != null)
         enableCustomRegions.set(enableCustomRegionsFile);
      Boolean enablePolygonizerFile = filePropertyHelper.loadBooleanProperty(REAModuleAPI.PlanarRegionsPolygonizerEnable.getName());
      if (enablePolygonizerFile != null)
         enablePolygonizer.set(enablePolygonizerFile);
      Boolean enableIntersectionCalulatorFile = filePropertyHelper.loadBooleanProperty(REAModuleAPI.PlanarRegionsIntersectionEnable.getName());
      if (enableIntersectionCalulatorFile != null)
         enableIntersectionCalulator.set(enableIntersectionCalulatorFile);

      String planarRegionSegmentationParametersFile = filePropertyHelper.loadProperty(REAModuleAPI.PlanarRegionsSegmentationParameters.getName());
      if (planarRegionSegmentationParametersFile != null)
         planarRegionSegmentationParameters.set(PlanarRegionSegmentationParameters.parse(planarRegionSegmentationParametersFile));

      String customRegionMergingParametersFile = filePropertyHelper.loadProperty(REAModuleAPI.CustomRegionsMergingParameters.getName());
      if (customRegionMergingParametersFile != null)
         customRegionMergingParameters.set(CustomRegionMergeParameters.parse(customRegionMergingParametersFile));

      String planarRegionConcaveHullFactoryParametersFile = filePropertyHelper.loadProperty(REAModuleAPI.PlanarRegionsConcaveHullParameters.getName());
      if (planarRegionConcaveHullFactoryParametersFile != null)
         concaveHullFactoryParameters.set(ConcaveHullFactoryParameters.parse(planarRegionConcaveHullFactoryParametersFile));

      String polygonizerParametersFile = filePropertyHelper.loadProperty(REAModuleAPI.PlanarRegionsPolygonizerParameters.getName());
      if (polygonizerParametersFile != null)
         polygonizerParameters.set(PolygonizerParameters.parse(polygonizerParametersFile));

      String intersectionEstimationParametersFile = filePropertyHelper.loadProperty(REAModuleAPI.PlanarRegionsIntersectionParameters.getName());
      if (intersectionEstimationParametersFile != null)
         intersectionEstimationParameters.set(IntersectionEstimationParameters.parse(intersectionEstimationParametersFile));
   }

   public void saveConfiguration(FilePropertyHelper filePropertyHelper)
   {
      filePropertyHelper.saveProperty(REAModuleAPI.PlanarRegionsSegmentationEnable.getName(), enableSegmentation.get());
      filePropertyHelper.saveProperty(REAModuleAPI.CustomRegionsMergingEnable.getName(), enableCustomRegions.get());
      filePropertyHelper.saveProperty(REAModuleAPI.PlanarRegionsPolygonizerEnable.getName(), enablePolygonizer.get());
      filePropertyHelper.saveProperty(REAModuleAPI.PlanarRegionsIntersectionEnable.getName(), enableIntersectionCalulator.get());

      filePropertyHelper.saveProperty(REAModuleAPI.PlanarRegionsSegmentationParameters.getName(), planarRegionSegmentationParameters.get().toString());
      filePropertyHelper.saveProperty(REAModuleAPI.CustomRegionsMergingParameters.getName(), customRegionMergingParameters.get().toString());
      filePropertyHelper.saveProperty(REAModuleAPI.PlanarRegionsConcaveHullParameters.getName(), concaveHullFactoryParameters.get().toString());
      filePropertyHelper.saveProperty(REAModuleAPI.PlanarRegionsPolygonizerParameters.getName(), polygonizerParameters.get().toString());
      filePropertyHelper.saveProperty(REAModuleAPI.PlanarRegionsIntersectionParameters.getName(), intersectionEstimationParameters.get().toString());
   }

   public void update()
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
      if(planarRegion.getRegionId() == PlanarRegion.NO_REGION_ID)
      {
         // ignore if region id isn't set
         return;
      }
      else if(planarRegion.isEmpty())
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
