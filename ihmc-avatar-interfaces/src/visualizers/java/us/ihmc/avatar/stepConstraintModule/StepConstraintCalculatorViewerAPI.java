package us.ihmc.avatar.stepConstraintModule;

import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotics.RegionInWorldInterface;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DBasics;

import java.util.HashMap;
import java.util.List;



public class StepConstraintCalculatorViewerAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final Category Root = apiFactory.createRootCategory("Constraint");
   private static final CategoryTheme StepConstraint = apiFactory.createCategoryTheme("StepConstraint");

   public static final Topic<PlanarRegionsList> PlanarRegionData = topic("PlanarRegionData");
   public static final Topic<PlanarRegionsList> TooSteepRegionData = topic("TooSteepRegionData");
   public static final Topic<PlanarRegionsList> TooSmallRegionData = topic("TooSmallRegionData");
   public static final Topic<PlanarRegionsList> MaskedRegionsData = topic("MaskedRegionsData");
   public static final Topic<List<StepConstraintRegion>> StepConstraintRegionData = topic("StepConstraintRegionData");
   public static final Topic<HashMap<RegionInWorldInterface, List<ConcavePolygon2DBasics>>> ObstacleExtrusionsData = topic("ObstacleExtrusionsData");
   public static final Topic<HashMap<RegionInWorldInterface, List<ConcavePolygon2DBasics>>> MaskedRegionsObstacleExtrusionsData = topic("MaskedRegionsObstacleExtrusionsData");

   public static final Topic<Boolean> ShowPlanarRegions = topic("ShowPlanarRegions");
   public static final Topic<Boolean> ShowTooSteepRegions = topic("ShowTooSteepRegions");
   public static final Topic<Boolean> ShowTooSmallRegions = topic("ShowTooSmallRegions");
   public static final Topic<Boolean> ShowStepConstraintRegions = topic("ShowStepConstraintRegions");
   public static final Topic<Boolean> ShowObstacleExtrusions = topic("ShowObstacleExtrusions");
   public static final Topic<Boolean> ShowExtrusionPoints = topic("ShowExtrusionPoints");
   public static final Topic<Boolean> ShowMaskedRegions = topic("ShowMaskedRegions");

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();

   private static final <T> MessagerAPIFactory.Topic<T> topic(String name)
   {
      return Root.child(StepConstraint).topic(apiFactory.createTypedTopicTheme(name));
   }
}
