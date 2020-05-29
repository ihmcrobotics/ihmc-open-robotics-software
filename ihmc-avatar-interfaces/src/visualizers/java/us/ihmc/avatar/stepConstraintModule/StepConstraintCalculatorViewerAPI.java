package us.ihmc.avatar.stepConstraintModule;

import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.List;

public class StepConstraintCalculatorViewerAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final MessagerAPIFactory.Category Root = apiFactory.createRootCategory("Constraint");
   private static final MessagerAPIFactory.CategoryTheme StepConstraint = apiFactory.createCategoryTheme("StepConstraint");

   public static final MessagerAPIFactory.Topic<PlanarRegionsList> PlanarRegionData = topic("PlanarRegionData");
   public static final MessagerAPIFactory.Topic<List<StepConstraintRegion>> StepConstraintRegionData = topic("StepConstraintRegionData");

   public static final MessagerAPIFactory.Topic<Boolean> ShowPlanarRegions = topic("ShowPlanarRegions");
   public static final MessagerAPIFactory.Topic<Boolean> ShowStepConstraintRegions = topic("ShowStepConstraintRegions");

   public static final MessagerAPIFactory.MessagerAPI API = apiFactory.getAPIAndCloseFactory();

   private static final <T> MessagerAPIFactory.Topic<T> topic(String name)
   {
      return Root.child(StepConstraint).topic(apiFactory.createTypedTopicTheme(name));
   }
}
