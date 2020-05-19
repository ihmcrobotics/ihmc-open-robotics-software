package us.ihmc.avatar.networkProcessor.stepConstraintToolboxModule;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class SteppableRegionsCalculator
{
   private static final double maxNormalAngleFromVertical = 0.3;
   private static final double minimumAreaToConsider = 0.01;

   private static final Vector3D verticalAxis = new Vector3D(0.0, 0.0, 1.0);

   private final YoDouble maxAngleForSteppable;
   private final YoDouble minimumAreaForSteppable;

   private List<PlanarRegion> steppableRegions = new ArrayList<>();
   private List<PlanarRegion> allPlanarRegions = new ArrayList<>();

   public SteppableRegionsCalculator(YoVariableRegistry registry)
   {
      maxAngleForSteppable = new YoDouble("maxAngleForSteppable", registry);
      minimumAreaForSteppable = new YoDouble("minimumAreaForSteppable", registry);
      maxAngleForSteppable.set(maxNormalAngleFromVertical);
      minimumAreaForSteppable.set(minimumAreaToConsider);
   }

   public void setPlanarRegions(List<PlanarRegion> planarRegions)
   {
      allPlanarRegions = planarRegions;
      steppableRegions = planarRegions.stream().filter(this::isRegionValidForStepping).collect(Collectors.toList());
   }

   public List<PlanarRegion> computeSteppableRegions()
   {
      return steppableRegions;
   }

   private boolean isRegionValidForStepping(PlanarRegion planarRegion)
   {
      double angle = planarRegion.getNormal().angle(verticalAxis);

      if (angle > maxAngleForSteppable.getValue())
         return false;

      return PlanarRegionTools.computePlanarRegionArea(planarRegion) > minimumAreaForSteppable.getValue();
   }
}
