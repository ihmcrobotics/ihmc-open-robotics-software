package us.ihmc.avatar.stepAdjustment;

import controller_msgs.msg.dds.StepConstraintsListMessage;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.StepConstraintRegionCalculator;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintListConverter;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintMessageConverter;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.lists.QuickSort;
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class PlanarRegionStepConstraintCalculator implements StepConstraintRegionCalculator
{
   private final List<PlanarRegion> planarRegionsList = new ArrayList<>();
   private final List<PlanarRegion> regionsToInclude = new ArrayList<>();
   private final RecyclingArrayList<StepConstraintRegion> steppableRegions = new RecyclingArrayList<>(StepConstraintRegion::new);

   private static final double extraReachRatioToInclude = 1.5;
   private final IntegerParameter numberOfRegionsToInclude;

   private final SteppingParameters steppingParameters;
   private final Comparator<PlanarRegion> areaComparator = Comparator.comparingDouble(region -> region.getConvexHull().getArea());

   public PlanarRegionStepConstraintCalculator(SteppingParameters steppingParameters, YoRegistry registry)
   {
      this.steppingParameters = steppingParameters;
      numberOfRegionsToInclude = new IntegerParameter("numberOfRegionsToIncludeAsConstraints", registry, 5);
   }

   public void setPlanarRegions(List<PlanarRegion> planarRegions)
   {
      planarRegionsList.clear();
      for (int i = 0; i < planarRegions.size(); i++)
         planarRegionsList.add(planarRegions.get(i));
   }

   private final Point3D pointToPack = new Point3D();

   @Override
   public void computeConstraintRegions(FramePose3DReadOnly stanceFootPose, FramePose3DReadOnly footstepPose, StepConstraintsListMessage stepConstraintsToPack)
   {
      double distanceToInclude = extraReachRatioToInclude * steppingParameters.getMaxStepLength();

      regionsToInclude.clear();
      for (int i = 0; i < planarRegionsList.size(); i++)
      {
         PlanarRegionTools.closestPointOnPlanarRegion(stanceFootPose.getPosition(), planarRegionsList.get(i), pointToPack);
         if (pointToPack.distance(stanceFootPose.getPosition()) > distanceToInclude)
            regionsToInclude.add(planarRegionsList.get(i));
      }

      QuickSort.sort(regionsToInclude, areaComparator);

      while (regionsToInclude.size() > numberOfRegionsToInclude.getValue())
         regionsToInclude.remove(0);

      StepConstraintListConverter.convertPlanarRegionListToStepConstraintRegion(regionsToInclude, steppableRegions);
      StepConstraintMessageConverter.convertToStepConstraintsListMessage(steppableRegions, stepConstraintsToPack);
   }
}
