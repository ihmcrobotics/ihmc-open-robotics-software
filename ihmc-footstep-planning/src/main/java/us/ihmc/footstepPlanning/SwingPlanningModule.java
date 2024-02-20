package us.ihmc.footstepPlanning;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.swing.AdaptiveSwingTrajectoryCalculator;
import us.ihmc.footstepPlanning.swing.CollisionFreeSwingCalculator;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialReadOnly;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

public class SwingPlanningModule
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final FootstepPlannerParametersReadOnly footstepPlannerParameters;
   private final SwingPlannerParametersBasics swingPlannerParameters;
   private final WalkingControllerParameters walkingControllerParameters;

   private final AdaptiveSwingTrajectoryCalculator adaptiveSwingTrajectoryCalculator;
   private final CollisionFreeSwingCalculator collisionFreeSwingCalculator;

   private double nominalSwingTrajectoryLength;

   private final List<EnumMap<Axis3D, List<PolynomialReadOnly>>> swingTrajectories = new ArrayList<>();


   public SwingPlanningModule(FootstepPlannerParametersReadOnly footstepPlannerParameters,
                              SwingPlannerParametersBasics swingPlannerParameters,
                              WalkingControllerParameters walkingControllerParameters,
                              SideDependentList<ConvexPolygon2D> footPolygons)

   {
      this.footstepPlannerParameters = footstepPlannerParameters;
      this.swingPlannerParameters = swingPlannerParameters;
      this.walkingControllerParameters = walkingControllerParameters;

      if (walkingControllerParameters == null)
      {
         this.adaptiveSwingTrajectoryCalculator = null;
         this.collisionFreeSwingCalculator = null;
      }
      else
      {
         this.adaptiveSwingTrajectoryCalculator = new AdaptiveSwingTrajectoryCalculator(swingPlannerParameters,
                                                                                        footstepPlannerParameters,
                                                                                        walkingControllerParameters);
         this.collisionFreeSwingCalculator = new CollisionFreeSwingCalculator(footstepPlannerParameters,
                                                                              swingPlannerParameters,
                                                                              walkingControllerParameters,
                                                                              footPolygons);
      }
   }

   public YoRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public void computeSwingWaypoints(HeightMapData heightMapData,
                                     FootstepPlan footstepPlan,
                                     SideDependentList<? extends Pose3DReadOnly> startFootPoses,
                                     SwingPlannerType swingPlannerType)
   {
      swingTrajectories.clear();
      if (heightMapData == null || heightMapData.isEmpty())
      {
         return;
      }

      if (swingPlannerType == SwingPlannerType.PROPORTION && adaptiveSwingTrajectoryCalculator != null)
      {
         adaptiveSwingTrajectoryCalculator.setHeightMapData(heightMapData);
         adaptiveSwingTrajectoryCalculator.setSwingParameters(startFootPoses, footstepPlan);
         for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
            swingTrajectories.add(null);
      }
      else if (swingPlannerType == SwingPlannerType.MULTI_WAYPOINT_POSITION && collisionFreeSwingCalculator != null)
      {
         collisionFreeSwingCalculator.setHeightMapData(heightMapData);
         collisionFreeSwingCalculator.computeSwingTrajectories(startFootPoses, footstepPlan);
         swingTrajectories.addAll(collisionFreeSwingCalculator.getSwingTrajectories());
      }
   }

   public SwingPlannerParametersBasics getSwingPlannerParameters()
   {
      return swingPlannerParameters;
   }

   public AdaptiveSwingTrajectoryCalculator getAdaptiveSwingTrajectoryCalculator()
   {
      return adaptiveSwingTrajectoryCalculator;
   }

   public CollisionFreeSwingCalculator getCollisionFreeSwingCalculator()
   {
      return collisionFreeSwingCalculator;
   }

   public List<EnumMap<Axis3D, List<PolynomialReadOnly>>> getSwingTrajectories()
   {
      return swingTrajectories;
   }
}
