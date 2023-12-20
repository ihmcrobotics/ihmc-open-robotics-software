package us.ihmc.footstepPlanning;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsTrajectoryExpander;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointSwingGenerator;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.swing.AdaptiveSwingTrajectoryCalculator;
import us.ihmc.footstepPlanning.swing.CollisionFreeSwingCalculator;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SwingPlanningModule
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final FootstepPlannerParametersReadOnly footstepPlannerParameters;
   private final SwingPlannerParametersBasics swingPlannerParameters;
   private final WalkingControllerParameters walkingControllerParameters;

   private final AdaptiveSwingTrajectoryCalculator adaptiveSwingTrajectoryCalculator;
   private final SwingOverPlanarRegionsTrajectoryExpander swingOverPlanarRegionsTrajectoryExpander;
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
         this.swingOverPlanarRegionsTrajectoryExpander = null;
         this.collisionFreeSwingCalculator = null;
      }
      else
      {
         this.adaptiveSwingTrajectoryCalculator = new AdaptiveSwingTrajectoryCalculator(swingPlannerParameters,
                                                                                        footstepPlannerParameters,
                                                                                        walkingControllerParameters);
         this.swingOverPlanarRegionsTrajectoryExpander = new SwingOverPlanarRegionsTrajectoryExpander(walkingControllerParameters,
                                                                                                      registry,
                                                                                                      new YoGraphicsListRegistry());
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

      if (swingPlannerType == SwingPlannerType.PROPORTION && adaptiveSwingTrajectoryCalculator != null && false)
      {
         // TODO need to make work with the height m ap.
//         adaptiveSwingTrajectoryCalculator.setPlanarRegionsList(planarRegionsList);
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

   public SwingOverPlanarRegionsTrajectoryExpander getSwingOverPlanarRegionsTrajectoryExpander()
   {
      return swingOverPlanarRegionsTrajectoryExpander;
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
