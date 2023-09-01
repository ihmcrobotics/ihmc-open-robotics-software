package us.ihmc.avatar.stepAdjustment;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.ConstraintOptimizerParametersReadOnly;
import us.ihmc.commonWalkingControlModules.configurations.SteppingEnvironmentalConstraintParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepPlanAdjustment;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.footstepPlanning.polygonSnapping.GarbageFreePlanarRegionListPolygonSnapper;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * This class is designed to use the {@link PlanarRegionFootstepSnapper} Footstep Adjustment algorithm to snap the entire plan. This snapper uses planar regions
 * to snap the desired foothodls.
 */
public class PlanarRegionFootstepPlanSnapper implements FootstepPlanAdjustment
{
   private final PlanarRegionFootstepSnapper snapper;

   // callback function to enable visualization
   private PlanarRegionSnapVisualizer planarRegionSnapVisualizer;

   // temp variables for calculation
   private final FramePose2D unsnappedPose = new FramePose2D();
   private final FramePose3D snappedPose = new FramePose3D();

   public PlanarRegionFootstepPlanSnapper(SideDependentList<ConvexPolygon2D> footPolygons,
                                          SteppableRegionsProvider steppableRegionsProvider,
                                          SteppingEnvironmentalConstraintParameters constraintOptimizerParameters,
                                          YoRegistry parentRegistry)
   {
      snapper = new PlanarRegionFootstepSnapper(footPolygons, steppableRegionsProvider, constraintOptimizerParameters, parentRegistry);
   }

   public void attachPlanarRegionSnapVisualizer(PlanarRegionSnapVisualizer planarRegionSnapperCallback)
   {
      this.planarRegionSnapVisualizer = planarRegionSnapperCallback;
      snapper.attachPlanarRegionSnapVisualizer(planarRegionSnapperCallback);
   }

   /** {@inheritDoc} **/
   @Override
   public void adjustFootstepPlan(FramePose3DReadOnly stanceFootPose, int footstepIndexToStart, FootstepDataListMessage footstepDataListMessageToAdjust)
   {
      for (int index = footstepIndexToStart; index < footstepDataListMessageToAdjust.getFootstepDataList().size(); index++)
      {
         if (planarRegionSnapVisualizer != null)
         {
            // used to set the index for the data container being visualized
            planarRegionSnapVisualizer.setFootIndex(index);
         }

         FootstepDataMessage dataMessage = footstepDataListMessageToAdjust.getFootstepDataList().get(index);

         unsnappedPose.getPosition().set(dataMessage.getLocation());
         unsnappedPose.getOrientation().set(dataMessage.getOrientation());

         RobotSide side = RobotSide.fromByte(dataMessage.getRobotSide());

         snapper.adjustFootstep(stanceFootPose, unsnappedPose, side, dataMessage);
      }
   }
}
