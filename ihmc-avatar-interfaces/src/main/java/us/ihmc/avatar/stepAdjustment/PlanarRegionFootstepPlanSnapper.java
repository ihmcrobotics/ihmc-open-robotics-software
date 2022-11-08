package us.ihmc.avatar.stepAdjustment;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.ConvexStepConstraintOptimizer;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.YoConstraintOptimizerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepAdjustment;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepPlanAdjustment;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.SteppableRegionsProvider;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DBasics;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.polygonSnapping.GarbageFreePlanarRegionListPolygonSnapper;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.ArrayList;
import java.util.List;

public class PlanarRegionFootstepPlanSnapper implements FootstepPlanAdjustment
{
   private PlanarRegionSnapperCallback planarRegionSnapperCallback;
   private final PlanarRegionFootstepSnapper snapper;

   public PlanarRegionFootstepPlanSnapper(SideDependentList<ConvexPolygon2D> footPolygons, SteppableRegionsProvider steppableRegionsProvider)
   {
      snapper = new PlanarRegionFootstepSnapper(footPolygons, steppableRegionsProvider);
   }

   public YoRegistry getRegistry()
   {
      return snapper.getRegistry();
   }

   public GarbageFreePlanarRegionListPolygonSnapper getSnapper()
   {
      return snapper.getSnapper();
   }

   public void attachPlanarRegionSnapperCallback(PlanarRegionSnapperCallback planarRegionSnapperCallback)
   {
      this.planarRegionSnapperCallback = planarRegionSnapperCallback;
      snapper.attachPlanarRegionSnapperCallback(planarRegionSnapperCallback);
   }

   public ConvexPolygon2DReadOnly getFootPolygon(RobotSide robotSide)
   {
      return snapper.getFootPolygon(robotSide);
   }

   private final FramePose2D unsnappedPose = new FramePose2D();
   private final FramePose3D snappedPose = new FramePose3D();

   @Override
   public void adjustFootstepPlan(FramePose3DReadOnly stanceFootPose, int footstepIndexToStart, FootstepDataListMessage footstepDataListMessageToAdjust)
   {
      for (int index = footstepIndexToStart; index < footstepDataListMessageToAdjust.getFootstepDataList().size(); index++)
      {
         planarRegionSnapperCallback.setFootIndex(index);
         FootstepDataMessage dataMessage = footstepDataListMessageToAdjust.getFootstepDataList().get(index);

         unsnappedPose.getPosition().set(dataMessage.getLocation());
         unsnappedPose.getOrientation().set(dataMessage.getOrientation());

         RobotSide side = RobotSide.fromByte(dataMessage.getRobotSide());

         if (snapper.adjustFootstep(stanceFootPose, unsnappedPose, side, snappedPose))
         {
            dataMessage.getLocation().set(snappedPose.getPosition());
            dataMessage.getOrientation().set(snappedPose.getOrientation());
         }
      }
   }
}
