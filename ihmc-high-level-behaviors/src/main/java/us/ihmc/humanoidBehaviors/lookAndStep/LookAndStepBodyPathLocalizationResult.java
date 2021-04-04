package us.ihmc.humanoidBehaviors.lookAndStep;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.List;

public class LookAndStepBodyPathLocalizationResult
{
   private final Point3D closestPointAlongPath;
   private final int closestSegmentIndex;
   private final Pose3DReadOnly midFeetAlongPath;
   private final List<? extends Pose3DReadOnly> bodyPathPlan;
   private final SideDependentList<MinimalFootstep> stanceForPlanning;

   public LookAndStepBodyPathLocalizationResult(Point3D closestPointAlongPath,
                                                int closestSegmentIndex,
                                                Pose3DReadOnly midFeetAlongPath,
                                                List<? extends Pose3DReadOnly> bodyPathPlan,
                                                SideDependentList<MinimalFootstep> stanceForPlanning)
   {
      this.closestPointAlongPath = closestPointAlongPath;
      this.closestSegmentIndex = closestSegmentIndex;
      this.midFeetAlongPath = midFeetAlongPath;
      this.bodyPathPlan = bodyPathPlan;
      this.stanceForPlanning = stanceForPlanning;
   }

   public Point3D getClosestPointAlongPath()
   {
      return closestPointAlongPath;
   }

   public int getClosestSegmentIndex()
   {
      return closestSegmentIndex;
   }

   public Pose3DReadOnly getMidFeetAlongPath()
   {
      return midFeetAlongPath;
   }

   public List<? extends Pose3DReadOnly> getBodyPathPlan()
   {
      return bodyPathPlan;
   }

   public SideDependentList<MinimalFootstep> getStanceForPlanning()
   {
      return stanceForPlanning;
   }
}
