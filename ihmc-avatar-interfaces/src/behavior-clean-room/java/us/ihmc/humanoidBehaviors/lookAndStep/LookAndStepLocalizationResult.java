package us.ihmc.humanoidBehaviors.lookAndStep;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.List;

public class LookAndStepLocalizationResult
{
   private final Point3D closestPointAlongPath;
   private final int closestSegmentIndex;
   private final FramePose3D subGoalPoseBetweenFeet;
   private final List<? extends Pose3DReadOnly> bodyPathPlan;
   private final SideDependentList<MinimalFootstep> stanceForPlanning;

   public LookAndStepLocalizationResult(Point3D closestPointAlongPath,
                                        int closestSegmentIndex,
                                        FramePose3D subGoalPoseBetweenFeet,
                                        List<? extends Pose3DReadOnly> bodyPathPlan,
                                        SideDependentList<MinimalFootstep> stanceForPlanning)
   {
      this.closestPointAlongPath = closestPointAlongPath;
      this.closestSegmentIndex = closestSegmentIndex;
      this.subGoalPoseBetweenFeet = subGoalPoseBetweenFeet;
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

   public FramePose3D getSubGoalPoseBetweenFeet()
   {
      return subGoalPoseBetweenFeet;
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
