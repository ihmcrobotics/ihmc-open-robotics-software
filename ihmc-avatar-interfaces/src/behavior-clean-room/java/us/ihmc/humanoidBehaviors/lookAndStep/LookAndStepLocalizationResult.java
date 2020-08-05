package us.ihmc.humanoidBehaviors.lookAndStep;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Point3D;

import java.util.List;

public class LookAndStepLocalizationResult
{
   private final Point3D closestPointAlongPath;
   private final int closestSegmentIndex;
   private final FramePose3D subGoalPoseBetweenFeet;
   private final boolean reachedGoalZone;
   private final List<? extends Pose3DReadOnly> bodyPathPlan;

   public LookAndStepLocalizationResult(Point3D closestPointAlongPath,
                                        int closestSegmentIndex,
                                        FramePose3D subGoalPoseBetweenFeet,
                                        boolean reachedGoalZone,
                                        List<? extends Pose3DReadOnly> bodyPathPlan)
   {
      this.closestPointAlongPath = closestPointAlongPath;
      this.closestSegmentIndex = closestSegmentIndex;
      this.subGoalPoseBetweenFeet = subGoalPoseBetweenFeet;
      this.reachedGoalZone = reachedGoalZone;
      this.bodyPathPlan = bodyPathPlan;
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

   public boolean isReachedGoalZone()
   {
      return reachedGoalZone;
   }

   public List<? extends Pose3DReadOnly> getBodyPathPlan()
   {
      return bodyPathPlan;
   }
}
