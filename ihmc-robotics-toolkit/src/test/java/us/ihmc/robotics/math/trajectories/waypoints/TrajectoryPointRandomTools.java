package us.ihmc.robotics.math.trajectories.waypoints;

import java.util.Random;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.robotics.math.trajectories.trajectorypoints.EuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSO3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.SE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.SO3TrajectoryPoint;

public class TrajectoryPointRandomTools
{
   public static EuclideanWaypoint nextEuclideanWaypoint(Random random)
   {
      return new EuclideanWaypoint(EuclidCoreRandomTools.nextPoint3D(random), EuclidCoreRandomTools.nextVector3D(random));
   }

   public static SO3Waypoint nextSO3Waypoint(Random random)
   {
      return new SO3Waypoint(EuclidCoreRandomTools.nextOrientation3D(random), EuclidCoreRandomTools.nextVector3D(random));
   }

   public static SE3Waypoint nextSE3Waypoint(Random random)
   {
      return new SE3Waypoint(EuclidCoreRandomTools.nextPoint3D(random),
                             EuclidCoreRandomTools.nextOrientation3D(random),
                             EuclidCoreRandomTools.nextVector3D(random),
                             EuclidCoreRandomTools.nextVector3D(random));
   }

   public static EuclideanTrajectoryPoint nextEuclideanTrajectoryPoint(Random random)
   {
      return new EuclideanTrajectoryPoint(random.nextDouble(), EuclidCoreRandomTools.nextPoint3D(random), EuclidCoreRandomTools.nextVector3D(random));
   }

   public static SO3TrajectoryPoint nextSO3TrajectoryPoint(Random random)
   {
      return new SO3TrajectoryPoint(random.nextDouble(), EuclidCoreRandomTools.nextOrientation3D(random), EuclidCoreRandomTools.nextVector3D(random));
   }

   public static SE3TrajectoryPoint nextSE3TrajectoryPoint(Random random)
   {
      return new SE3TrajectoryPoint(random.nextDouble(),
                                    EuclidCoreRandomTools.nextPoint3D(random),
                                    EuclidCoreRandomTools.nextOrientation3D(random),
                                    EuclidCoreRandomTools.nextVector3D(random),
                                    EuclidCoreRandomTools.nextVector3D(random));
   }

   public static FrameEuclideanWaypoint nextFrameEuclideanWaypoint(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameEuclideanWaypoint(EuclidFrameRandomTools.nextFramePoint3D(random, referenceFrame),
                                        EuclidFrameRandomTools.nextFrameVector3D(random, referenceFrame));
   }

   public static FrameSO3Waypoint nextFrameSO3Waypoint(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameSO3Waypoint(EuclidFrameRandomTools.nextFrameOrientation3D(random, referenceFrame),
                                  EuclidFrameRandomTools.nextFrameVector3D(random, referenceFrame));
   }

   public static FrameSE3Waypoint nextFrameSE3Waypoint(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameSE3Waypoint(EuclidFrameRandomTools.nextFramePoint3D(random, referenceFrame),
                                  EuclidFrameRandomTools.nextFrameOrientation3D(random, referenceFrame),
                                  EuclidFrameRandomTools.nextFrameVector3D(random, referenceFrame),
                                  EuclidFrameRandomTools.nextFrameVector3D(random, referenceFrame));
   }

   public static FrameEuclideanTrajectoryPoint nextFrameEuclideanTrajectoryPoint(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameEuclideanTrajectoryPoint(random.nextDouble(),
                                               EuclidFrameRandomTools.nextFramePoint3D(random, referenceFrame),
                                               EuclidFrameRandomTools.nextFrameVector3D(random, referenceFrame));
   }

   public static FrameSO3TrajectoryPoint nextFrameSO3TrajectoryPoint(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameSO3TrajectoryPoint(random.nextDouble(),
                                         EuclidFrameRandomTools.nextFrameOrientation3D(random, referenceFrame),
                                         EuclidFrameRandomTools.nextFrameVector3D(random, referenceFrame));
   }

   public static FrameSE3TrajectoryPoint nextFrameSE3TrajectoryPoint(Random random, ReferenceFrame referenceFrame)
   {
      return new FrameSE3TrajectoryPoint(random.nextDouble(),
                                         EuclidFrameRandomTools.nextFramePoint3D(random, referenceFrame),
                                         EuclidFrameRandomTools.nextFrameOrientation3D(random, referenceFrame),
                                         EuclidFrameRandomTools.nextFrameVector3D(random, referenceFrame),
                                         EuclidFrameRandomTools.nextFrameVector3D(random, referenceFrame));
   }

}
