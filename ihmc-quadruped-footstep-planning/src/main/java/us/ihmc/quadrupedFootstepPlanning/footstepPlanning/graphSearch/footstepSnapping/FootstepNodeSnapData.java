package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class FootstepNodeSnapData
{
   private final QuadrantDependentList<RigidBodyTransform> snapTransforms;

   public FootstepNodeSnapData(QuadrantDependentList<RigidBodyTransform> snapTransforms)
   {
      this.snapTransforms = snapTransforms;
   }

   public RigidBodyTransform getSnapTransform(RobotQuadrant robotQuadrant)
   {
      return snapTransforms.get(robotQuadrant);
   }

   private static final FootstepNodeSnapData EMPTY_SNAP_DATA;
   private static final FootstepNodeSnapData IDENTITY_SNAP_DATA;

   static
   {
      IDENTITY_SNAP_DATA = new FootstepNodeSnapData(new QuadrantDependentList<>(new RigidBodyTransform(), new RigidBodyTransform(), new RigidBodyTransform(),
                                                                                new RigidBodyTransform()));

      QuadrantDependentList<RigidBodyTransform> emptySnapTransforms = new QuadrantDependentList<>();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         RigidBodyTransform snapTransform = new RigidBodyTransform();
         snapTransform.setToNaN();
         emptySnapTransforms.put(robotQuadrant, snapTransform);
      }

      EMPTY_SNAP_DATA = new FootstepNodeSnapData(emptySnapTransforms);
   }

   public static FootstepNodeSnapData emptyData()
   {
      return EMPTY_SNAP_DATA;
   }

   public static FootstepNodeSnapData identityData()
   {
      return IDENTITY_SNAP_DATA;
   }
}
