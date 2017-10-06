package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;

public class BipedalFootstepPlannerNodeUtils
{
   public static RigidBodyTransform shiftInSoleFrame(Vector2D shiftVector, RigidBodyTransform soleTransform)
   {
      RigidBodyTransform shiftTransform = new RigidBodyTransform();
      shiftTransform.setTranslation(new Vector3D(shiftVector.getX(), shiftVector.getY(), 0.0));
      soleTransform.multiply(shiftTransform);
      return soleTransform;
   }

   public static RigidBodyTransform getSnappedSoleTransform(BipedalFootstepPlannerNode node, RigidBodyTransform snapTransform)
   {
      RigidBodyTransform soleTransform = new RigidBodyTransform();
      node.getSoleTransform(soleTransform);
      soleTransform.setTranslationZ(0.0);
      soleTransform.preMultiply(snapTransform);
      return soleTransform;
   }

   public static Point3D getSolePosition(BipedalFootstepPlannerNode node)
   {
      RigidBodyTransform soleTransform = new RigidBodyTransform();
      node.getSoleTransform(soleTransform);

      Point3D currentSolePosition = new Point3D();
      soleTransform.transform(currentSolePosition);
      return currentSolePosition;
   }

   public static double getSoleYaw(BipedalFootstepPlannerNode node)
   {
      RigidBodyTransform soleTransform = new RigidBodyTransform();
      node.getSoleTransform(soleTransform);

      Vector3D eulerAngles = new Vector3D();
      soleTransform.getRotationEuler(eulerAngles);
      return eulerAngles.getZ();
   }

   public static double getCostFromStartToNode(BipedalFootstepPlannerNode node)
   {
      if (node.getParentNode() == null)
         return node.getSingleStepCost();
      return node.getSingleStepCost() + getCostFromStartToNode(node.getParentNode());
   }

   public static FootstepPlan createFootstepPlanFromEndNode(BipedalFootstepPlannerNode endNode, PlanarRegionsList planarRegionsList, BipedalFootstepPlannerParameters plannerParameters,
                                                            SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame)
   {
      FootstepPlan footstepPlan = new FootstepPlan();
      footstepPlan.clear();
      BipedalFootstepPlannerNode node = endNode;

      BipedalFootstepPlannerSnapAndWiggler snapAndWiggler = null;
      if(planarRegionsList != null)
      {
         snapAndWiggler = new BipedalFootstepPlannerSnapAndWiggler(plannerParameters);
         snapAndWiggler.setFootPolygonsInSoleFrame(footPolygonsInSoleFrame);
         snapAndWiggler.setPlanarRegionsList(planarRegionsList);
      }

      while (node != null)
      {
         RigidBodyTransform soleTransform;

         if(planarRegionsList == null)
         {
            soleTransform = new RigidBodyTransform();
            node.getSoleTransform(soleTransform);
         }
         else
         {
            RigidBodyTransform snapTransform = snapAndWiggler.getSnapAndWiggleTransform(plannerParameters.getWiggleInsideDelta(), node, new PlanarRegion());
            soleTransform = getSnappedSoleTransform(node, snapTransform);
         }

         FramePose framePose = new FramePose(ReferenceFrame.getWorldFrame(), soleTransform);
         SimpleFootstep simpleFootstep = footstepPlan.addFootstep(node.getRobotSide(), framePose);
         if (node.isPartialFoothold())
            simpleFootstep.setFoothold(node.getPartialFootholdPolygon());

         node = node.getParentNode();
      }

      footstepPlan.reverse();
      return footstepPlan;
   }
}
