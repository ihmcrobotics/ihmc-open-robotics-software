package us.ihmc.valkyrie.roughTerrainWalking;

import java.util.ArrayList;

import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraph;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.SnapBasedNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.parameters.ValkyrieFootstepPlannerParameters;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

public class FootstepPlannerStepUpTest
{
   @Test
   public void testGrandparentNodeCheck()
   {
      FootstepNode node1 = new FootstepNode(1.0, 0.15, -0.349, RobotSide.LEFT);
      FootstepNode node2 = new FootstepNode(1.2, -0.55, -0.349, RobotSide.RIGHT);
      FootstepNode node3 = new FootstepNode(1.6, -0.35, -0.349, RobotSide.LEFT);

      FootstepGraph graph = new FootstepGraph();
      graph.initialize(node1);
      graph.checkAndSetEdge(node1, node2, 1.0);
      graph.checkAndSetEdge(node2, node3, 1.0);

      FootstepPlannerParametersReadOnly parameters = new ValkyrieFootstepPlannerParameters();
      SideDependentList<ConvexPolygon2D> footPolygons = createFootPolygonsFromContactPoints(
            new ValkyrieRobotModel(RobotTarget.SCS, true).getContactPointParameters());
      SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(footPolygons);

      RigidBodyTransform t1 = new RigidBodyTransform();
      RigidBodyTransform t2 = new RigidBodyTransform();
      RigidBodyTransform t3 = new RigidBodyTransform();

      t1.setTranslation(0.0, 0.0, 0.102);
      t2.setTranslation(0.0, 0.0, 0.193);
      t3.setTranslation(0.0, 0.0, 0.193);

      snapper.addSnapData(node1, new FootstepNodeSnapData(t1, footPolygons.get(RobotSide.LEFT)));
      snapper.addSnapData(node2, new FootstepNodeSnapData(t2, footPolygons.get(RobotSide.RIGHT)));
      snapper.addSnapData(node3, new FootstepNodeSnapData(t3, footPolygons.get(RobotSide.LEFT)));

      SnapBasedNodeChecker checker = new SnapBasedNodeChecker(parameters, footPolygons, snapper);
      checker.addFootstepGraph(graph);
      Assert.assertTrue(!checker.isNodeValid(node3, node2));
   }

   private static SideDependentList<ConvexPolygon2D> createFootPolygonsFromContactPoints(RobotContactPointParameters<RobotSide> contactPointParameters)
   {
      SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
      {
         ArrayList<Point2D> footPoints = contactPointParameters.getFootContactPoints().get(side);
         ConvexPolygon2D scaledFoot = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(footPoints));
         footPolygons.set(side, scaledFoot);
      }

      return footPolygons;
   }
}
