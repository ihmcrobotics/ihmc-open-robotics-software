package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.listeners.BipedalFootstepPlannerListener;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static us.ihmc.robotics.Assert.*;

public class SnapAndWiggleBasedNodeCheckerTest
{
   private static final double barelyTooSteepEpsilon = 1.0e-5;

   private static final int iters = 100000;

   @Test
   public void testSnappingToInclinedPlane()
   {
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(1.0, 1.0);
      polygon.addVertex(1.0, -1.0);
      polygon.addVertex(-1.0, -1.0);
      polygon.addVertex(-1.0, 1.0);
      polygon.update();
      ArrayList<ConvexPolygon2D> polygons = new ArrayList<>();
      polygons.add(polygon);

      PlanarRegion planarRegion = new PlanarRegion();
      planarRegion.set(transformToWorld, polygons);

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(planarRegion);

      TestListener rejectionListener = new TestListener();

      double footLength = 0.2;
      double footWidth = 0.1;
      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createFootPolygons(footLength, footWidth);
      FootstepPlannerParametersReadOnly parameters = new DefaultFootstepPlannerParameters()
      {
         // don't use 45
         @Override
         public double getMinimumSurfaceInclineRadians()
         {
            return Math.toRadians(37.0);
         }
      };

      SnapAndWiggleBasedNodeChecker nodeChecker = new SnapAndWiggleBasedNodeChecker(footPolygons, parameters);
      nodeChecker.addPlannerListener(rejectionListener);

      nodeChecker.setPlanarRegions(planarRegionsList);

      FootstepNode previousNode = new FootstepNode(0.0, 0.1, 0.0, RobotSide.LEFT);
      FootstepNode node = new FootstepNode(0.0, -0.1, 0.0, RobotSide.RIGHT);

      assertTrue(nodeChecker.isNodeValidInternal(node, previousNode));
      assertEquals(null, rejectionListener.getRejectionReason());

      double rotationAngle;

      // test a bunch of independent roll/pitch valid angles
      for (rotationAngle = -parameters.getMinimumSurfaceInclineRadians() + barelyTooSteepEpsilon; rotationAngle < parameters.getMinimumSurfaceInclineRadians() - barelyTooSteepEpsilon; rotationAngle += 0.001)
      {
         rejectionListener.tickAndUpdate();
         transformToWorld.setIdentity();
         transformToWorld.appendRollRotation(rotationAngle);
         planarRegion.set(transformToWorld, polygons);
         nodeChecker.setPlanarRegions(planarRegionsList);

         assertTrue(nodeChecker.isNodeValidInternal(node, previousNode));
         assertEquals(null, rejectionListener.getRejectionReason());

         rejectionListener.tickAndUpdate();
         transformToWorld.setIdentity();
         transformToWorld.appendPitchRotation(rotationAngle);
         planarRegion.set(transformToWorld, polygons);
         nodeChecker.setPlanarRegions(planarRegionsList);

         assertTrue(nodeChecker.isNodeValidInternal(node, previousNode));
         assertEquals(null, rejectionListener.getRejectionReason());
      }

      for (rotationAngle = parameters.getMinimumSurfaceInclineRadians() + 0.001; rotationAngle < Math.toRadians(75); rotationAngle += 0.001)
      {
         rejectionListener.tickAndUpdate();
         transformToWorld.setIdentity();
         transformToWorld.appendRollRotation(rotationAngle);
         planarRegion.set(transformToWorld, polygons);
         nodeChecker.setPlanarRegions(planarRegionsList);

         assertFalse("rotation = " + rotationAngle, nodeChecker.isNodeValidInternal(node, previousNode));
         assertEquals("rotation = " + rotationAngle, BipedalFootstepPlannerNodeRejectionReason.SURFACE_NORMAL_TOO_STEEP_TO_SNAP, rejectionListener.getRejectionReason());

         rejectionListener.tickAndUpdate();
         transformToWorld.setIdentity();
         transformToWorld.appendPitchRotation(rotationAngle);
         planarRegion.set(transformToWorld, polygons);
         nodeChecker.setPlanarRegions(planarRegionsList);

         assertFalse("rotation = " + rotationAngle, nodeChecker.isNodeValidInternal(node, previousNode));
         assertEquals("rotation = " + rotationAngle, BipedalFootstepPlannerNodeRejectionReason.SURFACE_NORMAL_TOO_STEEP_TO_SNAP, rejectionListener.getRejectionReason());
      }

      for (rotationAngle = -Math.toRadians(75); rotationAngle < -parameters.getMinimumSurfaceInclineRadians(); rotationAngle += 0.001)
      {
         rejectionListener.tickAndUpdate();
         transformToWorld.setIdentity();
         transformToWorld.appendRollRotation(rotationAngle);
         planarRegion.set(transformToWorld, polygons);
         nodeChecker.setPlanarRegions(planarRegionsList);

         assertFalse("rotation = " + rotationAngle, nodeChecker.isNodeValidInternal(node, previousNode));
         assertEquals("rotation = " + rotationAngle, BipedalFootstepPlannerNodeRejectionReason.SURFACE_NORMAL_TOO_STEEP_TO_SNAP, rejectionListener.getRejectionReason());

         rejectionListener.tickAndUpdate();
         transformToWorld.setIdentity();
         transformToWorld.appendPitchRotation(rotationAngle);
         planarRegion.set(transformToWorld, polygons);
         nodeChecker.setPlanarRegions(planarRegionsList);

         assertFalse("rotation = " + rotationAngle, nodeChecker.isNodeValidInternal(node, previousNode));
         assertEquals("rotation = " + rotationAngle, BipedalFootstepPlannerNodeRejectionReason.SURFACE_NORMAL_TOO_STEEP_TO_SNAP, rejectionListener.getRejectionReason());
      }

      // test random orientations
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         QuaternionReadOnly orientation3DReadOnly = EuclidCoreRandomTools.nextQuaternion(random);

         rejectionListener.tickAndUpdate();
         transformToWorld.setIdentity();
         transformToWorld.setRotation(orientation3DReadOnly);
         planarRegion.set(transformToWorld, polygons);
         nodeChecker.setPlanarRegions(planarRegionsList);

         Vector3D vertical = new Vector3D(0.0, 0.0, 1.0);
         Vector3D normal = new Vector3D(vertical);
         orientation3DReadOnly.transform(normal);
//         QuaternionReadOnly noOrientation = new Quaternion(orientation3DReadOnly.getYaw(), 0.0, 0.0);

         double angleFromFlat = vertical.angle(normal);
         if (Math.abs(angleFromFlat) > parameters.getMinimumSurfaceInclineRadians())
         {
            String message = "actual rotation = " + angleFromFlat + ", allowed rotation = " + parameters.getMinimumSurfaceInclineRadians();
            assertFalse(message, nodeChecker.isNodeValidInternal(node, previousNode));
            boolean correctRejection = BipedalFootstepPlannerNodeRejectionReason.SURFACE_NORMAL_TOO_STEEP_TO_SNAP == rejectionListener.getRejectionReason() ||
                  BipedalFootstepPlannerNodeRejectionReason.COULD_NOT_SNAP == rejectionListener.getRejectionReason();
            assertTrue(message, correctRejection);
         }
         else
         {
            assertTrue(nodeChecker.isNodeValidInternal(node, previousNode));
            assertEquals(null, rejectionListener.getRejectionReason());
         }

      }

   }

   public class TestListener implements BipedalFootstepPlannerListener
   {
      private BipedalFootstepPlannerNodeRejectionReason rejectionReason;

      @Override
      public void addNode(FootstepNode node, FootstepNode previousNode)
      {

      }

      @Override
      public void rejectNode(FootstepNode rejectedNode, FootstepNode parentNode, BipedalFootstepPlannerNodeRejectionReason reason)
      {
         rejectionReason = reason;
      }

      @Override
      public void plannerFinished(List<FootstepNode> plan)
      {

      }

      @Override
      public void reportLowestCostNodeList(List<FootstepNode> plan)
      {

      }

      @Override
      public void tickAndUpdate()
      {
         rejectionReason = null;
      }

      public BipedalFootstepPlannerNodeRejectionReason getRejectionReason()
      {
         return rejectionReason;
      }
   }
}
