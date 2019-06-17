package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeChecker;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.QuadrupedFootstepPlannerNodeRejectionReason;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.listeners.QuadrupedFootstepPlannerListener;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeChecking.FootstepNodeChecker;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeChecking.FootstepNodeTransitionChecker;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeChecking.SnapBasedNodeChecker;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeChecking.SnapBasedNodeTransitionChecker;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class SnapBasedNodeCheckerTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Test
   public void testSnappingToBlock()
   {
      double stanceLength = 1.0;
      double stanceWidth = 0.5;

      double cinderWidth = 7.5 * 2.54 / 100;
      double cinderLength = 15.5 * 2.54 / 100;
      double cinderHeight = 7.5 * 2.54 / 100;

      TestParameters parameters = new TestParameters();
      parameters.setDistanceInside(0.5 * cinderWidth);
      SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(parameters, parameters::getProjectInsideDistanceForExpansion,
                                                                                                parameters::getProjectInsideUsingConvexHullDuringExpansion, true);
      FootstepNodeChecker nodeChecker = new SnapBasedNodeChecker(parameters, snapper);

      TestListener testListener = new TestListener();
      nodeChecker.addPlannerListener(testListener);

      // set up kind of a random ground plane
      ConvexPolygon2D groundPlanePolygon = new ConvexPolygon2D();
      groundPlanePolygon.addVertex(10.0, 10.0);
      groundPlanePolygon.addVertex(-10.0, 10.0);
      groundPlanePolygon.addVertex(10.0, -10.0);
      groundPlanePolygon.addVertex(-10.0, -10.0);
      groundPlanePolygon.update();

      ConvexPolygon2D cinderPolygon = new ConvexPolygon2D();
      cinderPolygon.addVertex(0.5 * cinderWidth, 0.5 * cinderLength);
      cinderPolygon.addVertex(0.5 * cinderWidth, -0.5 * cinderLength);
      cinderPolygon.addVertex(-0.5 * cinderWidth, -0.5 * cinderLength);
      cinderPolygon.addVertex(-0.5 * cinderWidth, 0.5 * cinderLength);
      cinderPolygon.update();

      RigidBodyTransform cinderTransform = new RigidBodyTransform(new Quaternion(), new Vector3D(0.0, 0.0, cinderHeight));

      PlanarRegionsList planarRegionList = new PlanarRegionsList();
      planarRegionList.addPlanarRegion(new PlanarRegion(new RigidBodyTransform(), groundPlanePolygon));
      planarRegionList.addPlanarRegion(new PlanarRegion(cinderTransform, cinderPolygon));

      nodeChecker.setPlanarRegions(planarRegionList);

      // have to set this really small, so that we don't get a ton of rounding issues
      RobotQuadrant robotQuadrant = RobotQuadrant.FRONT_LEFT;

      FramePoint2D frontLeft = new FramePoint2D(worldFrame, 0.0, 0.0);
      FramePoint2D frontRight = new FramePoint2D(worldFrame, 0.0, -stanceWidth);
      FramePoint2D hindLeft = new FramePoint2D(worldFrame, -stanceLength, 0.0);
      FramePoint2D hindRight = new FramePoint2D(worldFrame, -stanceLength, -stanceWidth); // way further out


      double yaw = FootstepNode.computeNominalYaw(frontLeft.getX(), frontLeft.getY(), frontRight.getX(), frontRight.getY(), hindLeft.getX(), hindLeft.getY(),
                                                  hindRight.getX(), hindRight.getY());

      FootstepNode node = new FootstepNode(robotQuadrant, frontLeft, frontRight, hindLeft, hindRight, yaw, stanceLength, stanceWidth);

      boolean isValid = nodeChecker.isNodeValid(node);

      // should be valid
      testListener.assertCorrectRejection("", null, null);
      assertTrue(isValid);

      // run another test where it's just slightly too far inside
      parameters.setDistanceInside(0.5 * cinderWidth + 0.01);

      // reset snap data
      snapper.setPlanarRegions(planarRegionList);

      isValid = nodeChecker.isNodeValid(node);

      // should be invalid
      testListener.assertCorrectRejection("", node, QuadrupedFootstepPlannerNodeRejectionReason.COULD_NOT_SNAP);
      assertFalse(isValid);


      snapper.setPlanarRegions(planarRegionList);
      parameters.setDistanceInside(0.08);
      frontLeft.setX(0.9 * 0.5 * cinderWidth);

      node = new FootstepNode(robotQuadrant, frontLeft, frontRight, hindLeft, hindRight, yaw, stanceLength, stanceWidth);

      isValid = nodeChecker.isNodeValid(node);
      testListener.assertCorrectRejection("", node, QuadrupedFootstepPlannerNodeRejectionReason.COULD_NOT_SNAP);
      assertFalse(isValid);
   }


   private class TestListener implements QuadrupedFootstepPlannerListener
   {
      private final AtomicReference<QuadrupedFootstepPlannerNodeRejectionReason> reason = new AtomicReference<>();
      private final AtomicReference<FootstepNode> rejectedNode = new AtomicReference<>();

      @Override
      public void addNode(FootstepNode node, FootstepNode previousNode)
      {

      }

      public void assertCorrectRejection(String message, FootstepNode node, QuadrupedFootstepPlannerNodeRejectionReason rejectionReason)
      {
         assertEquals(rejectionReason, reason.getAndSet(null), message);
         assertEquals(node, rejectedNode.getAndSet(null), message);
      }

      @Override
      public void rejectNode(FootstepNode node, FootstepNode parentNode, QuadrupedFootstepPlannerNodeRejectionReason rejectionReason)
      {
         rejectedNode.set(node);
         reason.set(rejectionReason);
      }

      @Override
      public void rejectNode(FootstepNode node, QuadrupedFootstepPlannerNodeRejectionReason rejectionReason)
      {
         rejectedNode.set(node);
         reason.set(rejectionReason);
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

      }
   }

   private class TestParameters extends DefaultFootstepPlannerParameters
   {
      double distance;
      @Override
      public double getProjectInsideDistanceForExpansion()
      {
         return distance;
      }

      public void setDistanceInside(double distanceInside)
      {
         this.distance = distanceInside;
      }
   }
}
