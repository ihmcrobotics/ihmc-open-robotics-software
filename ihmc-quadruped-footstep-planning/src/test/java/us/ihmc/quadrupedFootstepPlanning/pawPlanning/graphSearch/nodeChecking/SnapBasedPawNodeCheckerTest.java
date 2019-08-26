package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.nodeChecking;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.PawStepPlannerNodeRejectionReason;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping.SimplePlanarRegionPawNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.listeners.PawStepPlannerListener;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.DefaultPawStepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class SnapBasedPawNodeCheckerTest
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
      SimplePlanarRegionPawNodeSnapper snapper = new SimplePlanarRegionPawNodeSnapper(parameters, parameters::getProjectInsideDistance,
                                                                                      parameters::getProjectInsideUsingConvexHull, true);
      PawNodeChecker nodeChecker = new SnapBasedPawNodeChecker(parameters, snapper);

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


      double yaw = PawNode.computeNominalYaw(frontLeft.getX(), frontLeft.getY(), frontRight.getX(), frontRight.getY(), hindLeft.getX(), hindLeft.getY(),
                                             hindRight.getX(), hindRight.getY());

      PawNode node = new PawNode(robotQuadrant, frontLeft, frontRight, hindLeft, hindRight, yaw, stanceLength, stanceWidth);

      boolean isValid = nodeChecker.isNodeValid(node);

      // should be valid
      testListener.assertCorrectRejection("", null, null);
      assertTrue(isValid);

      // run another test where it's just slightly too far inside
      parameters.setDistanceInside(0.5 * cinderWidth + 0.01);

      // reset snap data
      snapper.setPlanarRegions(planarRegionList);

      isValid = nodeChecker.isNodeValid(node);

      // should be valid, but at center
      testListener.assertCorrectRejection("", null, null);
      assertTrue(isValid);


      // reset snap data
      snapper.setPlanarRegions(planarRegionList);
      parameters.setDistanceInside(0.08);
      frontLeft.setX(0.9 * 0.5 * cinderWidth);

      node = new PawNode(robotQuadrant, frontLeft, frontRight, hindLeft, hindRight, yaw, stanceLength, stanceWidth);

      isValid = nodeChecker.isNodeValid(node);
      // should be invalid, as it requires shifting way too far
      testListener.assertCorrectRejection("", node, PawStepPlannerNodeRejectionReason.COULD_NOT_SNAP);
      assertFalse(isValid);
   }


   private class TestListener implements PawStepPlannerListener
   {
      private final AtomicReference<PawStepPlannerNodeRejectionReason> reason = new AtomicReference<>();
      private final AtomicReference<PawNode> rejectedNode = new AtomicReference<>();

      @Override
      public void addNode(PawNode node, PawNode previousNode)
      {

      }

      public void assertCorrectRejection(String message, PawNode node, PawStepPlannerNodeRejectionReason rejectionReason)
      {
         assertEquals(rejectionReason, reason.getAndSet(null), message);
         assertEquals(node, rejectedNode.getAndSet(null), message);
      }

      @Override
      public void rejectNode(PawNode node, PawNode parentNode, PawStepPlannerNodeRejectionReason rejectionReason)
      {
         rejectedNode.set(node);
         reason.set(rejectionReason);
      }

      @Override
      public void rejectNode(PawNode node, PawStepPlannerNodeRejectionReason rejectionReason)
      {
         rejectedNode.set(node);
         reason.set(rejectionReason);
      }

      @Override
      public void plannerFinished(List<PawNode> plan)
      {

      }

      @Override
      public void reportLowestCostNodeList(List<PawNode> plan)
      {

      }

      @Override
      public void tickAndUpdate()
      {

      }
   }

   private class TestParameters extends DefaultPawStepPlannerParameters
   {
      double distance;
      @Override
      public double getProjectInsideDistance()
      {
         return distance;
      }

      public void setDistanceInside(double distanceInside)
      {
         this.distance = distanceInside;
      }
   }
}
