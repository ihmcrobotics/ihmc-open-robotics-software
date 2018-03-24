package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import static org.junit.Assert.assertTrue;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.RecycledLinkedListBuilder.RecycledLinkedListEntry;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;

public class CentroidalMotionPlannerTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final CentroidalMotionPlannerParameters parameters = new CentroidalMotionPlannerParameters();
   static
   {
      parameters.setRobotMass(1.0);
      parameters.setDeltaTMin(0.001);
      parameters.setGravityZ(-9.81);
      parameters.setMaxForce(new Vector3D(10.0, 10.0, 10.0));
      parameters.setMinForce(new Vector3D(-10.0, -10.0, 0.0));
      parameters.setMaxForceRate(new Vector3D(10.0, 10.0, 10.0));
      parameters.setMinForceRate(new Vector3D(-10.0, -10.0, 0.0));
   }

   @Test
   public void testConstructor()
   {
      CentroidalMotionPlanner motionPlanner = new CentroidalMotionPlanner(parameters);
      assertTrue(motionPlanner != null);
   }

   @Test
   public void testNodeEntrySubmission()
   {
      CentroidalMotionNode node1 = new CentroidalMotionNode();
      node1.setTime(0.1);
      node1.setForceConstraint(new FrameVector3D(ReferenceFrame.getWorldFrame(), Double.NaN, Double.NaN, 100.0));
      CentroidalMotionNode node2 = new CentroidalMotionNode();
      node2.setTime(0.0);
      node2.setForceConstraint(new FrameVector3D(ReferenceFrame.getWorldFrame(), Double.NaN, Double.NaN, 123.0));
      CentroidalMotionNode node3 = new CentroidalMotionNode();
      node3.setTime(0.5);
      node3.setForceConstraint(new FrameVector3D(ReferenceFrame.getWorldFrame(), Double.NaN, Double.NaN, 0.0));

      CentroidalMotionPlanner motionPlanner = new CentroidalMotionPlanner(parameters);
      motionPlanner.submitNode(node1);
      motionPlanner.submitNode(node2);
      motionPlanner.submitNode(node3);

      RecycledLinkedListBuilder<CentroidalMotionNode> nodeList = motionPlanner.getNodeList();
      assertTrue(nodeList.getSize() == 3);
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> node = nodeList.getFirstEntry();
      assertTrue(node.element.getTime() == 0.0);
      node = node.getNext();
      assertTrue(node.element.getTime() == 0.1);
      node = node.getNext();
      assertTrue(node.element.getTime() == 0.5);
   }

   @Test
   public void testCentroidalTrajectoryGeneration()
   {
      CentroidalMotionNode node1 = new CentroidalMotionNode();
      CentroidalMotionNode node2 = new CentroidalMotionNode();
      CentroidalMotionNode node3 = new CentroidalMotionNode();
      CentroidalMotionNode node4 = new CentroidalMotionNode();
      CentroidalMotionNode node5 = new CentroidalMotionNode();
      double gravityZ = -9.81;
      double forceZValue = -1.0 * gravityZ;
      node1.setTime(0.0);
      node1.setForceConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, forceZValue));
      node1.setForceRateConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0));
      node1.setPositionObjective(new FramePoint3D(worldFrame, 0.0, 0.0, 0.85), new FrameVector3D(worldFrame, 10.0, 10.0, 10.0));
      node1.setLinearVelocityObjective(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0), new FrameVector3D(worldFrame, 10.0, 10.0, 10.0));
      node2.setTime(0.2);
      node2.setForceObjective(new FrameVector3D(worldFrame, 0.0, 0.0, forceZValue), new FrameVector3D(worldFrame, 0.001, 0.001, 0.001));
      node2.setForceRateObjective(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0), new FrameVector3D(worldFrame, 0.001, 0.001, 0.1));
      node3.setTime(0.4);
      node3.setForceObjective(new FrameVector3D(worldFrame, 0.0, 0.0, forceZValue), new FrameVector3D(worldFrame, 0.001, 0.001, 0.001));
      node3.setForceRateObjective(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0), new FrameVector3D(worldFrame, 0.001, 0.001, 0.1));
      node4.setTime(0.6);
      node4.setForceObjective(new FrameVector3D(worldFrame, 0.0, 0.0, forceZValue), new FrameVector3D(worldFrame, 0.01, 0.01, 0.01));
      node4.setForceRateObjective(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0), new FrameVector3D(worldFrame, 0.001, 0.001, 0.1));
      node5.setTime(0.8);
      node5.setForceConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, forceZValue));
      node5.setForceRateConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0));
      node5.setPositionObjective(new FramePoint3D(worldFrame, 0.04, 0.0, 0.85), new FrameVector3D(worldFrame, 10.0, 10.0, 10.0));
      node5.setLinearVelocityObjective(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0), new FrameVector3D(worldFrame, 10.0, 10.0, 10.0));

      CentroidalMotionPlanner motionPlanner = new CentroidalMotionPlanner(parameters);
      motionPlanner.reset();
      motionPlanner.submitNode(node1);
      motionPlanner.submitNode(node2);
      motionPlanner.submitNode(node3);
      motionPlanner.submitNode(node4);
      motionPlanner.submitNode(node5);
      CentroidalMotionSupportPolygon supportPolygon = new CentroidalMotionSupportPolygon();
      supportPolygon.setStartTime(0.0);
      supportPolygon.setEndTime(10.0);
      FrameConvexPolygon2d polygon = new FrameConvexPolygon2d();
      FramePoint2D point = new FramePoint2D();
      point.set(-0.05, -0.05);
      polygon.addVertex(point);
      point.set(-0.05, 0.05);
      polygon.addVertex(point);
      point.set(0.05, -0.05);
      polygon.addVertex(point);
      point.set(0.05, 0.05);
      polygon.addVertex(point);
      polygon.update();
      supportPolygon.setSupportPolygon(polygon);
      motionPlanner.submitSupportPolygon(supportPolygon);

      RecycledLinkedListBuilder<CentroidalMotionNode> nodeList = motionPlanner.getNodeList();
      assertTrue(nodeList.getSize() == 5);

      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> entry = nodeList.getFirstEntry();
      assertTrue("Node time: " + entry.element.getTime(), node1.getTime() == entry.element.getTime());
      entry = entry.getNext();
      assertTrue("Node time: " + entry.element.getTime(), node2.getTime() == entry.element.getTime());
      entry = entry.getNext();
      assertTrue("Node time: " + entry.element.getTime(), node3.getTime() == entry.element.getTime());

      motionPlanner.compute();
      ForceTrajectory force = motionPlanner.getForceProfile();
      assertTrue(force != null);
      assertTrue(force.getNumberOfSegments() == 4);
      DenseMatrix64F[] positionValues = motionPlanner.getOptimizedPositionValues();
      PrintTools.debug(positionValues[0].toString());
      PrintTools.debug(positionValues[1].toString());
   }

   @Test
   public void testTorqueTrajectoryOptimization()
   {
      CentroidalMotionNode node1 = new CentroidalMotionNode();
      CentroidalMotionNode node2 = new CentroidalMotionNode();
      CentroidalMotionNode node3 = new CentroidalMotionNode();
      CentroidalMotionNode node4 = new CentroidalMotionNode();
      CentroidalMotionNode node5 = new CentroidalMotionNode();
      CentroidalMotionNode node6 = new CentroidalMotionNode();
      CentroidalMotionNode node7 = new CentroidalMotionNode();
      CentroidalMotionNode node8 = new CentroidalMotionNode();
      CentroidalMotionNode node9 = new CentroidalMotionNode();
      double gravityZ = -9.81;
      double forceZValue = -1.0 * gravityZ;
      node1.setTime(0.0);
      node1.setForceConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, forceZValue));
      node1.setForceRateConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0));
      node1.setPositionObjective(new FramePoint3D(worldFrame, 0.0, 0.0, 0.85), new FrameVector3D(worldFrame, 10.0, 10.0, 10.0));
      node1.setLinearVelocityObjective(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0), new FrameVector3D(worldFrame, 10.0, 10.0, 10.0));
      node1.setCoPConstraint(new FramePoint2D(worldFrame, 0.0, 0.0));

      node2.setTime(0.1);
      node2.setForceObjective(new FrameVector3D(worldFrame, 0.0, 0.0, forceZValue), new FrameVector3D(worldFrame, 0.0001, 0.0001, 0.0001));
      node2.setForceRateObjective(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0), new FrameVector3D(worldFrame, 0.0001, 0.0001, 0.0001));

      node3.setTime(0.2);
      node3.setForceObjective(new FrameVector3D(worldFrame, 0.0, 0.0, forceZValue), new FrameVector3D(worldFrame, 0.0001, 0.0001, 0.0001));
      node3.setForceRateObjective(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0), new FrameVector3D(worldFrame, 0.0001, 0.0001, 0.0001));

      node4.setTime(0.3);
      node4.setForceObjective(new FrameVector3D(worldFrame, 0.0, 0.0, forceZValue), new FrameVector3D(worldFrame, 0.0001, 0.0001, 0.0001));
      node4.setForceRateObjective(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0), new FrameVector3D(worldFrame, 0.0001, 0.0001, 0.0001));

      node5.setTime(0.4);
      node5.setForceObjective(new FrameVector3D(worldFrame, 0.0, 0.0, forceZValue), new FrameVector3D(worldFrame, 0.0001, 0.0001, 0.0001));
      node5.setForceRateObjective(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0), new FrameVector3D(worldFrame, 0.0001, 0.0001, 0.0001));

      node6.setTime(0.1);
      node6.setForceObjective(new FrameVector3D(worldFrame, 0.0, 0.0, forceZValue), new FrameVector3D(worldFrame, 0.0001, 0.0001, 0.0001));
      node6.setForceRateObjective(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0), new FrameVector3D(worldFrame, 0.0001, 0.0001, 0.0001));

      node7.setTime(0.2);
      node7.setForceObjective(new FrameVector3D(worldFrame, 0.0, 0.0, forceZValue), new FrameVector3D(worldFrame, 0.0001, 0.0001, 0.0001));
      node7.setForceRateObjective(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0), new FrameVector3D(worldFrame, 0.0001, 0.0001, 0.0001));

      node8.setTime(0.3);
      node8.setForceObjective(new FrameVector3D(worldFrame, 0.0, 0.0, forceZValue), new FrameVector3D(worldFrame, 0.0001, 0.0001, 0.0001));
      node8.setForceRateObjective(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0), new FrameVector3D(worldFrame, 0.0001, 0.0001, 0.0001));

      node9.setTime(0.8);
      node9.setForceConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, forceZValue));
      node9.setForceRateConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0));
      node9.setPositionConstraint(new FramePoint3D(worldFrame, 0.04, 0.0, 0.85)); //, new FrameVector3D(worldFrame, 10.0, 10.0, 10.0));
      node9.setLinearVelocityConstraint(new FrameVector3D(worldFrame, 0.0, 0.0, 0.0)); //, new FrameVector3D(worldFrame, 10.0, 10.0, 10.0));

      CentroidalMotionPlanner motionPlanner = new CentroidalMotionPlanner(parameters);
      motionPlanner.reset();
      motionPlanner.submitNode(node1);
      motionPlanner.submitNode(node2);
      motionPlanner.submitNode(node3);
      motionPlanner.submitNode(node4);
      motionPlanner.submitNode(node5);
      motionPlanner.submitNode(node6);
      motionPlanner.submitNode(node7);
      motionPlanner.submitNode(node8);
      motionPlanner.submitNode(node9);

      CentroidalMotionSupportPolygon supportPolygon = new CentroidalMotionSupportPolygon();
      supportPolygon.setStartTime(0.0);
      supportPolygon.setEndTime(1.0);
      FrameConvexPolygon2d polygon = new FrameConvexPolygon2d();
      FramePoint2D point = new FramePoint2D();
      point.set(-0.05, -0.05);
      polygon.addVertex(point);
      point.set(-0.05, 0.05);
      polygon.addVertex(point);
      point.set(0.05, -0.05);
      polygon.addVertex(point);
      point.set(0.05, 0.05);
      polygon.addVertex(point);
      polygon.update();
      supportPolygon.setSupportPolygon(polygon);
      motionPlanner.submitSupportPolygon(supportPolygon);

      RecycledLinkedListBuilder<CentroidalMotionNode> nodeList = motionPlanner.getNodeList();
      assertTrue(nodeList.getSize() == 9);

      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> entry = nodeList.getFirstEntry();
      assertTrue("Node time: " + entry.element.getTime(), node1.getTime() == entry.element.getTime());
      entry = entry.getNext();
      assertTrue("Node time: " + entry.element.getTime(), node2.getTime() == entry.element.getTime());

      motionPlanner.compute();
      ForceTrajectory force = motionPlanner.getForceProfile();
      assertTrue(force != null);
      assertTrue(force.getNumberOfSegments() == 8);
      DenseMatrix64F[] positionValues = motionPlanner.getOptimizedPositionValues();
      DenseMatrix64F[] velocityValues = motionPlanner.getOptimizedVelocityValues();
      DenseMatrix64F[] forceValues = motionPlanner.getOptimizedForceValues();
      PrintTools.debug(positionValues[0].toString());
      PrintTools.debug(positionValues[1].toString());
      PrintTools.debug(positionValues[2].toString());
      PrintTools.debug(velocityValues[0].toString());
      PrintTools.debug(velocityValues[1].toString());
      PrintTools.debug(velocityValues[2].toString());
      PrintTools.debug(forceValues[0].toString());
      PrintTools.debug(forceValues[1].toString());
      PrintTools.debug(forceValues[2].toString());
   }
}