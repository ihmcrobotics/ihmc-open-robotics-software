package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import us.ihmc.commons.Epsilons;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;

public class AngularControlModuleHelperTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final CentroidalMotionPlannerParameters parameters = new CentroidalMotionPlannerParameters();
   static
   {
      parameters.setRobotMass(1.0);
      parameters.setDeltaTMin(0.001);
      parameters.setGravityZ(-9.81);
   }

   @Test
   public void testConstructor()
   {
      AngularControlModuleHelper angularHelper = new AngularControlModuleHelper(parameters);
      assertTrue(angularHelper != null);
   }

   @Test
   public void testCoPPolygonConstraintGeneration()
   {
      AngularControlModuleHelper angularHelper = new AngularControlModuleHelper(parameters);
      RecycledLinkedListBuilder<CentroidalMotionNode> nodeList = new RecycledLinkedListBuilder<>(CentroidalMotionNode.class);
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> nodeEntry1 = nodeList.getOrCreateFirstEntry();
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> nodeEntry2 = nodeList.insertAfter(nodeEntry1);
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> nodeEntry3 = nodeList.insertAfter(nodeEntry2);

      RecycledLinkedListBuilder<CentroidalMotionSupportPolygon> supportPolygonList = new RecycledLinkedListBuilder<>(CentroidalMotionSupportPolygon.class);
      RecycledLinkedListBuilder<CentroidalMotionSupportPolygon>.RecycledLinkedListEntry<CentroidalMotionSupportPolygon> supportPolygonEntry1 = supportPolygonList.getOrCreateFirstEntry();
      RecycledLinkedListBuilder<CentroidalMotionSupportPolygon>.RecycledLinkedListEntry<CentroidalMotionSupportPolygon> supportPolygonEntry2 = supportPolygonList.insertAfter(supportPolygonEntry1);
      RecycledLinkedListBuilder<CentroidalMotionSupportPolygon>.RecycledLinkedListEntry<CentroidalMotionSupportPolygon> supportPolygonEntry3 = supportPolygonList.insertAfter(supportPolygonEntry2);

      FrameConvexPolygon2d polygon1 = new FrameConvexPolygon2d(worldFrame);
      FramePoint2D point = new FramePoint2D(worldFrame);
      double l = 0.05;
      for (int i = 0; i < 4; i++)
      {
         point.set(Math.pow(-1, i) * l, Math.pow(-1, i / 2) * l);
         polygon1.addVertex(point);
      }
      polygon1.update();
      FrameConvexPolygon2d polygon2 = new FrameConvexPolygon2d(worldFrame);
      for (int i = 0; i < 4; i++)
      {
         point.set(Math.pow(-1, i) * l + 0.1, Math.pow(-1, i / 2) * l);
         polygon2.addVertex(point);
      }
      polygon2.update();

      FrameConvexPolygon2d polygon3 = new FrameConvexPolygon2d(worldFrame);
      for (int i = 0; i < 4; i++)
      {
         point.set(Math.pow(-1, i) * l + 0.2, Math.pow(-1, i / 2) * l);
         polygon3.addVertex(point);
      }
      polygon3.update();

      CentroidalMotionSupportPolygon supportPolygon1 = supportPolygonEntry1.element;
      supportPolygon1.setStartTime(0.0);
      supportPolygon1.setEndTime(0.3);
      supportPolygon1.setSupportPolygon(polygon1);
      CentroidalMotionSupportPolygon supportPolygon2 = supportPolygonEntry2.element;
      supportPolygon2.setStartTime(0.7);
      supportPolygon2.setEndTime(1.0);
      supportPolygon2.setSupportPolygon(polygon2);
      CentroidalMotionSupportPolygon supportPolygon3 = supportPolygonEntry3.element;
      supportPolygon3.setStartTime(1.0);
      supportPolygon3.setEndTime(1.1);
      supportPolygon3.setSupportPolygon(polygon3);

      CentroidalMotionNode node1 = nodeEntry1.element;
      node1.setTime(0.0);
      CentroidalMotionNode node2 = nodeEntry2.element;
      node2.setTime(0.5);
      CentroidalMotionNode node3 = nodeEntry3.element;
      node3.setTime(1.0);

      angularHelper.computeCoPPointConstraints(nodeList, supportPolygonList);
      DenseMatrix64F Ain = angularHelper.getCoPSupportPolygonConstraintAinMatrix();
      DenseMatrix64F bin = angularHelper.getCoPSupportPolygonConstraintbinMatrix();
      assertTrue(Ain.getNumRows() == 12);
      assertTrue(bin.getNumRows() == 12);
      assertTrue(Ain.getNumCols() == 6);
      assertTrue(bin.getNumCols() == 1);

      double epsilon = Epsilons.ONE_BILLIONTH;
      assertEquals(0.005, bin.get(0, 0), epsilon);
      assertEquals(0.005, bin.get(1, 0), epsilon);
      assertEquals(0.005, bin.get(2, 0), epsilon);
      assertEquals(0.005, bin.get(3, 0), epsilon);
      assertEquals(0.005, bin.get(4, 0), epsilon);
      assertEquals(0.015, bin.get(5, 0), epsilon);
      assertEquals(0.005, bin.get(6, 0), epsilon);
      assertEquals(-0.005, bin.get(7, 0), epsilon);
      assertEquals(0.005, bin.get(8, 0), epsilon);
      assertEquals(0.025, bin.get(9, 0), epsilon);
      assertEquals(0.005, bin.get(10, 0), epsilon);
      assertEquals(-0.015, bin.get(11, 0), epsilon);
   }

}
