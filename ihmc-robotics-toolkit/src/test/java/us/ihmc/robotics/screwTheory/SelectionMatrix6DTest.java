package us.ihmc.robotics.screwTheory;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertNull;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.ejml.dense.row.RandomMatrices_DDRM;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.matrixlib.MatrixTools;

public class SelectionMatrix6DTest
{
   private static final int ITERATIONS = 500;

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testGettersSetters()
   {
      Random random = new Random(123423L);

      SelectionMatrix6D selectionMatrix6D = new SelectionMatrix6D();
      assertNull(selectionMatrix6D.getAngularSelectionFrame());
      assertNull(selectionMatrix6D.getLinearSelectionFrame());
      assertTrue(selectionMatrix6D.isAngularXSelected());
      assertTrue(selectionMatrix6D.isAngularYSelected());
      assertTrue(selectionMatrix6D.isAngularZSelected());
      assertTrue(selectionMatrix6D.isAngularPartActive());
      assertTrue(selectionMatrix6D.isLinearXSelected());
      assertTrue(selectionMatrix6D.isLinearYSelected());
      assertTrue(selectionMatrix6D.isLinearZSelected());
      assertTrue(selectionMatrix6D.isLinearPartActive());

      for (int i = 0; i < ITERATIONS; i++)
      {
         boolean xAngularSelected = random.nextBoolean();
         boolean yAngularSelected = random.nextBoolean();
         boolean zAngularSelected = random.nextBoolean();
         boolean xLinearSelected = random.nextBoolean();
         boolean yLinearSelected = random.nextBoolean();
         boolean zLinearSelected = random.nextBoolean();
         selectionMatrix6D.setAngularAxisSelection(xAngularSelected, yAngularSelected, zAngularSelected);
         selectionMatrix6D.setLinearAxisSelection(xLinearSelected, yLinearSelected, zLinearSelected);
         assertEquals(xAngularSelected, selectionMatrix6D.isAngularXSelected());
         assertEquals(yAngularSelected, selectionMatrix6D.isAngularYSelected());
         assertEquals(zAngularSelected, selectionMatrix6D.isAngularZSelected());
         assertEquals(xAngularSelected || yAngularSelected || zAngularSelected, selectionMatrix6D.isAngularPartActive());
         assertEquals(xLinearSelected, selectionMatrix6D.isLinearXSelected());
         assertEquals(yLinearSelected, selectionMatrix6D.isLinearYSelected());
         assertEquals(zLinearSelected, selectionMatrix6D.isLinearZSelected());
         assertEquals(xLinearSelected || yLinearSelected || zLinearSelected, selectionMatrix6D.isLinearPartActive());
         

         xAngularSelected = random.nextBoolean();
         yAngularSelected = random.nextBoolean();
         zAngularSelected = random.nextBoolean();
         xLinearSelected = random.nextBoolean();
         yLinearSelected = random.nextBoolean();
         zLinearSelected = random.nextBoolean();

         selectionMatrix6D.selectAngularX(xAngularSelected);
         selectionMatrix6D.selectAngularY(yAngularSelected);
         selectionMatrix6D.selectAngularZ(zAngularSelected);
         selectionMatrix6D.selectLinearX(xLinearSelected);
         selectionMatrix6D.selectLinearY(yLinearSelected);
         selectionMatrix6D.selectLinearZ(zLinearSelected);
         assertEquals(xAngularSelected, selectionMatrix6D.isAngularXSelected());
         assertEquals(yAngularSelected, selectionMatrix6D.isAngularYSelected());
         assertEquals(zAngularSelected, selectionMatrix6D.isAngularZSelected());
         assertEquals(xLinearSelected, selectionMatrix6D.isLinearXSelected());
         assertEquals(yLinearSelected, selectionMatrix6D.isLinearYSelected());
         assertEquals(zLinearSelected, selectionMatrix6D.isLinearZSelected());

         selectionMatrix6D.resetSelection();
         assertTrue(selectionMatrix6D.isAngularXSelected());
         assertTrue(selectionMatrix6D.isAngularYSelected());
         assertTrue(selectionMatrix6D.isAngularZSelected());
         assertTrue(selectionMatrix6D.isAngularPartActive());
         assertTrue(selectionMatrix6D.isLinearXSelected());
         assertTrue(selectionMatrix6D.isLinearYSelected());
         assertTrue(selectionMatrix6D.isLinearZSelected());
         assertTrue(selectionMatrix6D.isLinearPartActive());

         selectionMatrix6D.setToLinearSelectionOnly();
         assertFalse(selectionMatrix6D.isAngularXSelected());
         assertFalse(selectionMatrix6D.isAngularYSelected());
         assertFalse(selectionMatrix6D.isAngularZSelected());
         assertFalse(selectionMatrix6D.isAngularPartActive());
         assertTrue(selectionMatrix6D.isLinearXSelected());
         assertTrue(selectionMatrix6D.isLinearYSelected());
         assertTrue(selectionMatrix6D.isLinearZSelected());
         assertTrue(selectionMatrix6D.isLinearPartActive());

         selectionMatrix6D.setToAngularSelectionOnly();
         assertTrue(selectionMatrix6D.isAngularXSelected());
         assertTrue(selectionMatrix6D.isAngularYSelected());
         assertTrue(selectionMatrix6D.isAngularZSelected());
         assertTrue(selectionMatrix6D.isAngularPartActive());
         assertFalse(selectionMatrix6D.isLinearXSelected());
         assertFalse(selectionMatrix6D.isLinearYSelected());
         assertFalse(selectionMatrix6D.isLinearZSelected());
         assertFalse(selectionMatrix6D.isLinearPartActive());

         assertNull(selectionMatrix6D.getAngularSelectionFrame());
         assertNull(selectionMatrix6D.getLinearSelectionFrame());
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame("blop" + i, random, ReferenceFrame.getWorldFrame());
         selectionMatrix6D.setSelectionFrame(randomFrame);
         assertTrue(randomFrame == selectionMatrix6D.getAngularSelectionFrame());
         assertTrue(randomFrame == selectionMatrix6D.getLinearSelectionFrame());

         selectionMatrix6D.clearSelectionFrame();
         assertNull(selectionMatrix6D.getAngularSelectionFrame());
         assertNull(selectionMatrix6D.getLinearSelectionFrame());

         selectionMatrix6D.setSelectionFrame(randomFrame);
         selectionMatrix6D.setToLinearSelectionOnly();
         assertNull(selectionMatrix6D.getAngularSelectionFrame());
         assertNull(selectionMatrix6D.getLinearSelectionFrame());

         selectionMatrix6D.setSelectionFrame(randomFrame);
         selectionMatrix6D.setToAngularSelectionOnly();
         assertNull(selectionMatrix6D.getAngularSelectionFrame());
         assertNull(selectionMatrix6D.getLinearSelectionFrame());

         selectionMatrix6D.setSelectionFrame(randomFrame);
         selectionMatrix6D.resetSelection();
         assertNull(selectionMatrix6D.getAngularSelectionFrame());
         assertNull(selectionMatrix6D.getLinearSelectionFrame());

         selectionMatrix6D.setSelectionFrame(randomFrame);
         selectionMatrix6D.clearAngularSelectionFrame();
         assertNull(selectionMatrix6D.getAngularSelectionFrame());
         assertTrue(randomFrame == selectionMatrix6D.getLinearSelectionFrame());

         selectionMatrix6D.setSelectionFrame(randomFrame);
         selectionMatrix6D.clearLinearSelectionFrame();
         assertTrue(randomFrame == selectionMatrix6D.getAngularSelectionFrame());
         assertNull(selectionMatrix6D.getLinearSelectionFrame());
      }
   }

   @Test
   public void testGetFullSelectionMatrixInFrame() throws Exception
   {
      Random random = new Random(456465L);
      SelectionMatrix6D selectionMatrix6D = new SelectionMatrix6D();
      SelectionMatrix3D angularPart3D = new SelectionMatrix3D();
      SelectionMatrix3D linearPart3D = new SelectionMatrix3D();

      RigidBodyTransform randomTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

      List<ReferenceFrame> referenceFrames = new ArrayList<>();
      referenceFrames.add(null);
      referenceFrames.add(ReferenceFrame.getWorldFrame());
      referenceFrames.add(ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("blop1", ReferenceFrame.getWorldFrame(), randomTransform));
      referenceFrames.add(EuclidFrameRandomTools.nextReferenceFrame("blop2", random, ReferenceFrame.getWorldFrame()));
      referenceFrames.add(ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("blop1Bis", ReferenceFrame.getWorldFrame(), randomTransform));

      for (int i = 0; i < ITERATIONS; i++)
      {
         for (ReferenceFrame selectionFrame : referenceFrames)
         {
            for (ReferenceFrame destinationFrame : referenceFrames.subList(1, referenceFrames.size()))
            {
               DMatrixRMaj actualSelectionMatrix = RandomMatrices_DDRM.rectangle(6, 6, -1.0, 1.0, random);
               DMatrixRMaj expectedSelectionMatrix = RandomMatrices_DDRM.rectangle(6, 6, -1.0, 1.0, random);

               boolean xAngularSelected = random.nextBoolean();
               boolean yAngularSelected = random.nextBoolean();
               boolean zAngularSelected = random.nextBoolean();
               boolean xLinearSelected = random.nextBoolean();
               boolean yLinearSelected = random.nextBoolean();
               boolean zLinearSelected = random.nextBoolean();

               selectionMatrix6D.setAngularAxisSelection(xAngularSelected, yAngularSelected, zAngularSelected);
               selectionMatrix6D.setLinearAxisSelection(xLinearSelected, yLinearSelected, zLinearSelected);
               selectionMatrix6D.setSelectionFrame(selectionFrame);

               angularPart3D.setAxisSelection(xAngularSelected, yAngularSelected, zAngularSelected);
               angularPart3D.setSelectionFrame(selectionFrame);
               linearPart3D.setAxisSelection(xLinearSelected, yLinearSelected, zLinearSelected);
               linearPart3D.setSelectionFrame(selectionFrame);

               selectionMatrix6D.getFullSelectionMatrixInFrame(destinationFrame, actualSelectionMatrix);

               expectedSelectionMatrix.zero();
               angularPart3D.getFullSelectionMatrixInFrame(destinationFrame, 0, 0, expectedSelectionMatrix);
               linearPart3D.getFullSelectionMatrixInFrame(destinationFrame, 3, 3, expectedSelectionMatrix);

               assertMatrixEquals(expectedSelectionMatrix, actualSelectionMatrix, 1.0e-12);

               // Verifies that it has the intended application: Being able to apply the selection to any frame
               DMatrixRMaj expectedSubspaceVector = new DMatrixRMaj(6, 1);
               DMatrixRMaj actualSubspaceVector = new DMatrixRMaj(6, 1);

               FrameVector3D randomAngularVector = EuclidFrameRandomTools.nextFrameVector3D(random, destinationFrame);
               FrameVector3D randomLinearVector = EuclidFrameRandomTools.nextFrameVector3D(random, destinationFrame);
               DMatrixRMaj originalVector = new DMatrixRMaj(6, 1);
               randomAngularVector.get(0, originalVector);
               randomLinearVector.get(3, originalVector);
               selectionMatrix6D.getFullSelectionMatrixInFrame(destinationFrame, expectedSelectionMatrix);
               CommonOps_DDRM.mult(expectedSelectionMatrix, originalVector, actualSubspaceVector);

               if (selectionFrame != null)
               {
                  randomAngularVector.changeFrame(selectionFrame);
                  randomLinearVector.changeFrame(selectionFrame);
               }

               if (!xAngularSelected)
                  randomAngularVector.setX(0.0);
               if (!yAngularSelected)
                  randomAngularVector.setY(0.0);
               if (!zAngularSelected)
                  randomAngularVector.setZ(0.0);

               if (!xLinearSelected)
                  randomLinearVector.setX(0.0);
               if (!yLinearSelected)
                  randomLinearVector.setY(0.0);
               if (!zLinearSelected)
                  randomLinearVector.setZ(0.0);

               if (selectionFrame != null)
               {
                  randomAngularVector.changeFrame(destinationFrame);
                  randomLinearVector.changeFrame(destinationFrame);
               }
               randomAngularVector.get(0, expectedSubspaceVector);
               randomLinearVector.get(3, expectedSubspaceVector);
               assertMatrixEquals(expectedSubspaceVector, actualSubspaceVector, 1.0e-12);
            }
         }
      }
   }

   @Test
   public void testGetEfficientSelectionMatrixInFrame() throws Exception
   {
      Random random = new Random(456465L);
      SelectionMatrix6D selectionMatrix6D = new SelectionMatrix6D();

      RigidBodyTransform randomTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

      List<ReferenceFrame> referenceFrames = new ArrayList<>();
      referenceFrames.add(null);
      referenceFrames.add(ReferenceFrame.getWorldFrame());
      referenceFrames.add(ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("blop1", ReferenceFrame.getWorldFrame(), randomTransform));
      referenceFrames.add(EuclidFrameRandomTools.nextReferenceFrame("blop2", random, ReferenceFrame.getWorldFrame()));
      referenceFrames.add(ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("blop1Bis", ReferenceFrame.getWorldFrame(), randomTransform));

      for (int i = 0; i < ITERATIONS; i++)
      {
         for (ReferenceFrame selectionFrame : referenceFrames)
         {
            for (ReferenceFrame destinationFrame : referenceFrames.subList(1, referenceFrames.size()))
            {
               DMatrixRMaj actualSelectionMatrix = RandomMatrices_DDRM.rectangle(6, 6, -1.0, 1.0, random);
               DMatrixRMaj expectedSelectionMatrix = RandomMatrices_DDRM.rectangle(6, 6, -1.0, 1.0, random);

               boolean xAngularSelected = random.nextBoolean();
               boolean yAngularSelected = random.nextBoolean();
               boolean zAngularSelected = random.nextBoolean();
               boolean xLinearSelected = random.nextBoolean();
               boolean yLinearSelected = random.nextBoolean();
               boolean zLinearSelected = random.nextBoolean();

               selectionMatrix6D.setAngularAxisSelection(xAngularSelected, yAngularSelected, zAngularSelected);
               selectionMatrix6D.setLinearAxisSelection(xLinearSelected, yLinearSelected, zLinearSelected);
               selectionMatrix6D.setSelectionFrame(selectionFrame);

               selectionMatrix6D.getFullSelectionMatrixInFrame(destinationFrame, expectedSelectionMatrix);
               MatrixTools.removeZeroRows(expectedSelectionMatrix, 1.0e-7);

               selectionMatrix6D.getCompactSelectionMatrixInFrame(destinationFrame, actualSelectionMatrix);

               assertMatrixEquals(expectedSelectionMatrix, actualSelectionMatrix, 1.0e-12);
            }
         }
      }
   }

   private static void assertMatrixEquals(DMatrixRMaj expected, DMatrixRMaj actual, double epsilon)
   {
      assertTrue(assertErrorMessage(expected, actual), MatrixFeatures_DDRM.isEquals(expected, actual, epsilon));
   }

   private static String assertErrorMessage(DMatrixRMaj expected, DMatrixRMaj actual)
   {
      return "Expected:\n" + expected + "\nActual:\n" + actual;
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator mutationTestFacilitator = new MutationTestFacilitator();
      mutationTestFacilitator.addClassesToMutate(SelectionMatrix3D.class, SelectionMatrix6D.class);
      mutationTestFacilitator.addTestClassesToRun(SelectionMatrix3DTest.class, SelectionMatrix6DTest.class);
      mutationTestFacilitator.doMutationTest();
      mutationTestFacilitator.openResultInBrowser();
   }
}
