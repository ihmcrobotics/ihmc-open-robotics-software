package us.ihmc.robotics.screwTheory;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertNull;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixFeatures;
import org.ejml.ops.RandomMatrices;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FrameMatrix3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.linearAlgebra.MatrixTools;

public class SelectionMatrix3DTest
{
   private static final int ITERATIONS = 1000;

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testSettersGetters() throws Exception
   {
      Random random = new Random(123423L);

      SelectionMatrix3D selectionMatrix3D = new SelectionMatrix3D();
      assertNull(selectionMatrix3D.getSelectionFrame());
      assertTrue(selectionMatrix3D.isXSelected());
      assertTrue(selectionMatrix3D.isYSelected());
      assertTrue(selectionMatrix3D.isZSelected());

      for (int i = 0; i < ITERATIONS; i++)
      {
         boolean xSelected = random.nextBoolean();
         boolean ySelected = random.nextBoolean();
         boolean zSelected = random.nextBoolean();
         selectionMatrix3D.setAxisSelection(xSelected, ySelected, zSelected);
         assertEquals(xSelected, selectionMatrix3D.isXSelected());
         assertEquals(ySelected, selectionMatrix3D.isYSelected());
         assertEquals(zSelected, selectionMatrix3D.isZSelected());

         xSelected = random.nextBoolean();
         ySelected = random.nextBoolean();
         zSelected = random.nextBoolean();

         selectionMatrix3D.selectXAxis(xSelected);
         selectionMatrix3D.selectYAxis(ySelected);
         selectionMatrix3D.selectZAxis(zSelected);
         assertEquals(xSelected, selectionMatrix3D.isXSelected());
         assertEquals(ySelected, selectionMatrix3D.isYSelected());
         assertEquals(zSelected, selectionMatrix3D.isZSelected());

         selectionMatrix3D.resetSelection();
         assertTrue(selectionMatrix3D.isXSelected());
         assertTrue(selectionMatrix3D.isYSelected());
         assertTrue(selectionMatrix3D.isZSelected());

         selectionMatrix3D.clearSelection();
         assertFalse(selectionMatrix3D.isXSelected());
         assertFalse(selectionMatrix3D.isYSelected());
         assertFalse(selectionMatrix3D.isZSelected());

         assertNull(selectionMatrix3D.getSelectionFrame());
         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame("blop" + i, random, ReferenceFrame.getWorldFrame());
         selectionMatrix3D.setSelectionFrame(randomFrame);
         assertTrue(randomFrame == selectionMatrix3D.getSelectionFrame());

         selectionMatrix3D.clearSelectionFrame();
         assertNull(selectionMatrix3D.getSelectionFrame());

         selectionMatrix3D.setSelectionFrame(randomFrame);
         selectionMatrix3D.clearSelection();
         assertNull(selectionMatrix3D.getSelectionFrame());

         selectionMatrix3D.setSelectionFrame(randomFrame);
         selectionMatrix3D.resetSelection();
         assertNull(selectionMatrix3D.getSelectionFrame());
      }
   }

   @Test
   public void testApplySelection()
   {
      Random random = new Random(2342L);

      SelectionMatrix3D selectionMatrix3D = new SelectionMatrix3D();
      FrameMatrix3D frameMatrix3D = new FrameMatrix3D();

      RigidBodyTransform randomTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

      List<ReferenceFrame> referenceFrames = new ArrayList<>();
      referenceFrames.add(null);
      referenceFrames.add(ReferenceFrame.getWorldFrame());
      referenceFrames.add(ReferenceFrame.constructFrameWithUnchangingTransformToParent("blop1", ReferenceFrame.getWorldFrame(), randomTransform));
      referenceFrames.add(EuclidFrameRandomTools.nextReferenceFrame("blop2", random, ReferenceFrame.getWorldFrame()));
      referenceFrames.add(ReferenceFrame.constructFrameWithUnchangingTransformToParent("blop1Bis", ReferenceFrame.getWorldFrame(), randomTransform));

      for (int i = 0; i < ITERATIONS; i++)
      {
         for (ReferenceFrame selectionFrame : referenceFrames)
         {
            for (ReferenceFrame vectorFrame : referenceFrames.subList(1, referenceFrames.size()))
            {
               FrameVector3D originalVector = EuclidFrameRandomTools.nextFrameVector3D(random, vectorFrame);
               FrameVector3D expectedVector = new FrameVector3D(originalVector);
               FrameVector3D actualVector = new FrameVector3D(originalVector);

               boolean xSelected = random.nextBoolean();
               boolean ySelected = random.nextBoolean();
               boolean zSelected = random.nextBoolean();

               selectionMatrix3D.setAxisSelection(xSelected, ySelected, zSelected);
               selectionMatrix3D.setSelectionFrame(selectionFrame);

               if (selectionFrame == null)
                  frameMatrix3D.setToZero(vectorFrame);
               else
                  frameMatrix3D.setToZero(selectionFrame);
               frameMatrix3D.setM00(xSelected ? 1.0 : 0.0);
               frameMatrix3D.setM11(ySelected ? 1.0 : 0.0);
               frameMatrix3D.setM22(zSelected ? 1.0 : 0.0);

               frameMatrix3D.changeFrame(vectorFrame);
               frameMatrix3D.transform(expectedVector);

               selectionMatrix3D.applySelection(actualVector);

               assertEquals(expectedVector.getReferenceFrame(), actualVector.getReferenceFrame());
               EuclidCoreTestTools.assertTuple3DEquals(expectedVector, actualVector, 1.0e-12);
            }
         }
      }
   }

   @Test
   public void testGetFullSelectionMatrixInFrame() throws Exception
   {
      Random random = new Random(456465L);
      SelectionMatrix3D selectionMatrix3D = new SelectionMatrix3D();
      FrameMatrix3D frameMatrix3D = new FrameMatrix3D();

      RigidBodyTransform randomTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

      List<ReferenceFrame> referenceFrames = new ArrayList<>();
      referenceFrames.add(null);
      referenceFrames.add(ReferenceFrame.getWorldFrame());
      referenceFrames.add(ReferenceFrame.constructFrameWithUnchangingTransformToParent("blop1", ReferenceFrame.getWorldFrame(), randomTransform));
      referenceFrames.add(EuclidFrameRandomTools.nextReferenceFrame("blop2", random, ReferenceFrame.getWorldFrame()));
      referenceFrames.add(ReferenceFrame.constructFrameWithUnchangingTransformToParent("blop1Bis", ReferenceFrame.getWorldFrame(), randomTransform));

      for (int i = 0; i < ITERATIONS; i++)
      {
         for (ReferenceFrame selectionFrame : referenceFrames)
         {
            for (ReferenceFrame destinationFrame : referenceFrames.subList(1, referenceFrames.size()))
            {
               DenseMatrix64F actualSelectionMatrix = RandomMatrices.createRandom(3, 3, -1.0, 1.0, random);
               DenseMatrix64F expectedSelectionMatrix = RandomMatrices.createRandom(3, 3, -1.0, 1.0, random);

               boolean xSelected = random.nextBoolean();
               boolean ySelected = random.nextBoolean();
               boolean zSelected = random.nextBoolean();

               selectionMatrix3D.setAxisSelection(xSelected, ySelected, zSelected);
               selectionMatrix3D.setSelectionFrame(selectionFrame);

               frameMatrix3D.setToZero(selectionFrame);
               frameMatrix3D.setM00(xSelected ? 1.0 : 0.0);
               frameMatrix3D.setM11(ySelected ? 1.0 : 0.0);
               frameMatrix3D.setM22(zSelected ? 1.0 : 0.0);

               selectionMatrix3D.getFullSelectionMatrixInFrame(destinationFrame, actualSelectionMatrix);
               if (selectionFrame != null)
                  frameMatrix3D.changeFrame(destinationFrame);
               frameMatrix3D.get(expectedSelectionMatrix);
               assertMatrixEquals(expectedSelectionMatrix, actualSelectionMatrix, 1.0e-12);

               // Verifies that it has the intended application: Being able to apply the selection to any frame
               DenseMatrix64F expectedSubspaceVector = new DenseMatrix64F(3, 1);
               DenseMatrix64F actualSubspaceVector = new DenseMatrix64F(3, 1);

               FrameVector3D randomVector = EuclidFrameRandomTools.nextFrameVector3D(random, destinationFrame);
               DenseMatrix64F originalVector = new DenseMatrix64F(3, 1);
               randomVector.get(originalVector);
               selectionMatrix3D.getFullSelectionMatrixInFrame(destinationFrame, expectedSelectionMatrix);
               CommonOps.mult(expectedSelectionMatrix, originalVector, actualSubspaceVector);

               if (selectionFrame != null)
                  randomVector.changeFrame(selectionFrame);

               if (!xSelected)
                  randomVector.setX(0.0);
               if (!ySelected)
                  randomVector.setY(0.0);
               if (!zSelected)
                  randomVector.setZ(0.0);

               if (selectionFrame != null)
                  randomVector.changeFrame(destinationFrame);
               randomVector.get(expectedSubspaceVector);

               assertMatrixEquals(expectedSubspaceVector, actualSubspaceVector, 1.0e-12);
            }
         }
      }
   }

   @Test
   public void testGetEfficientSelectionMatrixInFrame() throws Exception
   {
      Random random = new Random(456465L);
      SelectionMatrix3D selectionMatrix3D = new SelectionMatrix3D();

      RigidBodyTransform randomTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

      List<ReferenceFrame> referenceFrames = new ArrayList<>();
      referenceFrames.add(null);
      referenceFrames.add(ReferenceFrame.getWorldFrame());
      referenceFrames.add(ReferenceFrame.constructFrameWithUnchangingTransformToParent("blop1", ReferenceFrame.getWorldFrame(), randomTransform));
      referenceFrames.add(EuclidFrameRandomTools.nextReferenceFrame("blop2", random, ReferenceFrame.getWorldFrame()));
      referenceFrames.add(ReferenceFrame.constructFrameWithUnchangingTransformToParent("blop1Bis", ReferenceFrame.getWorldFrame(), randomTransform));

      for (int i = 0; i < ITERATIONS; i++)
      {
         for (ReferenceFrame selectionFrame : referenceFrames)
         {
            for (ReferenceFrame destinationFrame : referenceFrames.subList(1, referenceFrames.size()))
            {
               DenseMatrix64F actualSelectionMatrix = RandomMatrices.createRandom(3, 3, -1.0, 1.0, random);
               DenseMatrix64F expectedSelectionMatrix = RandomMatrices.createRandom(3, 3, -1.0, 1.0, random);

               boolean xSelected = random.nextBoolean();
               boolean ySelected = random.nextBoolean();
               boolean zSelected = random.nextBoolean();

               selectionMatrix3D.setAxisSelection(xSelected, ySelected, zSelected);
               selectionMatrix3D.setSelectionFrame(selectionFrame);

               selectionMatrix3D.getFullSelectionMatrixInFrame(destinationFrame, expectedSelectionMatrix);
               MatrixTools.removeZeroRows(expectedSelectionMatrix, 1.0e-7);

               selectionMatrix3D.getEfficientSelectionMatrixInFrame(destinationFrame, actualSelectionMatrix);

               assertMatrixEquals(expectedSelectionMatrix, actualSelectionMatrix, 1.0e-12);
            }
         }
      }
   }

   private static void assertMatrixEquals(DenseMatrix64F expected, DenseMatrix64F actual, double epsilon)
   {
      assertTrue(assertErrorMessage(expected, actual), MatrixFeatures.isEquals(expected, actual, epsilon));
   }

   private static String assertErrorMessage(DenseMatrix64F expected, DenseMatrix64F actual)
   {
      return "Expected:\n" + expected + "\nActual:\n" + actual;
   }
}
