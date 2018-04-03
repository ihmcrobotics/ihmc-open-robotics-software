package us.ihmc.robotics.screwTheory;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixFeatures;
import org.ejml.ops.RandomMatrices;
import org.junit.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.linearAlgebra.MatrixTools;

public class SelectionMatrix6DTest
{

   private static final int ITERATIONS = 500;

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
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

   @ContinuousIntegrationTest(estimatedDuration = 0.9)
   @Test(timeout = 30000)
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
      referenceFrames.add(ReferenceFrame.constructFrameWithUnchangingTransformToParent("blop1", ReferenceFrame.getWorldFrame(), randomTransform));
      referenceFrames.add(EuclidFrameRandomTools.nextReferenceFrame("blop2", random, ReferenceFrame.getWorldFrame()));
      referenceFrames.add(ReferenceFrame.constructFrameWithUnchangingTransformToParent("blop1Bis", ReferenceFrame.getWorldFrame(), randomTransform));

      for (int i = 0; i < ITERATIONS; i++)
      {
         for (ReferenceFrame selectionFrame : referenceFrames)
         {
            for (ReferenceFrame destinationFrame : referenceFrames.subList(1, referenceFrames.size()))
            {
               DenseMatrix64F actualSelectionMatrix = RandomMatrices.createRandom(6, 6, -1.0, 1.0, random);
               DenseMatrix64F expectedSelectionMatrix = RandomMatrices.createRandom(6, 6, -1.0, 1.0, random);

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
               DenseMatrix64F expectedSubspaceVector = new DenseMatrix64F(6, 1);
               DenseMatrix64F actualSubspaceVector = new DenseMatrix64F(6, 1);

               FrameVector3D randomAngularVector = EuclidFrameRandomTools.nextFrameVector3D(random, destinationFrame);
               FrameVector3D randomLinearVector = EuclidFrameRandomTools.nextFrameVector3D(random, destinationFrame);
               DenseMatrix64F originalVector = new DenseMatrix64F(6, 1);
               randomAngularVector.get(0, originalVector);
               randomLinearVector.get(3, originalVector);
               selectionMatrix6D.getFullSelectionMatrixInFrame(destinationFrame, expectedSelectionMatrix);
               CommonOps.mult(expectedSelectionMatrix, originalVector, actualSubspaceVector);

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

   @ContinuousIntegrationTest(estimatedDuration = 0.8)
   @Test(timeout = 30000)
   public void testGetEfficientSelectionMatrixInFrame() throws Exception
   {
      Random random = new Random(456465L);
      SelectionMatrix6D selectionMatrix6D = new SelectionMatrix6D();

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
               DenseMatrix64F actualSelectionMatrix = RandomMatrices.createRandom(6, 6, -1.0, 1.0, random);
               DenseMatrix64F expectedSelectionMatrix = RandomMatrices.createRandom(6, 6, -1.0, 1.0, random);

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

   private static void assertMatrixEquals(DenseMatrix64F expected, DenseMatrix64F actual, double epsilon)
   {
      assertTrue(assertErrorMessage(expected, actual), MatrixFeatures.isEquals(expected, actual, epsilon));
   }

   private static String assertErrorMessage(DenseMatrix64F expected, DenseMatrix64F actual)
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
