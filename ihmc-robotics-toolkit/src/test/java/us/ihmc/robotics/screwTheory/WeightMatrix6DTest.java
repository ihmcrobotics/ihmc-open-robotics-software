package us.ihmc.robotics.screwTheory;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertNull;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.ejml.dense.row.RandomMatrices_DDRM;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;

public class WeightMatrix6DTest
{
   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   private static final int ITERATIONS = 500;

   @Test
   public void testGettersSetters()
   {
      Random random = new Random(123423L);

      WeightMatrix6D weightMatrix6D = new WeightMatrix6D();
      assertNull(weightMatrix6D.getAngularWeightFrame());
      assertNull(weightMatrix6D.getLinearWeightFrame());
      WeightMatrix3D linearPart = weightMatrix6D.getLinearPart();
      assertTrue(Double.isNaN(linearPart.getXAxisWeight()));
      assertTrue(Double.isNaN(linearPart.getYAxisWeight()));
      assertTrue(Double.isNaN(linearPart.getZAxisWeight()));
      WeightMatrix3D angularPart = weightMatrix6D.getAngularPart();
      assertTrue(Double.isNaN(angularPart.getXAxisWeight()));
      assertTrue(Double.isNaN(angularPart.getYAxisWeight()));
      assertTrue(Double.isNaN(angularPart.getZAxisWeight()));

      for (int i = 0; i < ITERATIONS; i++)
      {
         double angularXWeight = random.nextDouble();
         double angularYWeight = random.nextDouble();
         double angularZWeight = random.nextDouble();
         double linearXWeight = random.nextDouble();
         double linearYWeight = random.nextDouble();
         double linearZWeight = random.nextDouble();
         
         weightMatrix6D.setAngularWeights(angularXWeight, angularYWeight, angularZWeight);
         weightMatrix6D.setLinearWeights(linearXWeight, linearYWeight, linearZWeight);
         assertEquals(angularXWeight, weightMatrix6D.getAngularPart().getXAxisWeight(), 1e-8);
         assertEquals(angularYWeight, weightMatrix6D.getAngularPart().getYAxisWeight(), 1e-8);
         assertEquals(angularZWeight, weightMatrix6D.getAngularPart().getZAxisWeight(), 1e-8);
         assertEquals(linearXWeight, weightMatrix6D.getLinearPart().getXAxisWeight(), 1e-8);
         assertEquals(linearYWeight, weightMatrix6D.getLinearPart().getYAxisWeight(), 1e-8);
         assertEquals(linearZWeight, weightMatrix6D.getLinearPart().getZAxisWeight(), 1e-8);

         angularXWeight = random.nextDouble();
         angularYWeight = random.nextDouble();
         angularZWeight = random.nextDouble();
         linearXWeight = random.nextDouble();
         linearYWeight = random.nextDouble();
         linearZWeight = random.nextDouble();
         Vector3D angularWeights = new Vector3D(angularXWeight, angularYWeight, angularZWeight);
         Vector3D linearWeights = new Vector3D(linearXWeight, linearYWeight, linearZWeight);
         
         weightMatrix6D.setAngularWeights(angularWeights);
         weightMatrix6D.setLinearWeights(linearWeights);

         assertEquals(angularXWeight, weightMatrix6D.getAngularPart().getXAxisWeight(), 1e-8);
         assertEquals(angularYWeight, weightMatrix6D.getAngularPart().getYAxisWeight(), 1e-8);
         assertEquals(angularZWeight, weightMatrix6D.getAngularPart().getZAxisWeight(), 1e-8);
         assertEquals(linearXWeight, weightMatrix6D.getLinearPart().getXAxisWeight(), 1e-8);
         assertEquals(linearYWeight, weightMatrix6D.getLinearPart().getYAxisWeight(), 1e-8);
         assertEquals(linearZWeight, weightMatrix6D.getLinearPart().getZAxisWeight(), 1e-8);
         
         weightMatrix6D.clearAngularWeights();
         angularPart = weightMatrix6D.getAngularPart();
         assertTrue(Double.isNaN(angularPart.getXAxisWeight()));
         assertTrue(Double.isNaN(angularPart.getYAxisWeight()));
         assertTrue(Double.isNaN(angularPart.getZAxisWeight()));
         assertEquals(linearXWeight, weightMatrix6D.getLinearPart().getXAxisWeight(), 1e-8);
         assertEquals(linearYWeight, weightMatrix6D.getLinearPart().getYAxisWeight(), 1e-8);
         assertEquals(linearZWeight, weightMatrix6D.getLinearPart().getZAxisWeight(), 1e-8);
         
         weightMatrix6D.setAngularWeights(angularWeights);
         weightMatrix6D.clearLinearWeights();
         
         assertEquals(angularXWeight, weightMatrix6D.getAngularPart().getXAxisWeight(), 1e-8);
         assertEquals(angularYWeight, weightMatrix6D.getAngularPart().getYAxisWeight(), 1e-8);
         assertEquals(angularZWeight, weightMatrix6D.getAngularPart().getZAxisWeight(), 1e-8);
         linearPart = weightMatrix6D.getLinearPart();
         assertTrue(Double.isNaN(linearPart.getXAxisWeight()));
         assertTrue(Double.isNaN(linearPart.getYAxisWeight()));
         assertTrue(Double.isNaN(linearPart.getZAxisWeight()));
         
         weightMatrix6D.clear();
         assertNull(weightMatrix6D.getLinearWeightFrame());
         assertNull(weightMatrix6D.getAngularWeightFrame());
         
         linearPart = weightMatrix6D.getLinearPart();
         assertTrue(Double.isNaN(linearPart.getXAxisWeight()));
         assertTrue(Double.isNaN(linearPart.getYAxisWeight()));
         assertTrue(Double.isNaN(linearPart.getZAxisWeight()));
         angularPart = weightMatrix6D.getAngularPart();
         assertTrue(Double.isNaN(angularPart.getXAxisWeight()));
         assertTrue(Double.isNaN(angularPart.getYAxisWeight()));
         assertTrue(Double.isNaN(angularPart.getZAxisWeight()));

         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame("blop" + i, random, ReferenceFrame.getWorldFrame());
         weightMatrix6D.setWeightFrame(randomFrame);
         assertTrue(randomFrame == weightMatrix6D.getLinearWeightFrame());
         assertTrue(randomFrame == weightMatrix6D.getAngularWeightFrame());

         weightMatrix6D.clearWeightFrame();
         assertNull(weightMatrix6D.getLinearWeightFrame());
         assertNull(weightMatrix6D.getAngularWeightFrame());

         weightMatrix6D.setWeightFrame(randomFrame);
         weightMatrix6D.clear();
         assertNull(weightMatrix6D.getLinearWeightFrame());
         assertNull(weightMatrix6D.getAngularWeightFrame());

         weightMatrix6D.setWeightFrame(randomFrame);
         weightMatrix6D.clear();
         assertNull(weightMatrix6D.getLinearWeightFrame());
         assertNull(weightMatrix6D.getAngularWeightFrame());

         weightMatrix6D.setWeightFrames(randomFrame, ReferenceFrame.getWorldFrame());
         assertTrue(randomFrame == weightMatrix6D.getAngularWeightFrame());
         assertTrue(ReferenceFrame.getWorldFrame() == weightMatrix6D.getLinearWeightFrame());
         weightMatrix6D.clear();
         assertNull(weightMatrix6D.getLinearWeightFrame());
         assertNull(weightMatrix6D.getAngularWeightFrame());
         
         weightMatrix6D.setWeightFrames(ReferenceFrame.getWorldFrame(), randomFrame);
         assertTrue(randomFrame == weightMatrix6D.getLinearWeightFrame());
         assertTrue(ReferenceFrame.getWorldFrame() == weightMatrix6D.getAngularWeightFrame());
         weightMatrix6D.clear();
         assertNull(weightMatrix6D.getLinearWeightFrame());
         assertNull(weightMatrix6D.getAngularWeightFrame());
         
         weightMatrix6D.setWeightFrame(randomFrame);
         weightMatrix6D.clearAngularWeightFrame();
         assertTrue(randomFrame == weightMatrix6D.getLinearWeightFrame());
         assertNull(weightMatrix6D.getAngularWeightFrame());
         
         weightMatrix6D.setWeightFrame(randomFrame);
         weightMatrix6D.clearLinearWeightFrame();
         assertTrue(randomFrame == weightMatrix6D.getAngularWeightFrame());
         assertNull(weightMatrix6D.getLinearWeightFrame());
         
      }
   }

   @Test
   public void testGetFullSelectionMatrixInFrame() throws Exception
   {
      Random random = new Random(456465L);
      WeightMatrix6D weightMatrix6D = new WeightMatrix6D();
      WeightMatrix3D angularPart3D = new WeightMatrix3D();
      WeightMatrix3D linearPart3D = new WeightMatrix3D();

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

               double angularXWeight = random.nextDouble();
               double angularYWeight = random.nextDouble();
               double angularZWeight = random.nextDouble();
               double linearXWeight = random.nextDouble();
               double linearYWeight = random.nextDouble();
               double linearZWeight = random.nextDouble();

               weightMatrix6D.setAngularWeights(angularXWeight, angularYWeight, angularZWeight);
               weightMatrix6D.setLinearWeights(linearXWeight, linearYWeight, linearZWeight);
               weightMatrix6D.setWeightFrame(selectionFrame);

               angularPart3D.setWeights(angularXWeight, angularYWeight, angularZWeight);
               angularPart3D.setWeightFrame(selectionFrame);
               linearPart3D.setWeights(linearXWeight, linearYWeight, linearZWeight);
               linearPart3D.setWeightFrame(selectionFrame);

               weightMatrix6D.getFullWeightMatrixInFrame(destinationFrame, actualSelectionMatrix);

               expectedSelectionMatrix.zero();
               angularPart3D.getFullWeightMatrixInFrame(destinationFrame, 0, 0, expectedSelectionMatrix);
               linearPart3D.getFullWeightMatrixInFrame(destinationFrame, 3, 3, expectedSelectionMatrix);

               assertMatrixEquals(expectedSelectionMatrix, actualSelectionMatrix, 1.0e-12);

               // Verifies that it has the intended application: Being able to apply the selection to any frame
//               DMatrixRMaj expectedSubspaceVector = new DMatrixRMaj(6, 1);
//               DMatrixRMaj actualSubspaceVector = new DMatrixRMaj(6, 1);
//
//               FrameVector randomAngularVector = FrameVector.generateRandomFrameVector(random, destinationFrame);
//               FrameVector randomLinearVector = FrameVector.generateRandomFrameVector(random, destinationFrame);
//               DMatrixRMaj originalVector = new DMatrixRMaj(6, 1);
//               randomAngularVector.get(0, originalVector);
//               randomLinearVector.get(3, originalVector);
//               weightMatrix6D.getFullWeightMatrixInFrame(destinationFrame, expectedSelectionMatrix);
//               CommonOps_DDRM.mult(expectedSelectionMatrix, originalVector, actualSubspaceVector);
//
//               if (selectionFrame != null)
//               {
//                  randomAngularVector.changeFrame(selectionFrame);
//                  randomLinearVector.changeFrame(selectionFrame);
//               }
//
//               if (!angularXWeight)
//                  randomAngularVector.setX(0.0);
//               if (!angularYWeight)
//                  randomAngularVector.setY(0.0);
//               if (!angularZWeight)
//                  randomAngularVector.setZ(0.0);
//
//               if (!linearXWeight)
//                  randomLinearVector.setX(0.0);
//               if (!linearYWeight)
//                  randomLinearVector.setY(0.0);
//               if (!linearZWeight)
//                  randomLinearVector.setZ(0.0);
//
//               if (selectionFrame != null)
//               {
//                  randomAngularVector.changeFrame(destinationFrame);
//                  randomLinearVector.changeFrame(destinationFrame);
//               }
//               randomAngularVector.get(0, expectedSubspaceVector);
//               randomLinearVector.get(3, expectedSubspaceVector);
//               assertMatrixEquals(expectedSubspaceVector, actualSubspaceVector, 1.0e-12);
            }
         }
      }
   }

   @Test
   public void testGetEfficientSelectionMatrixInFrame() throws Exception
   {
      Random random = new Random(456465L);
      WeightMatrix6D weightMatrix6D = new WeightMatrix6D();

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

               double angularXWeight = random.nextDouble();
               double angularYWeight = random.nextDouble();
               double angularZWeight = random.nextDouble();
               double linearXWeight = random.nextDouble();
               double linearYWeight = random.nextDouble();
               double linearZWeight = random.nextDouble();

               weightMatrix6D.setAngularWeights(angularXWeight, angularYWeight, angularZWeight);
               weightMatrix6D.setLinearWeights(linearXWeight, linearYWeight, linearZWeight);
               weightMatrix6D.setWeightFrame(selectionFrame);

               weightMatrix6D.getFullWeightMatrixInFrame(destinationFrame, expectedSelectionMatrix);
               MatrixTools.removeZeroRows(expectedSelectionMatrix, 1.0e-7);

               weightMatrix6D.getCompactWeightMatrixInFrame(destinationFrame, actualSelectionMatrix);

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
      mutationTestFacilitator.addClassesToMutate(SelectionMatrix3D.class, WeightMatrix6D.class);
      mutationTestFacilitator.addTestClassesToRun(SelectionMatrix3DTest.class, WeightMatrix6DTest.class);
      mutationTestFacilitator.doMutationTest();
      mutationTestFacilitator.openResultInBrowser();
   }
}
