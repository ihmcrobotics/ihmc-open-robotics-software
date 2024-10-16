package us.ihmc.robotics.weightMatrices;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.ejml.dense.row.RandomMatrices_DDRM;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FrameMatrix3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.matrixlib.MatrixTools;

public class WeightMatrix3DTest
{
   private static final int ITERATIONS = 1000;

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testSettersGetters()
   {
      Random random = new Random(123423L);

      WeightMatrix3D weightMatrix3D = new WeightMatrix3D();
      assertNull(weightMatrix3D.getWeightFrame());
      assertTrue(Double.isNaN(weightMatrix3D.getXAxisWeight()));
      assertTrue(Double.isNaN(weightMatrix3D.getYAxisWeight()));
      assertTrue(Double.isNaN(weightMatrix3D.getZAxisWeight()));

      for (int i = 0; i < ITERATIONS; i++)
      {
         double xWeight = random.nextDouble();
         double yWeight = random.nextDouble();
         double zWeight = random.nextDouble();
         weightMatrix3D.setWeights(xWeight, yWeight, zWeight);
         assertEquals(xWeight, weightMatrix3D.getXAxisWeight(), 1e-8);
         assertEquals(yWeight, weightMatrix3D.getYAxisWeight(), 1e-8);
         assertEquals(zWeight, weightMatrix3D.getZAxisWeight(), 1e-8);

         xWeight = random.nextDouble();
         yWeight = random.nextDouble();
         zWeight = random.nextDouble();

         weightMatrix3D.setXAxisWeight(xWeight);
         weightMatrix3D.setYAxisWeight(yWeight);
         weightMatrix3D.setZAxisWeight(zWeight);
         assertEquals(xWeight, weightMatrix3D.getXAxisWeight(), 1e-8);
         assertEquals(yWeight, weightMatrix3D.getYAxisWeight(), 1e-8);
         assertEquals(zWeight, weightMatrix3D.getZAxisWeight(), 1e-8);

         weightMatrix3D.clear();
         assertNull(weightMatrix3D.getWeightFrame());
         assertTrue(Double.isNaN(weightMatrix3D.getXAxisWeight()));
         assertTrue(Double.isNaN(weightMatrix3D.getYAxisWeight()));
         assertTrue(Double.isNaN(weightMatrix3D.getZAxisWeight()));

         ReferenceFrame randomFrame = EuclidFrameRandomTools.nextReferenceFrame("blop" + i, random, ReferenceFrame.getWorldFrame());
         weightMatrix3D.setWeightFrame(randomFrame);
         assertEquals(randomFrame, weightMatrix3D.getWeightFrame());

         weightMatrix3D.clearWeightFrame();
         assertNull(weightMatrix3D.getWeightFrame());

         weightMatrix3D.setWeightFrame(randomFrame);
         weightMatrix3D.clear();
         assertNull(weightMatrix3D.getWeightFrame());

         weightMatrix3D.setWeightFrame(randomFrame);
         weightMatrix3D.clear();
         assertNull(weightMatrix3D.getWeightFrame());
      }
   }

   @Test
   public void testGetFullSelectionMatrixInFrame()
   {
      Random random = new Random(456465L);
      WeightMatrix3D weightMatrix3D = new WeightMatrix3D();
      FrameMatrix3D frameMatrix3D = new FrameMatrix3D();

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
               DMatrixRMaj actualWeightMatrix = RandomMatrices_DDRM.rectangle(3, 3, -1.0, 1.0, random);
               DMatrixRMaj expectedWeightMatrix = RandomMatrices_DDRM.rectangle(3, 3, -1.0, 1.0, random);

               double xWeight = random.nextDouble();
               double yWeight = random.nextDouble();
               double zWeight = random.nextDouble();

               weightMatrix3D.setWeights(xWeight, yWeight, zWeight);
               weightMatrix3D.setWeightFrame(selectionFrame);

               frameMatrix3D.setToZero(selectionFrame);
               frameMatrix3D.setM00(xWeight);
               frameMatrix3D.setM11(yWeight);
               frameMatrix3D.setM22(zWeight);

               weightMatrix3D.getFullWeightMatrixInFrame(destinationFrame, actualWeightMatrix);
               if (selectionFrame != null)
                  frameMatrix3D.changeFrame(destinationFrame);
               frameMatrix3D.get(expectedWeightMatrix);
               assertMatrixEquals(expectedWeightMatrix, actualWeightMatrix, 1.0e-12);
               
             // Verifies that it has the intended application: Being able to apply the selection to any frame
             DMatrixRMaj expectedSubspaceVector = new DMatrixRMaj(3, 1);
             DMatrixRMaj actualSubspaceVector = new DMatrixRMaj(3, 1);

             FrameVector3D randomVector = EuclidFrameRandomTools.nextFrameVector3D(random, destinationFrame);
             DMatrixRMaj originalVector = new DMatrixRMaj(3, 1);
             randomVector.get(originalVector);
             weightMatrix3D.getFullWeightMatrixInFrame(destinationFrame, expectedWeightMatrix);
             CommonOps_DDRM.mult(expectedWeightMatrix, originalVector, actualSubspaceVector);

             
             FrameVector3D weights = new FrameVector3D();
             weights.set(xWeight, yWeight, zWeight);
             
             if (selectionFrame != null)
             {
                weights.setIncludingFrame(selectionFrame, xWeight, yWeight, zWeight);
                randomVector.changeFrame(selectionFrame);
             }
                
             randomVector.setX(randomVector.getX() * weights.getX());
             randomVector.setY(randomVector.getY() * weights.getY());
             randomVector.setZ(randomVector.getZ() * weights.getZ());

             if (selectionFrame != null)
                randomVector.changeFrame(destinationFrame);
             randomVector.get(expectedSubspaceVector);

             assertMatrixEquals(expectedSubspaceVector, actualSubspaceVector, 1.0e-12);
            }
         }
      }
   }

   @Test
   public void testGetEfficientSelectionMatrixInFrame()
   {
      Random random = new Random(456465L);
      WeightMatrix3D weightMatrix3D = new WeightMatrix3D();

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
               DMatrixRMaj actualSelectionMatrix = RandomMatrices_DDRM.rectangle(3, 3, -1.0, 1.0, random);
               DMatrixRMaj expectedSelectionMatrix = RandomMatrices_DDRM.rectangle(3, 3, -1.0, 1.0, random);

               double xWeight = random.nextDouble();
               double yWeight = random.nextDouble();
               double zWeight = random.nextDouble();

               weightMatrix3D.setWeights(xWeight, yWeight, zWeight);
               weightMatrix3D.setWeightFrame(selectionFrame);

               weightMatrix3D.getFullWeightMatrixInFrame(destinationFrame, expectedSelectionMatrix);
               MatrixTools.removeZeroRows(expectedSelectionMatrix, 1.0e-6);

               weightMatrix3D.getEfficientWeightMatrixInFrame(destinationFrame, actualSelectionMatrix);

               assertMatrixEquals(expectedSelectionMatrix, actualSelectionMatrix, 1.0e-12);
            }
         }
      }
   }

   private static void assertMatrixEquals(DMatrixRMaj expected, DMatrixRMaj actual, double epsilon)
   {
      assertTrue(MatrixFeatures_DDRM.isEquals(expected, actual, epsilon), assertErrorMessage(expected, actual));
   }

   private static String assertErrorMessage(DMatrixRMaj expected, DMatrixRMaj actual)
   {
      return "Expected:\n" + expected + "\nActual:\n" + actual;
   }
}
