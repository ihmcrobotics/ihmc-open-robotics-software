package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.Random;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.matrixlib.NativeMatrix;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;

public class SelectionCalculatorTest
{
   private static final boolean visualize = false;

   private static final double regularizationWeight = 1.0E-6;
   private static final double objectiveWeight = 1.0;

   private static final double EPSILON = 0.001;

   private static final Random random = new Random(1971L);

   @Test
   public void testSelectionForSimpleObjective() throws Exception
   {
      ReferenceFrame taskFrame = ReferenceFrame.getWorldFrame();
      SelectionMatrix3D selectionMatrix = new SelectionMatrix3D();
      ReferenceFrame selectionFrame;
      Vector3D objective;
      Vector3D result;

      for (int i = 0; i < 1000; i++)
      {
         RigidBodyTransform selectionFrameTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         selectionFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent("SelectionFrame", ReferenceFrame.getWorldFrame(), selectionFrameTransform);
         objective = EuclidCoreRandomTools.nextVector3D(random);
         selectionMatrix.setSelectionFrame(selectionFrame);
         selectionMatrix.selectXAxis(random.nextBoolean());
         selectionMatrix.selectYAxis(random.nextBoolean());
         selectionMatrix.selectZAxis(random.nextBoolean());

         result = runTest(taskFrame, selectionMatrix, objective);
      }

      if (visualize)
      {
         visualize(taskFrame, selectionFrame, objective, result);
      }
   }

   private static Vector3D runTest(ReferenceFrame taskFrame, SelectionMatrix3D selectionMatrix, Vector3D objective)
   {
      // Setup an optimization such that we minimize
      // min 0.5 x' H x + f' x
      DMatrixRMaj H = new DMatrixRMaj(3, 3);
      DMatrixRMaj f = new DMatrixRMaj(3, 1);

      // Add a regularization so all unselected axes will be zeroed.
      CommonOps_DDRM.fill(f, 0.0);
      CommonOps_DDRM.setIdentity(H);
      CommonOps_DDRM.scale(regularizationWeight, H);

      WeightMatrix3D weightMatrix = new WeightMatrix3D();
      weightMatrix.setWeightFrame(selectionMatrix.getSelectionFrame());
      if (selectionMatrix.isXSelected())
      {
         weightMatrix.setXAxisWeight(objectiveWeight);
      }
      if (selectionMatrix.isYSelected())
      {
         weightMatrix.setYAxisWeight(objectiveWeight);
      }
      if (selectionMatrix.isZSelected())
      {
         weightMatrix.setZAxisWeight(objectiveWeight);
      }

      NativeMatrix taskJacobian = new NativeMatrix(0, 0);
      NativeMatrix taskObjective = new NativeMatrix(0, 0);
      computeTask(objective, taskJacobian, taskObjective);

      SelectionCalculator selectionCalculator = new SelectionCalculator();

      NativeMatrix taskJacobianAfterSelection = new NativeMatrix(0, 0);
      NativeMatrix taskObjectiveAfterSelection = new NativeMatrix(0, 0);
      NativeMatrix taskWeightAfterSelection = new NativeMatrix(0, 0);
      selectionCalculator.applySelectionToTask(selectionMatrix, weightMatrix, taskFrame, taskJacobian, taskObjective, taskJacobianAfterSelection,
                                               taskObjectiveAfterSelection, taskWeightAfterSelection);

      addTask(taskJacobianAfterSelection, taskObjectiveAfterSelection, taskWeightAfterSelection, H, f);

      // Without any constraints the solution to this problem is
      // H x = f
      DMatrixRMaj x = (new SimpleMatrix(H)).invert().mult(new SimpleMatrix(f)).getMatrix();
      Vector3D result = new Vector3D(x.get(0), x.get(1), x.get(2));

      FrameVector3D frameResult = new FrameVector3D(taskFrame, result);
      FrameVector3D frameObjective = new FrameVector3D(taskFrame, objective);
      frameResult.changeFrame(selectionMatrix.getSelectionFrame());
      frameObjective.changeFrame(selectionMatrix.getSelectionFrame());
      for (int i = 0; i < 3; i++)
      {
         if (!selectionMatrix.isAxisSelected(i))
         {
            Assert.assertEquals(0.0, frameResult.getElement(i), EPSILON);
         }
         else
         {
            Assert.assertEquals(frameObjective.getElement(i), frameResult.getElement(i), EPSILON);
         }
      }
      return result;
   }

   private static void visualize(ReferenceFrame taskFrame, ReferenceFrame selectionFrame, Vector3D objective, Vector3D result)
   {
      //      YoRegistry registry = new YoRegistry("TestRegistry");
      //      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      //
      //      YoGraphicCoordinateSystem taskFrameViz = new YoGraphicCoordinateSystem("TaskFrame", "Viz", registry, 1.0, YoAppearance.Blue());
      //      taskFrameViz.setToReferenceFrame(taskFrame);
      //      graphicsListRegistry.registerYoGraphic("ReferenceFrames", taskFrameViz);
      //
      //      YoGraphicCoordinateSystem selectionFrameViz = new YoGraphicCoordinateSystem("SelectionFrame", "Viz", registry, 1.0, YoAppearance.Red());
      //      selectionFrameViz.setToReferenceFrame(selectionFrame);
      //      graphicsListRegistry.registerYoGraphic("ReferenceFrames", selectionFrameViz);
      //
      //      YoFramePoint3D origin = new YoFramePoint3D("Origin", ReferenceFrame.getWorldFrame(), registry);
      //      YoFrameVector3D yoResult = new YoFrameVector3D("Result", ReferenceFrame.getWorldFrame(), registry);
      //      yoResult.set(result);
      //      YoGraphicVector resultViz = new YoGraphicVector("Result", origin, yoResult, 1.2, YoAppearance.Green());
      //      graphicsListRegistry.registerYoGraphic("Optimization", resultViz);
      //
      //      YoFrameVector3D yoObjective = new YoFrameVector3D("Objective", ReferenceFrame.getWorldFrame(), registry);
      //      yoObjective.set(objective);
      //      YoGraphicVector objectiveViz = new YoGraphicVector("Objective", origin, yoObjective, 1.1, YoAppearance.Blue());
      //      graphicsListRegistry.registerYoGraphic("Optimization", objectiveViz);
      //
      //      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Test"));
      //      scs.getRootRegistry().addChild(registry);
      //      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      //      scs.startOnAThread();
      //      ThreadTools.sleepForever();
   }

   private static void computeTask(Vector3D objective, NativeMatrix taskJacobianToPack, NativeMatrix taskObjectiveToPack)
   {
      taskJacobianToPack.reshape(3, 3);
      taskJacobianToPack.zero();
      taskJacobianToPack.fillDiagonal(1.0);

      taskObjectiveToPack.reshape(3, 1);
      objective.get(taskObjectiveToPack);
   }

   private static void addTask(NativeMatrix taskJacobian, NativeMatrix taskObjective, NativeMatrix taskWeight, DMatrixRMaj hToModify,
                               DMatrixRMaj fToModify)
   {
      SimpleMatrix J = new SimpleMatrix(new DMatrixRMaj(taskJacobian));
      SimpleMatrix W = new SimpleMatrix(new DMatrixRMaj(taskWeight));
      SimpleMatrix b = new SimpleMatrix(new DMatrixRMaj(taskObjective));

      SimpleMatrix H = J.transpose().mult(W).mult(J);
      SimpleMatrix f = J.transpose().mult(W).mult(b);

      CommonOps_DDRM.add(hToModify, H.getMatrix(), hToModify);
      CommonOps_DDRM.add(fToModify, f.getMatrix(), fToModify);
   }
}
