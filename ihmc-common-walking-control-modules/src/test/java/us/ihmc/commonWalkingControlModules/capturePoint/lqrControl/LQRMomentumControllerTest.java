package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.EjmlUnitTests;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.matrixlib.NativeCommonOps;
import us.ihmc.robotics.math.trajectories.Trajectory3D;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class LQRMomentumControllerTest
{
   private static final double epsilon = 1e-10;

   @Test
   public void test()
   {
      LQRMomentumController controller = new LQRMomentumController();

      Point3D vrpStart = new Point3D(0.0, 0.0, 1.0);
      Point3D vrpEnd = new Point3D(1.0, 0.5, 1.0);
      Trajectory3D vrpTrajectory = new Trajectory3D(4);
      vrpTrajectory.setLinear(0.0, 1.0, vrpStart, vrpEnd);
      List<Trajectory3D> trajectories = new ArrayList<>();
      trajectories.add(vrpTrajectory);

      controller.setVrpTrajectory(trajectories);

      DenseMatrix64F AExpected = new DenseMatrix64F(6, 6);
      AExpected.set(0, 3, 1.0);
      AExpected.set(1, 4, 1.0);
      AExpected.set(2, 5, 1.0);

      DenseMatrix64F BExpected = new DenseMatrix64F(6, 3);
      BExpected.set(3, 0, 1.0);
      BExpected.set(4, 1, 1.0);
      BExpected.set(5, 2, 1.0);

      DenseMatrix64F CExpected = new DenseMatrix64F(3, 6);

      CExpected.set(0, 0, 1.0);
      CExpected.set(1, 1, 1.0);
      CExpected.set(2, 2, 1.0);

      DenseMatrix64F DExpected = new DenseMatrix64F(3, 3);

      DExpected.set(0, 0, -1.0 / MathTools.square(LQRMomentumController.omega));
      DExpected.set(1, 1, -1.0 / MathTools.square(LQRMomentumController.omega));
      DExpected.set(2, 2, -1.0 / MathTools.square(LQRMomentumController.omega));

      EjmlUnitTests.assertEquals(AExpected, controller.A, epsilon);
      EjmlUnitTests.assertEquals(BExpected, controller.B, epsilon);
      EjmlUnitTests.assertEquals(CExpected, controller.C, epsilon);
      EjmlUnitTests.assertEquals(DExpected, controller.D, epsilon);

      DenseMatrix64F QExpected = new DenseMatrix64F(3, 3);
      QExpected.set(0, 0, LQRMomentumController.defaultVrpTrackingWeight);
      QExpected.set(1, 1, LQRMomentumController.defaultVrpTrackingWeight);
      QExpected.set(2, 2, LQRMomentumController.defaultVrpTrackingWeight);

      DenseMatrix64F RExpected = new DenseMatrix64F(3, 3);
      RExpected.set(0, 0, LQRMomentumController.defaultMomentumRateWeight);
      RExpected.set(1, 1, LQRMomentumController.defaultMomentumRateWeight);
      RExpected.set(2, 2, LQRMomentumController.defaultMomentumRateWeight);

      EjmlUnitTests.assertEquals(QExpected, controller.Q, epsilon);
      EjmlUnitTests.assertEquals(RExpected, controller.R, epsilon);

      DenseMatrix64F Q1Expected = new DenseMatrix64F(3, 3);
      NativeCommonOps.multQuad(CExpected, QExpected, Q1Expected);

      DenseMatrix64F R1Expected = new DenseMatrix64F(3, 3);
      DenseMatrix64F R1InverseExpected = new DenseMatrix64F(3, 3);
      NativeCommonOps.multQuad(DExpected, QExpected, R1Expected);
      CommonOps.addEquals(R1Expected, RExpected);
      NativeCommonOps.invert(R1Expected, R1InverseExpected);

      EjmlUnitTests.assertEquals(Q1Expected, controller.Q1, epsilon);
      EjmlUnitTests.assertEquals(R1Expected, controller.R1, epsilon);
      EjmlUnitTests.assertEquals(R1InverseExpected, controller.R1Inverse, epsilon);

      DenseMatrix64F NExpected = new DenseMatrix64F(6, 3);
      DenseMatrix64F NTransposeExpected = new DenseMatrix64F(3, 6);
      DenseMatrix64F tempMatrix = new DenseMatrix64F(3, 3);
      CommonOps.mult(QExpected, DExpected, tempMatrix);
      CommonOps.multTransA(CExpected, tempMatrix, NExpected);
      CommonOps.transpose(NExpected, NTransposeExpected);

      EjmlUnitTests.assertEquals(NExpected, controller.N, epsilon);
      EjmlUnitTests.assertEquals(NTransposeExpected, controller.NTranspose, epsilon);


         controller.computeS1();

      EjmlUnitTests.assertEquals(AExpected, controller.A, epsilon);
      EjmlUnitTests.assertEquals(BExpected, controller.B, epsilon);
      EjmlUnitTests.assertEquals(CExpected, controller.C, epsilon);
      EjmlUnitTests.assertEquals(DExpected, controller.D, epsilon);

      EjmlUnitTests.assertEquals(QExpected, controller.Q, epsilon);
      EjmlUnitTests.assertEquals(RExpected, controller.R, epsilon);

      EjmlUnitTests.assertEquals(Q1Expected, controller.Q1, epsilon);
      EjmlUnitTests.assertEquals(R1Expected, controller.R1, epsilon);
      EjmlUnitTests.assertEquals(R1InverseExpected, controller.R1Inverse, epsilon);

      EjmlUnitTests.assertEquals(NExpected, controller.N, epsilon);

      DenseMatrix64F QRiccatiExpected = new DenseMatrix64F(3, 3);
      DenseMatrix64F ARiccatiExpected = new DenseMatrix64F(6, 6);

      NativeCommonOps.multQuad(NTransposeExpected, R1InverseExpected, QRiccatiExpected);
      CommonOps.scale(-1.0, QRiccatiExpected);
      CommonOps.addEquals(QRiccatiExpected, Q1Expected);

      tempMatrix = new DenseMatrix64F(3, 6);
      CommonOps.multTransB(R1InverseExpected, NExpected, tempMatrix);
      CommonOps.mult(-1.0, BExpected, tempMatrix, ARiccatiExpected);
      CommonOps.addEquals(ARiccatiExpected, AExpected);

      EjmlUnitTests.assertEquals(QRiccatiExpected, controller.QRiccati, epsilon);
      EjmlUnitTests.assertEquals(ARiccatiExpected, controller.ARiccati, epsilon);


      Random random = new Random(1738L);
      DenseMatrix64F S1 = controller.S1;
//      DenseMatrix64F S1Constructor = new DenseMatrix64F(6, 6);
//      S1Constructor.setData(RandomNumbers.nextDoubleArray(random, 36, 10.0));
//      CommonOps.transpose(S1Constructor, S1);
//      CommonOps.addEquals(S1, S1Constructor);
//
      DenseMatrix64F NB = new DenseMatrix64F(NExpected);
      CommonOps.transpose(NB);
      CommonOps.multAddTransA(BExpected, S1, NB);
      DenseMatrix64F S1DotExpected = new DenseMatrix64F(6, 6);
      NativeCommonOps.multQuad(NB, R1InverseExpected, S1DotExpected);
      CommonOps.addEquals(S1DotExpected, -1.0, Q1Expected);
      CommonOps.multAdd(-1.0, S1, AExpected, S1DotExpected);
      CommonOps.multAddTransA(-1.0, AExpected, S1, S1DotExpected);

      DenseMatrix64F H = new DenseMatrix64F( 4, 4);
      H.set(0, 1, 1.0);
      H.set(1, 0, 0.5);
      H.set(1, 3, -1.0);
      H.set(2, 0, -0.5);
      H.set(2, 3, -0.5);
      H.set(3, 2, -1.0);



      tempMatrix.reshape(6, 3);
      DenseMatrix64F S1Dot = new DenseMatrix64F(6, 6);
      DenseMatrix64F BTranspose = new DenseMatrix64F(controller.B);
      CommonOps.transpose(BTranspose);
      NativeCommonOps.multQuad(BTranspose, controller.R1Inverse, tempMatrix);
      NativeCommonOps.multQuad(S1, tempMatrix, S1Dot);
      CommonOps.addEquals(S1Dot, -1.0, controller.QRiccati);
      CommonOps.multAdd(-1.0, S1, controller.ARiccati, S1Dot);
      CommonOps.multAddTransA(-1.0, controller.ARiccati, S1, S1Dot);

      EjmlUnitTests.assertEquals(S1DotExpected, S1Dot, epsilon);
      EjmlUnitTests.assertEquals(new DenseMatrix64F(6, 6), S1Dot, epsilon);
   }
}
