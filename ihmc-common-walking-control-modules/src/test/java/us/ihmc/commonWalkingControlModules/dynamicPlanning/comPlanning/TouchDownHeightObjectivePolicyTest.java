package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

public class TouchDownHeightObjectivePolicyTest
{
   @Test
   public void testPolicy()
   {
      double contactHeight = 0.3;
      double omega = 3.0;
      double duration = 2.5;
      YoDouble yoOmega = new YoDouble("omega", new YoRegistry("test"));
      yoOmega.set(omega);
      CoMTrajectoryPlanner planner = new CoMTrajectoryPlanner(9.81, yoOmega, new YoRegistry("test"));
      List<SettableContactStateProvider> contactSequence = new ArrayList<>();

      SettableContactStateProvider contact0 = new SettableContactStateProvider();
      SettableContactStateProvider contact1 = new SettableContactStateProvider();
      contact0.setContactState(ContactState.FLIGHT);
      contact0.getTimeInterval().setInterval(0.0, duration);
      contact1.setContactState(ContactState.IN_CONTACT);
      contact1.setStartCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, contactHeight));

      contactSequence.add(contact0);
      contactSequence.add(contact1);

      DMatrixRMaj hessian = new DMatrixRMaj(12, 12);
      DMatrixRMaj zGradient = new DMatrixRMaj(12, 1);
      DMatrixRMaj zCoefficients = new DMatrixRMaj(12, 1);

      MatrixTools.addDiagonal(hessian, 1e-4);

      TouchDownHeightObjectivePolicy policy = new TouchDownHeightObjectivePolicy(yoOmega, 1.0);
      policy.assessPolicy(planner, contactSequence, hessian, null, null, zGradient);

      LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.lu(6);
      MatrixTools.addDiagonal(hessian, 1e-5);
      CommonOps_DDRM.scale(-0.5, zGradient);

      solver.setA(hessian);
      solver.solve(zGradient, zCoefficients);

      double startingHeight = 0.0;
      for (int i = 0; i < 6; i++)
         startingHeight += zCoefficients.get(i) * CoMTrajectoryPlannerTools.getCoMPositionCoefficientTimeFunction(i, omega, duration);

      assertEquals(planner.getNominalCoMHeight() + contactHeight, startingHeight, 1e-4);
   }
}
