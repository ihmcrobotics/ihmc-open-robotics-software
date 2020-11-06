package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.interfaces.linsol.LinearSolverDense;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

public class TakeOffHeightObjectivePolicyTest
{
   @Test
   public void testPolicy()
   {
      double nominalHeight = 1.0;
      double contactHeight = 0.3;
      CoMTrajectoryPlanner planner = new CoMTrajectoryPlanner(9.81, nominalHeight, new YoRegistry("test"));
      List<SettableContactStateProvider> contactSequence = new ArrayList<>();

      SettableContactStateProvider contact0 = new SettableContactStateProvider();
      SettableContactStateProvider contact1 = new SettableContactStateProvider();
      contact0.setContactState(ContactState.IN_CONTACT);
      contact0.setEndCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, contactHeight));
      contact1.setContactState(ContactState.FLIGHT);

      contactSequence.add(contact0);
      contactSequence.add(contact1);

      DMatrixRMaj hessian = new DMatrixRMaj(12, 12);
      DMatrixRMaj zGradient = new DMatrixRMaj(12, 1);
      DMatrixRMaj zCoefficients = new DMatrixRMaj(12, 1);

      MatrixTools.addDiagonal(hessian, 1e-4);

      TakeOffHeightObjectivePolicy policy = new TakeOffHeightObjectivePolicy(() -> 3.0, 1.0);
      policy.assessPolicy(planner, contactSequence, hessian, null, null, zGradient);

      LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.lu(6);
      MatrixTools.addDiagonal(hessian, 1e-5);
      CommonOps_DDRM.scale(-0.5, zGradient);
      solver.setA(hessian);
      solver.solve(zGradient, zCoefficients);

      double startingHeight = 0.0;
      for (int i = 6; i < 12; i++)
         startingHeight += zCoefficients.get(i) * CoMTrajectoryPlannerTools.getCoMPositionCoefficientTimeFunction(i - 6, 3.0, 0.0);

      assertEquals(nominalHeight + contactHeight, startingHeight, 1e-4);
   }
}
