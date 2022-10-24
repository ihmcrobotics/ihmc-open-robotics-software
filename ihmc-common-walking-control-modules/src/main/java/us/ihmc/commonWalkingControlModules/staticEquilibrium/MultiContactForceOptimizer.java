package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.convexOptimization.quadraticProgram.SimpleEfficientActiveSetQPSolver;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.commonWalkingControlModules.staticEquilibrium.ContactPoint.basisVectorsPerContactPoint;

public class MultiContactForceOptimizer
{
   private static final int staticEquilibriumConstraints = 6;
   static final double mg = 1.0;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

   private final List<ContactPoint> contactPoints = new ArrayList<>();
   private MultiContactSupportRegionSolverInput input;

   private final DMatrixRMaj Aeq = new DMatrixRMaj(0);
   private final DMatrixRMaj beq = new DMatrixRMaj(0);
   private final DMatrixRMaj Ain = new DMatrixRMaj(0);
   private final DMatrixRMaj bin = new DMatrixRMaj(0);
   private final DMatrixRMaj rho = new DMatrixRMaj(0);
   private final DMatrixRMaj quadraticCost = new DMatrixRMaj(0);
   private final DMatrixRMaj linearCost = new DMatrixRMaj(0);

   private final SimpleEfficientActiveSetQPSolver qpSolver = new SimpleEfficientActiveSetQPSolver();

   public MultiContactForceOptimizer()
   {
      for (int i = 0; i < MultiContactSupportRegionSolverInput.maxContactPoints; i++)
      {
         contactPoints.add(new ContactPoint(i, registry, graphicsListRegistry));
      }
   }

   public void solve(MultiContactSupportRegionSolverInput input, double comX, double comY)
   {
      clear();

      int rhoSize = basisVectorsPerContactPoint * input.getNumberOfContacts();
      int decisionVariables = rhoSize;

      rho.reshape(rhoSize, 1);
      Aeq.reshape(staticEquilibriumConstraints, decisionVariables);
      beq.reshape(staticEquilibriumConstraints, 1);
      Ain.reshape(rhoSize, rhoSize);
      bin.reshape(rhoSize, 1);
      quadraticCost.reshape(rhoSize, rhoSize);
      linearCost.reshape(rhoSize, 1);

      CommonOps_DDRM.setIdentity(Ain);
      CommonOps_DDRM.setIdentity(quadraticCost);
      CommonOps_DDRM.scale(-1.0, Ain);
      CommonOps_DDRM.fill(beq, 0.0);

      for (int i = 0; i < contactPoints.size(); i++)
      {
         contactPoints.get(i).clear();
      }

      for (int i = 0; i < input.getNumberOfContacts(); i++)
      {
         ContactPoint contactPoint = contactPoints.get(i);
         contactPoint.initialize(input);

         FramePoint3D contactPointPosition = input.getContactPointPositions().get(i);

         for (int j = 0; j < basisVectorsPerContactPoint; j++)
         {
            YoFrameVector3D basisVector = contactPoint.getBasisVector(j);
            int column = basisVectorsPerContactPoint * i + j;

            Aeq.set(0, column, basisVector.getX());
            Aeq.set(1, column, basisVector.getY());
            Aeq.set(2, column, basisVector.getZ());

            // x-component of cross product
            double xMomentScale = contactPointPosition.getY() * basisVector.getZ() - contactPointPosition.getZ() * basisVector.getY();
            Aeq.set(3, column, xMomentScale);

            // y-component of cross product
            double yMomentScale = contactPointPosition.getZ() * basisVector.getX() - contactPointPosition.getX() * basisVector.getZ();
            Aeq.set(4, column, yMomentScale);

            // z-component of cross product
            double zMomentScale = contactPointPosition.getX() * basisVector.getY() - contactPointPosition.getY() * basisVector.getX();
            Aeq.set(5, column, zMomentScale);
         }
      }

      beq.set(2, 0, mg);
      beq.set(3, 0, mg * comY);
      beq.set(4, 0, -mg * comX);

      qpSolver.clear();
      qpSolver.resetActiveSet();

      qpSolver.setMaxNumberOfIterations(500);
      qpSolver.setConvergenceThreshold(1e-9);
      qpSolver.setQuadraticCostFunction(quadraticCost, linearCost, 0.0);
      qpSolver.setLinearInequalityConstraints(Aeq, beq);
      qpSolver.setLinearInequalityConstraints(Ain, bin);

      qpSolver.solve(rho);

      for (int i = 0; i < input.getNumberOfContacts(); i++)
      {
         contactPoints.get(i).setResolvedForce(rho.getData());
      }
   }

   public Tuple3DReadOnly getResolvedForce(int i)
   {
      return contactPoints.get(i).getResolvedForce();
   }

   private void clear()
   {
      Aeq.zero();
      beq.zero();
      Ain.zero();
      bin.zero();
      quadraticCost.zero();
      linearCost.zero();
      rho.zero();
   }

}
