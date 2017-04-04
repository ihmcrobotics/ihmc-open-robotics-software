package us.ihmc.robotics.screwTheory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import org.ejml.ops.SpecializedOps;

/**
 *
 * See: L. Sentis. Synthesis and Control of Whole-Body Behaviors in Humanoid Systems (2007)
 *
 * @author twan
 *         Date: 4/15/13
 */
public class OriginalDynamicallyConsistentNullspaceCalculator implements DynamicallyConsistentNullspaceCalculator
{
   private final FloatingInverseDynamicsJoint rootJoint;
   private final InverseDynamicsJoint[] jointsInOrder;
   private final Map<RigidBody, DenseMatrix64F> constrainedBodiesAndSelectionMatrices = new LinkedHashMap<RigidBody, DenseMatrix64F>();
   private final List<InverseDynamicsJoint> actuatedJoints = new ArrayList<InverseDynamicsJoint>();
   private final Map<RigidBody, List<InverseDynamicsJoint>> supportingBodyToJointPathMap = new LinkedHashMap<RigidBody, List<InverseDynamicsJoint>>();
   private final MassMatrixCalculator massMatrixCalculator;

   private final boolean computeSNsBar;

   private final LinearSolver<DenseMatrix64F> massMatrixSolver;
   private final LinearSolver<DenseMatrix64F> lambdaSolver;
   private final LinearSolver<DenseMatrix64F> pseudoInverseSolver;
   private final DenseMatrix64F S = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F Js = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F AInverse = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F AInverseJSTranspose = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F LambdasInverse = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F Lambdas = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F JsBar = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F Ns = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F SNs = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F AInverseSNsTranspose = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F PhiStar = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F PhiStarInverse = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F SNsBar = new DenseMatrix64F(1, 1);

   private final Twist tempTwist = new Twist();
   private final DenseMatrix64F tempTwistMatrix = new DenseMatrix64F(Twist.SIZE, 1);
   private final DenseMatrix64F tempJacobianPart = new DenseMatrix64F(1, 1);

   private final int nDegreesOfFreedom;
   private int nConstraints;

   public OriginalDynamicallyConsistentNullspaceCalculator(FloatingInverseDynamicsJoint rootJoint,
                                                           boolean computeSNsBar)
   {
      this.rootJoint = rootJoint;
      this.massMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(rootJoint.getSuccessor());
      jointsInOrder = massMatrixCalculator.getJointsInOrder();
      this.nDegreesOfFreedom = ScrewTools.computeDegreesOfFreedom(jointsInOrder);
      massMatrixSolver = LinearSolverFactory.symmPosDef(nDegreesOfFreedom);
      lambdaSolver = LinearSolverFactory.symmPosDef(nDegreesOfFreedom); // size of matrix is only used to choose algorithm. nDegreesOfFreedom is an upper limit
      pseudoInverseSolver = LinearSolverFactory.pseudoInverse(true);
      this.computeSNsBar = computeSNsBar;
   }

   @Override
   public void reset()
   {
      nConstraints = 0;
      constrainedBodiesAndSelectionMatrices.clear();
      actuatedJoints.clear();
   }

   @Override
   public void addConstraint(RigidBody body, DenseMatrix64F selectionMatrix)
   {
      constrainedBodiesAndSelectionMatrices.put(body, selectionMatrix);
      nConstraints += selectionMatrix.getNumRows();
      InverseDynamicsJoint[] jointPath = ScrewTools.createJointPath(rootJoint.getPredecessor(), body);
      this.supportingBodyToJointPathMap.put(body, Arrays.asList(jointPath));
   }

   @Override
   public void addActuatedJoint(InverseDynamicsJoint joint)
   {
      actuatedJoints.add(joint);
   }

   private static void computeS(DenseMatrix64F S, InverseDynamicsJoint[] jointsInOrder, Collection<InverseDynamicsJoint> actuatedJoints)
   {
      S.zero();
      int rowStart = 0;
      int columnStart = 0;
      for (InverseDynamicsJoint inverseDynamicsJoint : jointsInOrder)
      {
         int degreesOfFreedom = inverseDynamicsJoint.getDegreesOfFreedom();

         if (actuatedJoints.contains(inverseDynamicsJoint))
         {
            DenseMatrix64F identity = new DenseMatrix64F(degreesOfFreedom, degreesOfFreedom);
            CommonOps.setIdentity(identity);
            CommonOps.insert(identity, S, rowStart, columnStart);
            rowStart += degreesOfFreedom;
         }

         columnStart += degreesOfFreedom;
      }
   }

   @Override
   public void compute()
   {
      resizeMatrices();

      computeS(S, jointsInOrder, actuatedJoints);

      computeJs(Js, supportingBodyToJointPathMap, constrainedBodiesAndSelectionMatrices);

      massMatrixCalculator.compute();
      massMatrixSolver.setA(massMatrixCalculator.getMassMatrix());
      massMatrixSolver.invert(AInverse);
      CommonOps.multTransB(AInverse, Js, AInverseJSTranspose);
      CommonOps.mult(Js, AInverseJSTranspose, LambdasInverse);

      lambdaSolver.setA(LambdasInverse);
      lambdaSolver.invert(Lambdas);
      CommonOps.mult(AInverseJSTranspose, Lambdas, JsBar);

//      CommonOps.pinv(Js, JsBar);

      CommonOps.mult(JsBar, Js, Ns);
      CommonOps.changeSign(Ns);
      SpecializedOps.addIdentity(Ns, Ns, 1.0);

      if (computeSNsBar)
      {
         CommonOps.mult(S, Ns, SNs);
         CommonOps.multTransB(AInverse, SNs, AInverseSNsTranspose);
         CommonOps.mult(SNs, AInverseSNsTranspose, PhiStar);
         pseudoInverseSolver.setA(PhiStar);
         pseudoInverseSolver.invert(PhiStarInverse);
         CommonOps.mult(AInverseSNsTranspose, PhiStarInverse, SNsBar);

//         CommonOps.mult(S, Ns, SNs);
//         ConfigurableSolvePseudoInverseSVD solver = new ConfigurableSolvePseudoInverseSVD(SNs.getNumRows(), SNs.getNumCols(), 0.5);
//         solver.setA(SNs);
//         solver.invert(SNsBar);
      }
   }

   private void resizeMatrices()
   {
      int nActuatedDegreesOfFreedom = ScrewTools.computeDegreesOfFreedom(actuatedJoints);

      S.reshape(nActuatedDegreesOfFreedom, nDegreesOfFreedom);
      Js.reshape(nConstraints, nDegreesOfFreedom);
      AInverse.reshape(nDegreesOfFreedom, nDegreesOfFreedom);
      AInverseJSTranspose.reshape(AInverse.getNumRows(), Js.getNumRows());
      LambdasInverse.reshape(Js.getNumRows(), AInverseJSTranspose.getNumCols());
      Lambdas.reshape(LambdasInverse.getNumRows(), LambdasInverse.getNumCols());
      JsBar.reshape(AInverseJSTranspose.getNumRows(), LambdasInverse.getNumCols());
      Ns.reshape(nDegreesOfFreedom, nDegreesOfFreedom);
      SNs.reshape(S.getNumRows(), Ns.getNumCols());
      AInverseSNsTranspose.reshape(AInverse.getNumRows(), SNs.getNumRows());
      PhiStar.reshape(SNs.getNumRows(), AInverseSNsTranspose.getNumCols());
      PhiStarInverse.reshape(PhiStar.getNumRows(), PhiStar.getNumCols());
      SNsBar.reshape(nDegreesOfFreedom, nActuatedDegreesOfFreedom);
   }

   @Override
   public DenseMatrix64F getDynamicallyConsistentNullspace()
   {
      return Ns;
   }

   @Override
   public DenseMatrix64F getSNsBar()
   {
      if (!computeSNsBar)
         throw new RuntimeException("SNsBar not computed");

      return SNsBar;
   }

   private void computeJs(DenseMatrix64F Js, Map<RigidBody, List<InverseDynamicsJoint>> supportJacobians,
                          Map<RigidBody, DenseMatrix64F> constrainedBodiesAndSelectionMatrices)
   {
      Js.zero();

      int rowStartNumber = 0;
      for (RigidBody rigidBody : supportJacobians.keySet())
      {
         List<InverseDynamicsJoint> supportingJoints = supportJacobians.get(rigidBody);
         DenseMatrix64F selectionMatrix = constrainedBodiesAndSelectionMatrices.get(rigidBody);

         int columnStartNumber = 0;
         for (InverseDynamicsJoint inverseDynamicsJoint : jointsInOrder)
         {
            if (supportingJoints.contains(inverseDynamicsJoint))
            {
               for (int dofIndex = 0; dofIndex < inverseDynamicsJoint.getDegreesOfFreedom(); dofIndex++)
               {
                  inverseDynamicsJoint.getUnitTwist(dofIndex, tempTwist);
                  tempTwist.changeFrame(rootJoint.getFrameAfterJoint());
                  tempTwist.getMatrix(tempTwistMatrix, 0);
                  tempJacobianPart.reshape(selectionMatrix.getNumRows(), 1);
                  CommonOps.mult(selectionMatrix, tempTwistMatrix, tempJacobianPart);
                  CommonOps.insert(tempJacobianPart, Js, rowStartNumber, columnStartNumber++);
               }
            }
            else
            {
               columnStartNumber += inverseDynamicsJoint.getDegreesOfFreedom();
            }
         }

         rowStartNumber += selectionMatrix.getNumRows();
      }
   }
}
