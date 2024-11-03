package us.ihmc.robotics.screwTheory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.SpecializedOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;

import us.ihmc.mecano.algorithms.CompositeRigidBodyMassMatrixCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.tools.MultiBodySystemTools;

/**
 *
 * See: L. Sentis. Synthesis and Control of Whole-Body Behaviors in Humanoid Systems (2007)
 *
 * @author twan
 *         Date: 4/15/13
 */
public class OriginalDynamicallyConsistentNullspaceCalculator implements DynamicallyConsistentNullspaceCalculator
{
   private final FloatingJointBasics rootJoint;
   private final JointBasics[] jointsInOrder;
   private final Map<RigidBodyBasics, DMatrixRMaj> constrainedBodiesAndSelectionMatrices = new LinkedHashMap<RigidBodyBasics, DMatrixRMaj>();
   private final List<JointBasics> actuatedJoints = new ArrayList<JointBasics>();
   private final Map<RigidBodyBasics, List<JointBasics>> supportingBodyToJointPathMap = new LinkedHashMap<RigidBodyBasics, List<JointBasics>>();
   private final CompositeRigidBodyMassMatrixCalculator massMatrixCalculator;

   private final boolean computeSNsBar;

   private final LinearSolverDense<DMatrixRMaj> massMatrixSolver;
   private final LinearSolverDense<DMatrixRMaj> lambdaSolver;
   private final LinearSolverDense<DMatrixRMaj> pseudoInverseSolver;
   private final DMatrixRMaj S = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj Js = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj AInverse = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj AInverseJSTranspose = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj LambdasInverse = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj Lambdas = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj JsBar = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj Ns = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj SNs = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj AInverseSNsTranspose = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj PhiStar = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj PhiStarInverse = new DMatrixRMaj(1, 1);
   private final DMatrixRMaj SNsBar = new DMatrixRMaj(1, 1);

   private final Twist tempTwist = new Twist();
   private final DMatrixRMaj tempTwistMatrix = new DMatrixRMaj(Twist.SIZE, 1);
   private final DMatrixRMaj tempJacobianPart = new DMatrixRMaj(1, 1);

   private final int nDegreesOfFreedom;
   private int nConstraints;

   public OriginalDynamicallyConsistentNullspaceCalculator(FloatingJointBasics rootJoint,
                                                           boolean computeSNsBar)
   {
      this.rootJoint = rootJoint;
      this.massMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(rootJoint.getSuccessor());
      jointsInOrder = massMatrixCalculator.getInput().getJointMatrixIndexProvider().getIndexedJointsInOrder().toArray(new JointBasics[0]);
      this.nDegreesOfFreedom = MultiBodySystemTools.computeDegreesOfFreedom(jointsInOrder);
      massMatrixSolver = LinearSolverFactory_DDRM.symmPosDef(nDegreesOfFreedom);
      lambdaSolver = LinearSolverFactory_DDRM.symmPosDef(nDegreesOfFreedom); // size of matrix is only used to choose algorithm. nDegreesOfFreedom is an upper limit
      pseudoInverseSolver = LinearSolverFactory_DDRM.pseudoInverse(true);
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
   public void addConstraint(RigidBodyBasics body, DMatrixRMaj selectionMatrix)
   {
      constrainedBodiesAndSelectionMatrices.put(body, selectionMatrix);
      nConstraints += selectionMatrix.getNumRows();
      JointBasics[] jointPath = MultiBodySystemTools.createJointPath(rootJoint.getPredecessor(), body);
      this.supportingBodyToJointPathMap.put(body, Arrays.asList(jointPath));
   }

   @Override
   public void addActuatedJoint(JointBasics joint)
   {
      actuatedJoints.add(joint);
   }

   private static void computeS(DMatrixRMaj S, JointBasics[] jointsInOrder, Collection<JointBasics> actuatedJoints)
   {
      S.zero();
      int rowStart = 0;
      int columnStart = 0;
      for (JointBasics inverseDynamicsJoint : jointsInOrder)
      {
         int degreesOfFreedom = inverseDynamicsJoint.getDegreesOfFreedom();

         if (actuatedJoints.contains(inverseDynamicsJoint))
         {
            DMatrixRMaj identity = new DMatrixRMaj(degreesOfFreedom, degreesOfFreedom);
            CommonOps_DDRM.setIdentity(identity);
            CommonOps_DDRM.insert(identity, S, rowStart, columnStart);
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

      massMatrixCalculator.reset();
      massMatrixSolver.setA(massMatrixCalculator.getMassMatrix());
      massMatrixSolver.invert(AInverse);
      CommonOps_DDRM.multTransB(AInverse, Js, AInverseJSTranspose);
      CommonOps_DDRM.mult(Js, AInverseJSTranspose, LambdasInverse);

      lambdaSolver.setA(LambdasInverse);
      lambdaSolver.invert(Lambdas);
      CommonOps_DDRM.mult(AInverseJSTranspose, Lambdas, JsBar);

//      CommonOps_DDRM.pinv(Js, JsBar);

      CommonOps_DDRM.mult(JsBar, Js, Ns);
      CommonOps_DDRM.changeSign(Ns);
      SpecializedOps_DDRM.addIdentity(Ns, Ns, 1.0);

      if (computeSNsBar)
      {
         CommonOps_DDRM.mult(S, Ns, SNs);
         CommonOps_DDRM.multTransB(AInverse, SNs, AInverseSNsTranspose);
         CommonOps_DDRM.mult(SNs, AInverseSNsTranspose, PhiStar);
         pseudoInverseSolver.setA(PhiStar);
         pseudoInverseSolver.invert(PhiStarInverse);
         CommonOps_DDRM.mult(AInverseSNsTranspose, PhiStarInverse, SNsBar);

//         CommonOps_DDRM.mult(S, Ns, SNs);
//         ConfigurableSolvePseudoInverseSVD solver = new ConfigurableSolvePseudoInverseSVD(SNs.getNumRows(), SNs.getNumCols(), 0.5);
//         solver.setA(SNs);
//         solver.invert(SNsBar);
      }
   }

   private void resizeMatrices()
   {
      int nActuatedDegreesOfFreedom = MultiBodySystemTools.computeDegreesOfFreedom(actuatedJoints);

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
   public DMatrixRMaj getDynamicallyConsistentNullspace()
   {
      return Ns;
   }

   @Override
   public DMatrixRMaj getSNsBar()
   {
      if (!computeSNsBar)
         throw new RuntimeException("SNsBar not computed");

      return SNsBar;
   }

   private void computeJs(DMatrixRMaj Js, Map<RigidBodyBasics, List<JointBasics>> supportJacobians,
                          Map<RigidBodyBasics, DMatrixRMaj> constrainedBodiesAndSelectionMatrices)
   {
      Js.zero();

      int rowStartNumber = 0;
      for (RigidBodyBasics rigidBody : supportJacobians.keySet())
      {
         List<JointBasics> supportingJoints = supportJacobians.get(rigidBody);
         DMatrixRMaj selectionMatrix = constrainedBodiesAndSelectionMatrices.get(rigidBody);

         int columnStartNumber = 0;
         for (JointBasics inverseDynamicsJoint : jointsInOrder)
         {
            if (supportingJoints.contains(inverseDynamicsJoint))
            {
               for (int dofIndex = 0; dofIndex < inverseDynamicsJoint.getDegreesOfFreedom(); dofIndex++)
               {
                  tempTwist.setIncludingFrame(inverseDynamicsJoint.getUnitTwists().get(dofIndex));
                  tempTwist.changeFrame(rootJoint.getFrameAfterJoint());
                  tempTwist.get(0, tempTwistMatrix);
                  tempJacobianPart.reshape(selectionMatrix.getNumRows(), 1);
                  CommonOps_DDRM.mult(selectionMatrix, tempTwistMatrix, tempJacobianPart);
                  CommonOps_DDRM.insert(tempJacobianPart, Js, rowStartNumber, columnStartNumber++);
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
