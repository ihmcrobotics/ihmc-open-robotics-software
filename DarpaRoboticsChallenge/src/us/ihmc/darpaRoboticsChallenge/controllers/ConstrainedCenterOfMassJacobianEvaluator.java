package us.ihmc.darpaRoboticsChallenge.controllers;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.factory.SingularValueDecomposition;
import org.ejml.ops.CommonOps;
import org.ejml.ops.NormOps;
import org.ejml.ops.SingularOps;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.*;

import java.util.*;

/**
 * See: L. Sentis. Synthesis and Control of Whole-Body Behaviors in Humanoid Systems (2007)
 *
 * @author twan
 *         Date: 4/15/13
 */
public class ConstrainedCenterOfMassJacobianEvaluator implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ConstrainedCenterOfMassJacobianCalculator calculator;
   private final DoubleYoVariable conditionNumber = new DoubleYoVariable("suppConstJacCond", registry);
   private final DoubleYoVariable smallestSingularValue = new DoubleYoVariable("suppConstJacSmallestSingular", registry);

   public ConstrainedCenterOfMassJacobianEvaluator(FullRobotModel fullRobotModel)
   {
      Map<RigidBody, DenseMatrix64F> constrainedBodiesAndSelectionMatrices = new HashMap<RigidBody, DenseMatrix64F>();
      for (RobotSide robotSide : RobotSide.values())
      {
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         DenseMatrix64F selectionMatrix = new DenseMatrix64F(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
         CommonOps.setIdentity(selectionMatrix);
         constrainedBodiesAndSelectionMatrices.put(foot, selectionMatrix);
      }

      DenseMatrix64F orientationSelectionMatrix = new DenseMatrix64F(SpatialMotionVector.SIZE / 2, SpatialMotionVector.SIZE);
      orientationSelectionMatrix.set(0, 0, 1.0);
      orientationSelectionMatrix.set(1, 1, 1.0);
      orientationSelectionMatrix.set(2, 2, 1.0);
      constrainedBodiesAndSelectionMatrices.put(fullRobotModel.getPelvis(), orientationSelectionMatrix);

      Collection<InverseDynamicsJoint> actuatedJoints = new ArrayList<InverseDynamicsJoint>();
      for (RobotSide robotSide : RobotSide.values())
      {
//         OneDoFJoint kneeJoint = fullRobotModel.getLegJoint(robotSide, LegJointName.KNEE);
//         InverseDynamicsJoint[] kneeAndBelow = ScrewTools.createJointPath(kneeJoint.getPredecessor().getParentJoint().getPredecessor(), fullRobotModel.getFoot(robotSide));
//         actuatedJoints.addAll(Arrays.asList(kneeAndBelow));

         InverseDynamicsJoint[] jointPath = ScrewTools.createJointPath(fullRobotModel.getPelvis(), fullRobotModel.getFoot(robotSide));
         actuatedJoints.addAll(Arrays.asList(jointPath));
      }

//      actuatedJoints.addAll(Arrays.asList(ScrewTools.computeJointsInOrder(fullRobotModel.getElevator())));
//      actuatedJoints.remove(fullRobotModel.getRootJoint());

      calculator = new ConstrainedCenterOfMassJacobianCalculator(fullRobotModel.getRootJoint(), constrainedBodiesAndSelectionMatrices, actuatedJoints);
   }

   public void doControl()
   {
      calculator.compute();
      DenseMatrix64F constrainedCenterOfMassJacobian = calculator.getConstrainedCenterOfMassJacobian();

      this.conditionNumber.set(NormOps.conditionP2(constrainedCenterOfMassJacobian));
      this.smallestSingularValue.set(computeSmallestSingularValue(constrainedCenterOfMassJacobian));
   }

   public void initialize()
   {
      // empty
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return getClass().getSimpleName();
   }

   public String getDescription()
   {
      return getName();
   }

   private static double computeSmallestSingularValue(DenseMatrix64F A)
   {
      SingularValueDecomposition<DenseMatrix64F> svd = DecompositionFactory.svd(A.numRows, A.numCols, false, false, true);

      svd.decompose(A);

      double[] singularValues = svd.getSingularValues();

      int n = SingularOps.rank(svd, 1e-12);

      if (n == 0)
         return 0;

      double smallest = Double.MAX_VALUE;

      for (double s : singularValues)
      {
         if (s < smallest)
            smallest = s;
      }

      return smallest;
   }
}
