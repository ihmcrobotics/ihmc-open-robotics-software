package us.ihmc.commonWalkingControlModules.contact;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.algorithms.GeometricJacobianCalculator;
import us.ihmc.mecano.algorithms.InverseDynamicsCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.interfaces.FixedFrameSpatialVectorBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.functionApproximation.DampedLeastSquaresSolver;
import us.ihmc.robotics.math.filters.AlphaFilteredYoSpatialVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;

public class HandWrenchCalculator
{
   public static final double BREAK_FREQUENCY = 20.0;

   private final YoRegistry registry;
   private final GeometricJacobianCalculator geometricJacobianCalculator = new GeometricJacobianCalculator();
   private final List<JointReadOnly> jointsFromBaseToEndEffector;
   private final List<OneDoFJointBasics> armJoints;
   private final double[] jointTorquesForGravity;
   private final double[] jointTorques;
   private final InverseDynamicsCalculator inverseDynamicsCalculator;
   private final SpatialVector unfilteredWrench = new SpatialVector();
   private final AlphaFilteredYoSpatialVector yoFilteredWrench;
   private final DMatrixRMaj wrenchVector = new DMatrixRMaj(6, 1);
   private final DMatrixRMaj jointTorqueVector;
   private final DMatrixRMaj armJacobianTranspose;
   private final DMatrixRMaj armJacobianTransposeDagger;
   private final DampedLeastSquaresSolver dampedPseudoInverseSolver;

   public HandWrenchCalculator(RobotSide side, FullHumanoidRobotModel fullRobotModel, YoRegistry parentRegistry, double expectedComputeDT)
   {
      registry = new YoRegistry(HandWrenchCalculator.class.getSimpleName() + side.getPascalCaseName());

      geometricJacobianCalculator.setKinematicChain(fullRobotModel.getChest(), fullRobotModel.getHand(side));
      geometricJacobianCalculator.setJacobianFrame(fullRobotModel.getHandControlFrame(side));

      jointsFromBaseToEndEffector = geometricJacobianCalculator.getJointsFromBaseToEndEffector();
      armJoints = MultiBodySystemTools.filterJoints(jointsFromBaseToEndEffector, OneDoFJointBasics.class);
      jointTorquesForGravity = new double[armJoints.size()];
      jointTorques = new double[armJoints.size()];
      jointTorqueVector = new DMatrixRMaj(jointTorques.length, 1);
      jointTorqueVector.setData(jointTorques);

      armJacobianTranspose = CommonOps_DDRM.transpose(geometricJacobianCalculator.getJacobianMatrix(), null);
      armJacobianTransposeDagger = new DMatrixRMaj(armJacobianTranspose);
      double dampingFactor = 1e-6;
      dampedPseudoInverseSolver = new DampedLeastSquaresSolver(armJacobianTranspose.getNumRows(), dampingFactor);

      inverseDynamicsCalculator = new InverseDynamicsCalculator(MultiBodySystemReadOnly.toMultiBodySystemInput(armJoints));
      inverseDynamicsCalculator.setConsiderCoriolisAndCentrifugalForces(false);
      inverseDynamicsCalculator.setGravitionalAcceleration(-9.81);

      double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(BREAK_FREQUENCY, expectedComputeDT);
      yoFilteredWrench = new AlphaFilteredYoSpatialVector("filteredWrench",
                                                          side.toString(),
                                                          registry,
                                                          () -> alpha,
                                                          () -> alpha,
                                                          unfilteredWrench.getAngularPart(),
                                                          unfilteredWrench.getLinearPart());
      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   public void compute()
   {
      // Compute gravity compensation torques
      // TODO: Check if this looks good.
      //   Remove gravity compensation from the wrench when called? but gravity compensation would change as the arm moves...
      inverseDynamicsCalculator.compute();
      for (int i = 0; i < armJoints.size(); ++i)
      {
         DMatrixRMaj tau = inverseDynamicsCalculator.getComputedJointTau(armJoints.get(i));
         if (tau != null)
         {
            jointTorquesForGravity[i] = tau.get(0, 0);
         }
      }

      for (int i = 0; i < armJoints.size(); ++i)
      {
         jointTorques[i] = armJoints.get(i).getTau() - jointTorquesForGravity[i];
      }

      // Need to call rest to update the joint configurations in the jacobian calculator.
      geometricJacobianCalculator.reset();
      // getJacobianMatrix also updates the matrix
      CommonOps_DDRM.transpose(geometricJacobianCalculator.getJacobianMatrix(), armJacobianTranspose);

      armJacobianTransposeDagger.set(armJacobianTranspose);
      dampedPseudoInverseSolver.setA(armJacobianTranspose);
      dampedPseudoInverseSolver.invert(armJacobianTransposeDagger);

      CommonOps_DDRM.mult(armJacobianTransposeDagger, jointTorqueVector, wrenchVector);

      unfilteredWrench.setReferenceFrame(geometricJacobianCalculator.getJacobianFrame());
      unfilteredWrench.set(wrenchVector);
      unfilteredWrench.changeFrame(ReferenceFrame.getWorldFrame());
      // Negate to express the wrench the hand experiences from the external world
      unfilteredWrench.negate();

      yoFilteredWrench.update();
   }

   public SpatialVector getUnfilteredWrench()
   {
      return unfilteredWrench;
   }

   public AlphaFilteredYoSpatialVector getFilteredWrench()
   {
      return yoFilteredWrench;
   }

   public double getLinearWrenchMagnitude(boolean filtered)
   {
      FixedFrameSpatialVectorBasics wrench = filtered ? getFilteredWrench() : getUnfilteredWrench();
      return wrench.getLinearPart().norm();
   }

   public double getAngularWrenchMagnitude(boolean filtered)
   {
      FixedFrameSpatialVectorBasics wrench = filtered ? getFilteredWrench() : getUnfilteredWrench();
      return wrench.getAngularPart().norm();
   }
}