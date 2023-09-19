package us.ihmc.behaviors.tools;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commons.thread.Notification;
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
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;

public class HandWrenchCalculator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final SideDependentList<GeometricJacobianCalculator> jacobianCalculators = new SideDependentList<>();
   private final SideDependentList<List<JointReadOnly>> jointsFromBaseToEndEffector = new SideDependentList<>();
   private final SideDependentList<List<OneDoFJointBasics>> armJoints = new SideDependentList<>();
   private final SideDependentList<SpatialVector> rawWrenches = new SideDependentList<>(new SpatialVector(), new SpatialVector());
   private final SideDependentList<InverseDynamicsCalculator> inverseDynamicsCalculators = new SideDependentList<>();
   private final SideDependentList<double[]> jointTorquesForGravity = new SideDependentList<>();
   private final SideDependentList<double[]> jointTorques = new SideDependentList<>();
   private final SideDependentList<AlphaFilteredYoSpatialVector> alphaFilteredYoSpatialVectors = new SideDependentList<>();
   // RobotConfigurationData is published at 120Hz -> break freq.: 5Hz - 20Hz
   private final Notification receivedRCDNotification;

   public HandWrenchCalculator(FullHumanoidRobotModel fullRobotModel)
   {
      this(fullRobotModel, null);
   }

   public HandWrenchCalculator(ROS2SyncedRobotModel syncedRobot)
   {
      this(syncedRobot.getFullRobotModel(), new Notification());

      syncedRobot.addRobotConfigurationDataReceivedCallback(receivedRCDNotification::set);
   }

   private HandWrenchCalculator(FullHumanoidRobotModel fullRobotModel, Notification receivedRCDNotification)
   {
      this.receivedRCDNotification = receivedRCDNotification;

      for (RobotSide side : RobotSide.values) // Set up for each side
      {
         GeometricJacobianCalculator geometricJacobianCalculator = new GeometricJacobianCalculator();
         geometricJacobianCalculator.setKinematicChain(fullRobotModel.getChest(), fullRobotModel.getHand(side));
         geometricJacobianCalculator.setJacobianFrame(fullRobotModel.getHandControlFrame(side));
         // geometricJacobianCalculator.setJacobianFrame(ReferenceFrame.getWorldFrame());
         jacobianCalculators.set(side, geometricJacobianCalculator);
         jointsFromBaseToEndEffector.set(side, geometricJacobianCalculator.getJointsFromBaseToEndEffector());
         List<OneDoFJointBasics> oneDoFJoints = MultiBodySystemTools.filterJoints(jointsFromBaseToEndEffector.get(side), OneDoFJointBasics.class);
         armJoints.set(side, oneDoFJoints);
         MultiBodySystemReadOnly system = MultiBodySystemReadOnly.toMultiBodySystemInput(armJoints.get(side));
         InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(system);
         inverseDynamicsCalculator.setConsiderCoriolisAndCentrifugalForces(false);
         inverseDynamicsCalculator.setGravitionalAcceleration(-9.81);
         inverseDynamicsCalculators.set(side, inverseDynamicsCalculator);
         jointTorquesForGravity.set(side, new double[armJoints.get(side).size()]);

         SpatialVector spatialVectorForSetup = new SpatialVector();
         double breakFrequency = 20;
         double dt = 1.0 / 120.0;
         double alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(breakFrequency, dt);
         alphaFilteredYoSpatialVectors.set(side,
                                           new AlphaFilteredYoSpatialVector("filteredWrench",
                                                                            side.toString(),
                                                                            registry,
                                                                            () -> alpha,
                                                                            () -> alpha,
                                                                            spatialVectorForSetup.getAngularPart(),
                                                                            spatialVectorForSetup.getLinearPart()));
         jointTorques.set(side, new double[armJoints.get(side).size()]);
      }
   }

   private DMatrixRMaj leftPseudoInverse(DMatrixRMaj matrix)
   {
      DampedLeastSquaresSolver pseudoInverseSolver = new DampedLeastSquaresSolver(matrix.getNumRows(), 1e-6);
      pseudoInverseSolver.setA(matrix);
      DMatrixRMaj leftPseudoInverseOfMatrix = new DMatrixRMaj(matrix);
      pseudoInverseSolver.invert(leftPseudoInverseOfMatrix);
      return leftPseudoInverseOfMatrix;
   }

   public void compute()
   {
      // TODO: only compute / update when new RCD received. (try to match the frequency or maybe update at lower frequency)
      //  Check this

      if (receivedRCDNotification == null || receivedRCDNotification.poll())
      {
         for (RobotSide side : RobotSide.values)
         {
            double[] jointTorquesForGravity = getGravityCompensationTorques(side);
            List<OneDoFJointBasics> oneSideArmJoints = armJoints.get(side);

            for (int i = 0; i < oneSideArmJoints.size(); ++i)
            {
               jointTorques.get(side)[i] = oneSideArmJoints.get(i).getTau() - jointTorquesForGravity[i];
            }

            // Need to call rest to update the joint configurations in the jacobian calculator.
            jacobianCalculators.get(side).reset();
            // getJacobianMatrix updates the matrix and outputs in the form of DMatrixRMaj
            DMatrixRMaj armJacobian = jacobianCalculators.get(side).getJacobianMatrix();
            DMatrixRMaj armJacobianTransposed = CommonOps_DDRM.transpose(armJacobian, null);
            DMatrixRMaj armJacobianTransposedDagger = leftPseudoInverse(armJacobianTransposed);
            DMatrixRMaj jointTorqueVector = new DMatrixRMaj(jointTorques.get(side));
            DMatrixRMaj wrenchVector = new DMatrixRMaj(6, 1);
            CommonOps_DDRM.mult(armJacobianTransposedDagger, jointTorqueVector, wrenchVector);

            rawWrenches.set(side, makeWrench(jacobianCalculators.get(side).getJacobianFrame(), wrenchVector));
            alphaFilteredYoSpatialVectors.get(side).update(rawWrenches.get(side).getAngularPart(), rawWrenches.get(side).getLinearPart());
         }
      }
   }

   public SideDependentList<double[]> getJointTorques()
   {
      return jointTorques;
   }

   // Wrench expressed in world-aligned frame
   private static SpatialVector makeWrench(ReferenceFrame jacobianFrame, DMatrixRMaj wrenchVector)
   {
      // Linear and angular part into spatial vector
      SpatialVector spatialVector = new SpatialVector();
      spatialVector.setReferenceFrame(jacobianFrame);
      spatialVector.set(wrenchVector);
      spatialVector.changeFrame(ReferenceFrame.getWorldFrame());
      // Negate to express the wrench the hand experiences from the external world
      spatialVector.negate();

      return spatialVector;
   }

   public SideDependentList<SpatialVector> getUnfilteredWrench()
   {
      return rawWrenches;
   }

   // TODO: Check if this looks good.
   //   Remove gravity compensation from the wrench when called? but gravity compensation would change as the arm moves...
   public double[] getGravityCompensationTorques(RobotSide side)
   {
      inverseDynamicsCalculators.get(side).compute();
      for (int i = 0; i < armJoints.get(side).size(); ++i)
      {
         DMatrixRMaj tau = inverseDynamicsCalculators.get(side).getComputedJointTau(armJoints.get(side).get(i));
         if (tau != null)
         {
            jointTorquesForGravity.get(side)[i] = tau.get(0, 0);
         }
      }
      return jointTorquesForGravity.get(side);
   }

   public SideDependentList<AlphaFilteredYoSpatialVector> getFilteredWrench()
   {
      return alphaFilteredYoSpatialVectors;
   }

   public SideDependentList<List<OneDoFJointBasics>> getArmJoints()
   {
      return armJoints;
   }

   public double getLinearWrenchMagnitude(RobotSide side, boolean filtered)
   {
      SideDependentList<? extends FixedFrameSpatialVectorBasics> wrench = filtered ? getFilteredWrench() : getUnfilteredWrench();
      return wrench.get(side).getLinearPart().norm();
   }

   public double getAngularWrenchMagnitude(RobotSide side, boolean filtered)
   {
      SideDependentList<? extends FixedFrameSpatialVectorBasics> wrench = filtered ? getFilteredWrench() : getUnfilteredWrench();
      return wrench.get(side).getAngularPart().norm();
   }
}