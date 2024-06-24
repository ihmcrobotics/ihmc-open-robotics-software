package us.ihmc.commonWalkingControlModules.controlModules.multiContact;

import gnu.trove.map.hash.TObjectIntHashMap;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.HeuristicCoMMarginPostureCalculator;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.CenterOfMassStabilityMarginRegionCalculator;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.CoMMarginSensitivityCalculator;
import us.ihmc.commonWalkingControlModules.staticEquilibrium.WholeBodyContactState;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.algorithms.CentroidalMomentumCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.linearAlgebra.DampedNullspaceCalculator;
import us.ihmc.robotics.linearAlgebra.DampedSVDNullspaceCalculator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class MultiContactPostureAdjustmentCalculator implements WholeBodyPostureAdjustmentProvider
{
   private static final int LINEAR_DIMENSIONS = 3;
   private static final int SPATIAL_DIMENSIONS = 6;
   private static final boolean DEFAULT_ENABLED = false;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final FullHumanoidRobotModel fullRobotModel;
   private final TObjectIntHashMap<OneDoFJointBasics> jointToIndexMap = new TObjectIntHashMap<>(30);
   private final HeuristicCoMMarginPostureCalculator postureOptimizer;
   private final WholeBodyContactState contactState;

   private final DMatrixRMaj qdTProjected = new DMatrixRMaj(0);
   private final DMatrixRMaj qdSystem = new DMatrixRMaj(0);

   private final CentroidalMomentumCalculator centroidalMomentumCalculator;

   private final YoBoolean isEnabled = new YoBoolean("isEnabled", registry);
   private final YoDouble nullspaceProjectionAlpha = new YoDouble("nullspaceProjectionAlpha", registry);

   private final DMatrixRMaj stackedContactJacobian = new DMatrixRMaj(0);
   private final DampedNullspaceCalculator nullspaceCalculator = new DampedSVDNullspaceCalculator(50, 0.1);

   private final YoDouble[] qJointPostureAdjustment;
   private final YoDouble[] qdJointPostureAdjustment;
   private final YoDouble qPelvisHeightPostureAdjustment = new YoDouble("q_post_adj_pelvisHeight", registry);
   private final YoDouble qdPelvisHeightPostureAdjustment = new YoDouble("qd_post_adj_pelvisHeight", registry);
   private final YoFrameQuaternion pelvisOrientationPostureAdjustment = new YoFrameQuaternion("q_post_adj_pelvisOrientation", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D pelvisAngularVelocityPostureAdjustment = new YoFrameVector3D("qd_post_adj_pelvisOrientation", ReferenceFrame.getWorldFrame(), registry);

   private final double controlDT;
   private final YoDouble qdMultiplier = new YoDouble("qdMultiplier", registry);

   private final Vector3D pelvisRotationVectorAdjustment = new Vector3D();
   private final Quaternion pelvisRotationQuaternionAdjustment = new Quaternion();

   private final CoMMarginSensitivityCalculator optimizerLookAhead;
   private final ExecutionTimer comUpdatePreviewTimer = new ExecutionTimer("comUpdatePreviewTimer", registry);
   private final ExecutionTimer nullspaceProjectionTimer = new ExecutionTimer("nullspaceProjectionTimer", registry);
   private final ExecutionTimer postureOptimizationTimer = new ExecutionTimer("postureOptimizationTimer", registry);

   public MultiContactPostureAdjustmentCalculator(CenterOfMassStabilityMarginRegionCalculator staticStabilityRegionCalculator,
                                                  WholeBodyContactState contactState,
                                                  FullHumanoidRobotModel fullRobotModel,
                                                  JointBasics[] controlledJoints,
                                                  ReferenceFrame centerOfMassFrame,
                                                  double controlDT,
                                                  YoRegistry parentRegistry)
   {
      this.fullRobotModel = fullRobotModel;
      this.controlDT = controlDT;
      this.contactState = contactState;

      this.postureOptimizer = new HeuristicCoMMarginPostureCalculator(staticStabilityRegionCalculator,
                                                                      contactState,
                                                                      controlDT,
                                                                      parentRegistry);
      this.optimizerLookAhead = new CoMMarginSensitivityCalculator("", staticStabilityRegionCalculator, fullRobotModel, contactState, registry);

      isEnabled.set(DEFAULT_ENABLED);

      int numConstraints = 0;
      numConstraints += 2; // xy centroidal motion
      numConstraints += 2 * LINEAR_DIMENSIONS; // hand linear position
      numConstraints += 2 * SPATIAL_DIMENSIONS; // foot spatial pose
      stackedContactJacobian.reshape(numConstraints, SPATIAL_DIMENSIONS + contactState.getNumberOfJoints());

      qdTProjected.reshape(1, SPATIAL_DIMENSIONS + contactState.getNumberOfJoints());
      qdSystem.reshape(SPATIAL_DIMENSIONS + contactState.getNumberOfJoints(), 1);
      nullspaceProjectionAlpha.set(0.01);

      qJointPostureAdjustment = new YoDouble[contactState.getNumberOfJoints()];
      qdJointPostureAdjustment = new YoDouble[contactState.getNumberOfJoints()];

      for (int jointIdx = 0; jointIdx < contactState.getNumberOfJoints(); jointIdx++)
      {
         OneDoFJointBasics joint = contactState.getOneDoFJoints()[jointIdx];
         jointToIndexMap.put(joint, jointIdx);
         qJointPostureAdjustment[jointIdx] = new YoDouble("q_comMargin_" + joint.getName(), registry);
         qdJointPostureAdjustment[jointIdx] = new YoDouble("qd_comMargin_Proj_" + joint.getName(), registry);
      }

      qdMultiplier.set(0.01);

      MultiBodySystemBasics multiBodySystemInput = MultiBodySystemBasics.toMultiBodySystemBasics(controlledJoints);
      centroidalMomentumCalculator = new CentroidalMomentumCalculator(multiBodySystemInput, centerOfMassFrame);

      parentRegistry.addChild(registry);
   }

   @Override
   public void update()
   {
      postureOptimizationTimer.startMeasurement();
      postureOptimizer.computePostureAdjustment();

      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////////// COMPUTE CONSTRAINT JACOBIAN //////////////////////////////////////////
      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

      if (postureOptimizer.hasTorqueSaturatedConstraints())
      {
         int rowOffset = 0;

         centroidalMomentumCalculator.reset();
         DMatrixRMaj centroidalMomentumMatrix = centroidalMomentumCalculator.getCentroidalMomentumMatrix();
         int centroidalMomentumRowOffset = 3;
         int centroidalMomentumNumRows = 2;
         MatrixTools.setMatrixBlock(stackedContactJacobian, rowOffset, 0, centroidalMomentumMatrix, centroidalMomentumRowOffset, 0, centroidalMomentumNumRows, centroidalMomentumMatrix.getNumCols(), 1.0);
         rowOffset += centroidalMomentumNumRows;

         for (RobotSide robotSide : RobotSide.values)
         {
            rowOffset = stackJacobian(rowOffset, contactState.getJacobian(fullRobotModel.getFoot(robotSide)), false);
            rowOffset = stackJacobian(rowOffset, contactState.getJacobian(fullRobotModel.getHand(robotSide)), true);
         }

         qdTProjected.zero();
         for (int jointIdx = 0; jointIdx < contactState.getNumberOfJoints(); jointIdx++)
         {
            double qd = postureOptimizer.getOptimizedPostureAdjustmentVelocity(jointIdx);
            qdTProjected.set(0, SPATIAL_DIMENSIONS + jointIdx, qd);
         }

         nullspaceProjectionTimer.startMeasurement();
         nullspaceCalculator.setPseudoInverseAlpha(nullspaceProjectionAlpha.getValue());
         nullspaceCalculator.projectOntoNullspace(qdTProjected, stackedContactJacobian);
         nullspaceProjectionTimer.stopMeasurement();

         CommonOps_DDRM.transpose(qdTProjected, qdSystem);

         comUpdatePreviewTimer.startMeasurement();
         optimizerLookAhead.compute(false, false, qdSystem, 3.0e-3, postureOptimizer.getLowestMarginVertexIndices());
         comUpdatePreviewTimer.stopMeasurement();
      }
      else
      {
         setVelocitiesToZero();
         return;
      }

      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
      ////////////////////////////////////// INTEGRATE AND DECAY VELOCITIES  ////////////////////////////////////////
      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

      updateProjectedVelocities();
      doIntegration();

      postureOptimizationTimer.stopMeasurement();
   }

   private void setVelocitiesToZero()
   {
      qdPelvisHeightPostureAdjustment.set(0.0);
      pelvisAngularVelocityPostureAdjustment.setToZero();
   }

   private void updateProjectedVelocities()
   {
      qdPelvisHeightPostureAdjustment.set(qdMultiplier.getValue() * qdTProjected.get(0, 5));

      pelvisAngularVelocityPostureAdjustment.set(qdTProjected.get(0, 0), qdTProjected.get(0, 1), qdTProjected.get(0, 2));
      pelvisAngularVelocityPostureAdjustment.scale(qdMultiplier.getValue());

      for (int jointIdx = 0; jointIdx < contactState.getNumberOfJoints(); jointIdx++)
      {
         qdJointPostureAdjustment[jointIdx].set(qdMultiplier.getValue() * qdTProjected.get(0, SPATIAL_DIMENSIONS + jointIdx));
      }
   }

   private void doIntegration()
   {
      qdPelvisHeightPostureAdjustment.add(controlDT * qdPelvisHeightPostureAdjustment.getValue());

      pelvisRotationVectorAdjustment.setAndScale(controlDT, pelvisAngularVelocityPostureAdjustment);
      pelvisRotationQuaternionAdjustment.setRotationVector(pelvisRotationVectorAdjustment);
      pelvisOrientationPostureAdjustment.append(pelvisRotationQuaternionAdjustment);

      for (int jointIdx = 0; jointIdx < contactState.getNumberOfJoints(); jointIdx++)
      {
         qJointPostureAdjustment[jointIdx].add(controlDT * qdJointPostureAdjustment[jointIdx].getValue());
      }
   }

   private int stackJacobian(int rowOffset, GeometricJacobian jacobian, boolean linearOnly)
   {
      JointBasics[] joints = jacobian.getJointsInOrder();
      DMatrixRMaj jacobianMatrix = jacobian.getJacobianMatrix();
      int srcStartRow = linearOnly ? 3 : 0;
      int srcNumRows = linearOnly ? 3 : 6;

      // floating root joint is zero index in all cases
      MatrixTools.setMatrixBlock(stackedContactJacobian, rowOffset, 0, jacobianMatrix, srcStartRow, 0, srcNumRows, SPATIAL_DIMENSIONS, 1.0);

      // copy joint entries using the index map
      for (int jointIdx = 1; jointIdx < joints.length; jointIdx++)
      {
         OneDoFJointBasics joint = (OneDoFJointBasics) joints[jointIdx];
         int systemColumnIndex = getSystemJacobianColumn(joint);
         int jacobianColumn = SPATIAL_DIMENSIONS + jointIdx - 1;

         for (int rowIdx = 0; rowIdx < srcNumRows; rowIdx++)
         {
            double jacobianEntry = jacobianMatrix.get(srcStartRow + rowIdx, jacobianColumn);
            stackedContactJacobian.set(rowOffset + rowIdx, systemColumnIndex, jacobianEntry);
         }
      }

      return rowOffset + srcNumRows;
   }

   private int getSystemJacobianColumn(OneDoFJointBasics joint)
   {
      return SPATIAL_DIMENSIONS + jointToIndexMap.get(joint);
   }

   @Override
   public boolean isEnabled()
   {
      return isEnabled.getValue();
   }

   @Override
   public double getDesiredJointPositionOffset(OneDoFJointBasics joint)
   {
      if (isEnabled.getValue())
      {
         return qJointPostureAdjustment[jointToIndexMap.get(joint)].getValue();
      }
      else
      {
         return 0.0;
      }
   }

   @Override
   public double getDesiredJointVelocityOffset(OneDoFJointBasics joint)
   {
      if (isEnabled.getValue())
      {
         return qdJointPostureAdjustment[jointToIndexMap.get(joint)].getValue();
      }
      else
      {
         return 0.0;
      }
   }

   @Override
   public double getDesiredJointAccelerationOffset(OneDoFJointBasics joint)
   {
      return 0.0;
   }

   @Override
   public double getFloatingBasePositionOffsetZ()
   {
      if (isEnabled.getValue())
      {
         return qPelvisHeightPostureAdjustment.getValue();
      }
      else
      {
         return 0.0;
      }
   }

   @Override
   public double getFloatingBaseVelocityOffsetZ()
   {
      if (isEnabled.getValue())
      {
         return qdPelvisHeightPostureAdjustment.getValue();
      }
      else
      {
         return 0.0;
      }
   }

   @Override
   public double getFloatingBaseAccelerationOffsetZ()
   {
      return 0.0;
   }

   @Override
   public void packFloatingBaseOrientationOffset(FrameQuaternionBasics orientationOffsetToPack)
   {
      if (isEnabled.getValue())
      {
         orientationOffsetToPack.set(pelvisOrientationPostureAdjustment);
      }
      else
      {
         orientationOffsetToPack.setToZero();
      }
   }

   @Override
   public void packFloatingBaseAngularVelocityOffset(FrameVector3DBasics angularVelocityToPack)
   {
      if (isEnabled.getValue())
      {
         angularVelocityToPack.set(pelvisAngularVelocityPostureAdjustment);
      }
      else
      {
         angularVelocityToPack.setToZero();
      }
   }

   @Override
   public void packFloatingBaseAngularAccelerationOffset(FrameVector3DBasics angularAccelerationToPack)
   {
      angularAccelerationToPack.setToZero();
   }
}
