package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import gnu.trove.map.hash.TObjectIntHashMap;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.math.linearAlgebra.QRNullspaceCalculator;
import us.ihmc.math.linearAlgebra.SVDNullspaceCalculator;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.algorithms.CentroidalMomentumCalculator;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.PointJacobian;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.ArrayList;
import java.util.List;

/**
 * Computes the contact nullspace Jv = 0, where J = {J_0...J_n}^T and J_i is the Jacobian of contact point i.
 * Assumes planar foot and point hand contacts.
 */
public class ContactNullspaceCalculator
{
   private static final boolean USE_SVD = false;
   private static final int SPATIAL_DIMENSIONS = 6;

   private final OneDoFJointBasics[] controlledOneDoFJoints;

   private final DMatrixRMaj stackedContactJacobian = new DMatrixRMaj(0);
   private final CentroidalMomentumCalculator centroidalMomentumCalculator;
   private final SVDNullspaceCalculator svdNullspaceCalculator = new SVDNullspaceCalculator(40, false);
   private final QRNullspaceCalculator qrNullspaceCalculator = new QRNullspaceCalculator(40);
   private final TObjectIntHashMap<OneDoFJointBasics> jointToIndexMap = new TObjectIntHashMap<>(30);

   private final WholeBodyContactState wholeBodyContactState;

   private final SideDependentList<RigidBodyBasics> hands = new SideDependentList<>();
   private final SideDependentList<RigidBodyBasics> feet = new SideDependentList<>();
   private final SideDependentList<YoBoolean> handInContact = new SideDependentList<>();
   private final SideDependentList<YoBoolean> footInContact = new SideDependentList<>();

   private final PointJacobian pointJacobian = new PointJacobian();
   private final SideDependentList<FramePoint3DReadOnly> handControlPoints = new SideDependentList<>();

   private final List<OneDoFJointBasics> jointsToIgnore = new ArrayList<>();

   public ContactNullspaceCalculator(FullHumanoidRobotModel fullRobotModel,
                                     WholeBodyContactState wholeBodyContactState,
                                     YoRegistry registry)
   {
      JointBasics[] controlledJoints = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullRobotModel);
      this.controlledOneDoFJoints = MultiBodySystemTools.filterJoints(controlledJoints, OneDoFJointBasics.class);
      this.wholeBodyContactState = wholeBodyContactState;

      MultiBodySystemBasics multiBodySystemInput = MultiBodySystemBasics.toMultiBodySystemBasics(controlledJoints);
      centroidalMomentumCalculator = new CentroidalMomentumCalculator(multiBodySystemInput, ReferenceFrame.getWorldFrame());

      for (int jointIdx = 0; jointIdx < controlledOneDoFJoints.length; jointIdx++)
      {
         OneDoFJointBasics joint = controlledOneDoFJoints[jointIdx];
         jointToIndexMap.put(joint, jointIdx);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         hands.put(robotSide, fullRobotModel.getHand(robotSide));
         feet.put(robotSide, fullRobotModel.getFoot(robotSide));

         handInContact.put(robotSide, new YoBoolean(robotSide + "HandInContact", registry));
         footInContact.put(robotSide, new YoBoolean(robotSide + "FootInContact", registry));

         handControlPoints.put(robotSide, new FramePoint3D(fullRobotModel.getHandControlFrame(robotSide)));
      }

      OneDoFJointBasics spinePitch = fullRobotModel.getSpineJoint(SpineJointName.SPINE_PITCH);
      OneDoFJointBasics spineRoll = fullRobotModel.getSpineJoint(SpineJointName.SPINE_ROLL);

      if (spinePitch != null)
         jointsToIgnore.add(spinePitch);
      if (spineRoll != null)
         jointsToIgnore.add(spineRoll);
   }

   public void compute()
   {
      /* Setup matrix based on constraint dimensionality */
      int numConstraints = 0;
      numConstraints += 2; // xy centroidal motion
      numConstraints += jointsToIgnore.size(); // hard-coded spine joints

      for (RobotSide robotSide : RobotSide.values)
      {
         handInContact.get(robotSide).set(wholeBodyContactState.isBodyInContact(hands.get(robotSide)));
         footInContact.get(robotSide).set(wholeBodyContactState.isBodyInContact(feet.get(robotSide)));

         if (handInContact.get(robotSide).getValue())
            numConstraints += 3;
         if (footInContact.get(robotSide).getValue())
            numConstraints += 6;
      }

      stackedContactJacobian.reshape(numConstraints, SPATIAL_DIMENSIONS + controlledOneDoFJoints.length);
      stackedContactJacobian.zero();

      int rowOffset = 0;

      /* Compute and set centroidal mass momentum matrix */
      centroidalMomentumCalculator.reset();
      DMatrixRMaj centroidalMomentumMatrix = centroidalMomentumCalculator.getCentroidalMomentumMatrix();

      int centroidalMomentumRowOffset = 3;
      int centroidalMomentumNumRows = 2;

      MatrixTools.setMatrixBlock(stackedContactJacobian, rowOffset, 0, centroidalMomentumMatrix, centroidalMomentumRowOffset, 0, centroidalMomentumNumRows, centroidalMomentumMatrix.getNumCols(), 1.0);
      rowOffset += centroidalMomentumNumRows;

      for (int i = 0; i < jointsToIgnore.size(); i++)
      {
         stackedContactJacobian.set(rowOffset, getSystemJacobianColumn(jointsToIgnore.get(i)), 1.0);
         rowOffset++;
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         if (footInContact.get(robotSide).getValue())
         {
            /* Compute and set foot jacobian */
            GeometricJacobian footJacobian = wholeBodyContactState.getJacobian(feet.get(robotSide));
            rowOffset = stackJacobian(rowOffset, footJacobian.getJacobianMatrix(), footJacobian.getJointsInOrder());
         }
         if (handInContact.get(robotSide).getValue())
         {
            GeometricJacobian handJacobian = wholeBodyContactState.getJacobian(hands.get(robotSide));
            handJacobian.changeFrame(handJacobian.getBaseFrame());
            handJacobian.compute();
            pointJacobian.set(handJacobian, handControlPoints.get(robotSide));
            pointJacobian.compute();
            rowOffset = stackJacobian(rowOffset, pointJacobian.getJacobianMatrix(), handJacobian.getJointsInOrder());
         }
      }

      if (USE_SVD)
      {
         svdNullspaceCalculator.setMatrix(stackedContactJacobian, stackedContactJacobian.getNumCols() - stackedContactJacobian.getNumRows());
      }
      else
      {
         qrNullspaceCalculator.computeNullspaceProjector(stackedContactJacobian, null);
      }
   }

   public CentroidalMomentumCalculator getCentroidalMomentumCalculator()
   {
      return centroidalMomentumCalculator;
   }

   public DMatrixRMaj getNullspace()
   {
      if (USE_SVD)
      {
         return svdNullspaceCalculator.getNullspace();
      }
      else
      {
         return qrNullspaceCalculator.getNullspace();
      }
   }

   public void project(DMatrixRMaj velocityToProject, DMatrixRMaj projectedVelocity)
   {
      if (USE_SVD)
      {
         svdNullspaceCalculator.projectOntoNullspace(velocityToProject, stackedContactJacobian, projectedVelocity);
      }
      else
      {
         qrNullspaceCalculator.projectOntoNullspace(velocityToProject, stackedContactJacobian, projectedVelocity);
      }
   }

   private int stackJacobian(int rowOffset, DMatrixRMaj jacobianMatrix, JointBasics[] joints)
   {
      int numRows = jacobianMatrix.getNumRows();

      // floating root joint is zero index in all cases
      MatrixTools.setMatrixBlock(stackedContactJacobian, rowOffset, 0, jacobianMatrix, 0, 0, numRows, SPATIAL_DIMENSIONS, 1.0);

      // copy joint entries using the index map
      for (int jointIdx = 1; jointIdx < joints.length; jointIdx++)
      {
         OneDoFJointBasics joint = (OneDoFJointBasics) joints[jointIdx];
         int systemColumnIndex = getSystemJacobianColumn(joint);
         int jacobianColumn = SPATIAL_DIMENSIONS + jointIdx - 1;

         for (int rowIdx = 0; rowIdx < numRows; rowIdx++)
         {
            double jacobianEntry = jacobianMatrix.get(rowIdx, jacobianColumn);
            stackedContactJacobian.set(rowOffset + rowIdx, systemColumnIndex, jacobianEntry);
         }
      }

      return rowOffset + numRows;
   }

   private int getSystemJacobianColumn(OneDoFJointBasics joint)
   {
      return SPATIAL_DIMENSIONS + jointToIndexMap.get(joint);
   }
}
