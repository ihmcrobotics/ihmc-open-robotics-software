package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import gnu.trove.list.array.TIntArrayList;
import gnu.trove.map.hash.TObjectIntHashMap;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.linsol.svd.SolvePseudoInverseSvd_DDRM;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.PointJacobian;

import java.util.ArrayList;
import java.util.List;

public class ContactTangentCalculator
{
   private final RobotSide robotSide;
   private final FullHumanoidRobotModel fullRobotModel;
   private final GeometricJacobian geometricJacobian;

   /* Point jacobian to compute joint motion during contact adjustment calc */
   private final PointJacobian armJacobianCalculator = new PointJacobian();
   private final DMatrixRMaj armJacobianInv = new DMatrixRMaj(0);
   private final DMatrixRMaj handVelocityMatrix = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj qdArm = new DMatrixRMaj(0);
   private final DMatrixRMaj qd = new DMatrixRMaj(0);
   private final SolvePseudoInverseSvd_DDRM psuedoInverseSolver = new SolvePseudoInverseSvd_DDRM();

   private final OneDoFJointBasics[] controlledOneDoFJoints;
   private final TObjectIntHashMap<OneDoFJointBasics> jointToIndexMap = new TObjectIntHashMap<>(30);

   private final List<JointBasics> armJoints = new ArrayList<>();
   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FrameVector3D tempVector = new FrameVector3D();

   public ContactTangentCalculator(FullHumanoidRobotModel fullRobotModel, RobotSide robotSide)
   {
      this.robotSide = robotSide;
      this.fullRobotModel = fullRobotModel;

      armJoints.add(fullRobotModel.getSpineJoint(SpineJointName.SPINE_YAW));
      armJoints.add(fullRobotModel.getArmJoint(robotSide, ArmJointName.SHOULDER_PITCH));
      armJoints.add(fullRobotModel.getArmJoint(robotSide, ArmJointName.SHOULDER_ROLL));
      armJoints.add(fullRobotModel.getArmJoint(robotSide, ArmJointName.SHOULDER_YAW));
      armJoints.add(fullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_PITCH));

      ReferenceFrame jacobianFrame = armJoints.get(0).getPredecessor().getBodyFixedFrame();
      this.geometricJacobian = new GeometricJacobian(armJoints.toArray(new JointBasics[0]), jacobianFrame);

      JointBasics[] controlledJoints = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullRobotModel);
      this.controlledOneDoFJoints = MultiBodySystemTools.filterJoints(controlledJoints, OneDoFJointBasics.class);

      for (int jointIdx = 0; jointIdx < controlledOneDoFJoints.length; jointIdx++)
      {
         OneDoFJointBasics joint = controlledOneDoFJoints[jointIdx];
         jointToIndexMap.put(joint, jointIdx);
      }

      qd.reshape(6 + controlledJoints.length, 1);
      psuedoInverseSolver.setThreshold(0.001);
   }

   /**
    * Computes J^+, psuedo-inverse of the hand Jacobian J
    */
   public void computeJacobian()
   {
      geometricJacobian.compute();

      tempPoint.setToZero(fullRobotModel.getHandControlFrame(robotSide));
      armJacobianCalculator.set(geometricJacobian, tempPoint);
      armJacobianCalculator.compute();

      DMatrixRMaj pointJacobianMatrix = armJacobianCalculator.getJacobianMatrix();
      psuedoInverseSolver.setA(pointJacobianMatrix);
      psuedoInverseSolver.invert(armJacobianInv);
   }

   /**
    * Given a (3 x 1) hand linear velocity v, computes qd = J^+ v where qd is a whole-body velocity.
    */
   public DMatrixRMaj computeWholeBodyVelocity(FrameVector3DReadOnly handVelocityVector)
   {
      tempVector.setIncludingFrame(handVelocityVector);
      tempVector.changeFrame(geometricJacobian.getJacobianFrame());
      tempVector.get(handVelocityMatrix);
      CommonOps_DDRM.mult(armJacobianInv, handVelocityMatrix, qdArm);

      qd.zero();
      for (int i = 0; i < armJoints.size(); i++)
      {
         int systemIndex = 6 + jointToIndexMap.get(armJoints.get(i));
         qd.set(systemIndex, 0, qdArm.get(i));
      }

      return qd;
   }
}
