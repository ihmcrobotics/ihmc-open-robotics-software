package us.ihmc.externalControl;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;

import java.util.HashMap;

public class ExternalControl
{
   private final ExternalControlWrapper.ExternalControlImpl externalControlImpl;
   private final DMatrixRMaj robotState;
   private final DMatrixRMaj robotControl;
   private final DMatrixRMaj feetPositions;
   private boolean leftInContact;
   private boolean rightInContact;
   private final DMatrixRMaj solutionRobotState;
   private final DMatrixRMaj solutionTorqueVector;
   private final DMatrixRMaj solutionStiffnessVector;
   private final DMatrixRMaj solutionDampingVector;
   private final RigidBodyBasics baseBody;
   private final OneDoFJointReadOnly[] joints;
   private final FramePose3D basePose = new FramePose3D();
   private final FrameVector3D tempVector = new FrameVector3D();
   private final FrameVector3D tempPoint = new FrameVector3D();
   private final FramePose3D solutionBasePose = new FramePose3D();
   private final HashMap<OneDoFJointReadOnly, SolutionJointData> solutionJointData = new HashMap<>();

   public ExternalControl(RigidBodyBasics baseBody, OneDoFJointReadOnly[] joints, double defaultStiffness, double defaultDamping)
   {
      this.baseBody = baseBody;
      this.joints = joints;
      robotState = new DMatrixRMaj(2 * joints.length + 13, 1);
      robotControl = new DMatrixRMaj(joints.length, 1);
      feetPositions = new DMatrixRMaj(6, 1);
      solutionRobotState = new DMatrixRMaj(2 * joints.length + 13, 1);
      solutionTorqueVector = new DMatrixRMaj(joints.length, 1);
      solutionStiffnessVector = new DMatrixRMaj(joints.length, 1);
      solutionDampingVector = new DMatrixRMaj(joints.length, 1);
      for (OneDoFJointReadOnly joint : joints)
         solutionJointData.put(joint, new SolutionJointData());
      externalControlImpl = new ExternalControlWrapper.ExternalControlImpl(defaultStiffness, defaultDamping, joints.length);
   }

   public void setFootStates(SideDependentList<? extends ReferenceFrame> soleFrames, boolean leftInContact, boolean rightInContact)
   {
      int start = 0;
      for (RobotSide robotSide : RobotSide.values)
      {
         tempPoint.setToZero(soleFrames.get(robotSide));
         tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
         tempPoint.get(start, feetPositions);
         start += 3;
      }
      this.leftInContact = leftInContact;
      this.rightInContact = rightInContact;
   }

   public void writeRobotState(double currentTime, int hardwareStatus)
   {
      setRobotState();
      setRobotControl();
      if (!externalControlImpl.updateRobotState(currentTime,
                                                robotState.data,
                                                robotState.getNumRows(),
                                                robotControl.data,
                                                robotControl.getNumRows(),
                                                leftInContact,
                                                rightInContact,
                                                feetPositions.data,
                                                feetPositions.getNumRows(),
                                                hardwareStatus))
         throw new RuntimeException("Failed to successfully write the hardware state across the barrier.");
   }

   public void readControlSolution()
   {
      if (!externalControlImpl.getSolution(solutionRobotState.data,
                                           solutionRobotState.numRows,
                                           solutionTorqueVector.data,
                                           solutionTorqueVector.numRows,
                                           solutionStiffnessVector.data,
                                           solutionStiffnessVector.numRows,
                                           solutionDampingVector.data,
                                           solutionDampingVector.numRows))
      {
         throw new RuntimeException("Failed to retrieve solution data.");
      }
      solutionBasePose.getPosition().set(solutionRobotState);
      solutionBasePose.getOrientation().set(3, 0, solutionRobotState);
      int positionStart = 7;
      int velocityStart = 7 + 6 + joints.length;
      for (int i = 0; i < joints.length; i++)
      {
         SolutionJointData data = solutionJointData.get(joints[i]);
         data.desiredPosition = solutionRobotState.get(positionStart + i, 0);
         data.desiredVelocity = solutionRobotState.get(velocityStart + i, 0);
         data.torque = solutionTorqueVector.get(i, 0);
         data.stiffness = solutionStiffnessVector.get(i, 0);
         data.damping = solutionDampingVector.get(i, 0);
      }
   }

   public SolutionJointData getSolutionData(OneDoFJointReadOnly joint)
   {
      return solutionJointData.get(joint);
   }

   private void setRobotState()
   {
      basePose.setToZero(baseBody.getBodyFixedFrame());
      basePose.changeFrame(ReferenceFrame.getWorldFrame());
      // set the configuration state for the robot
      basePose.getPosition().get(0, robotState);
      basePose.getOrientation().get(3, robotState);
      int start = 7;
      for (int i = 0; i < joints.length; i++)
      {
         robotState.set(start + i, joints[i].getQ());
      }
      // set linear velocity in body frame
      start += joints.length;
      tempVector.setIncludingFrame(baseBody.getBodyFixedFrame().getTwistOfFrame().getLinearPart());
      tempVector.changeFrame(baseBody.getBodyFixedFrame());
      tempVector.get(start, robotState);
      // set angular velocity in body frame
      start += 3;
      tempVector.setIncludingFrame(baseBody.getBodyFixedFrame().getTwistOfFrame().getAngularPart());
      tempVector.changeFrame(baseBody.getBodyFixedFrame());
      tempVector.get(start, robotState);
      // set joint velocity
      start += 3;
      for (int i = 0; i < joints.length; i++)
      {
         robotState.set(start + i, joints[i].getQd());
      }
   }

   private void setRobotControl()
   {
      for (int i = 0; i < joints.length; i++)
      {
         robotControl.set(i, joints[i].getTau());
      }
   }

   public static class SolutionJointData
   {
      double desiredPosition;
      double desiredVelocity;
      double stiffness;
      double damping;
      double torque;

      public void getJointDesiredOutput(JointDesiredOutputBasics jointDesiredOutputToPack)
      {
         jointDesiredOutputToPack.setDesiredPosition(desiredPosition);
         jointDesiredOutputToPack.setDesiredVelocity(desiredVelocity);
         jointDesiredOutputToPack.setDesiredTorque(torque);
         jointDesiredOutputToPack.setStiffness(stiffness);
         jointDesiredOutputToPack.setDamping(damping);
      }
   }
}