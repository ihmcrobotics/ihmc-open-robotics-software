package us.ihmc.externalControl;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.externalControl.global.ExternalControlWrapper;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.yoVariables.filters.AlphaFilterTools;
import us.ihmc.yoVariables.math.YoMatrix;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import java.util.HashMap;

public class ExternalControl
{
   private final ExternalControlWrapper.ExternalControlImpl externalControlImpl;
   private final DMatrixRMaj robotState;
   private final DMatrixRMaj robotControl;
   private final DMatrixRMaj feetPositions;
   private boolean leftInContact;
   private boolean rightInContact;
   private final YoMatrix yoSolutionRobotState;
   private final YoMatrix yoRobotState;
   private final YoMatrix yoSolutionTorque;
   private final DMatrixRMaj solutionRobotState;
   private final DMatrixRMaj solutionTorqueVector;
   private final DMatrixRMaj solutionStiffnessVector;
   private final DMatrixRMaj solutionDampingVector;
   public final DMatrixRMaj solutionDebugData;
   public final String[] debugDataNames = {"behavior", "primal_res", "dual_res", "qp_iters", "solve_time",
                                           "L_x", "L_y", "L_z", "R_x", "R_y", "R_z"}; // TODO: query the ext controller for this?
   private final RigidBodyBasics baseBody;
   private final OneDoFJointReadOnly[] joints;
   private final FramePose3D basePose = new FramePose3D();
   private final FrameVector3D tempVector = new FrameVector3D();
   private final FrameVector3D tempPoint = new FrameVector3D();
   private final FramePose3D solutionBasePose = new FramePose3D();
   private final HashMap<OneDoFJointReadOnly, SolutionJointData> solutionJointData = new HashMap<>();

   private final YoDouble leftHipZTauBreakFrequency;
   private final AlphaFilteredYoVariable filteredLeftHipZTau;
   private final YoDouble rightHipZTauBreakFrequency;
   private final AlphaFilteredYoVariable filteredRightHipZTau;
   private final YoDouble spineZTauBreakFrequency;
   private final AlphaFilteredYoVariable filteredSpineZTau;

   public ExternalControl(RigidBodyBasics baseBody,
                          WholeBodySetpointParameters homeConfiguration,
                          OneDoFJointReadOnly[] joints,
                          double defaultStiffness,
                          double defaultDamping,
                          YoRegistry parentRegistry,
                          HighLevelHumanoidControllerToolbox controllerToolbox)
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

      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      yoSolutionRobotState = new YoMatrix("extSoln_", 2 * joints.length + 13, 1, STATE_ORDER, registry);
      yoRobotState = new YoMatrix("extRobotState_", 2 * joints.length + 13, 1, STATE_ORDER, registry);
      yoSolutionTorque = new YoMatrix("extSolnTorque_", joints.length, 1, TORQUES_ORDER, registry);



      DMatrixRMaj homeConfigurationVector = new DMatrixRMaj(joints.length, 1);
      for (int i = 0; i < joints.length; i++)
         homeConfigurationVector.set(i, 0, homeConfiguration.getSetpoint(joints[i].getName()));
      externalControlImpl.setHomeJointConfiguration(homeConfigurationVector.data, homeConfigurationVector.numRows);

      solutionDebugData = new DMatrixRMaj(externalControlImpl.getDebugDataSize(), 1);

      parentRegistry.addChild(registry);

      leftHipZTauBreakFrequency = new YoDouble("leftHipZTauBreakFrequency", registry);
      leftHipZTauBreakFrequency.set(5.0);
      DoubleProvider leftHipZTauAlphaProvider = () -> AlphaFilterTools.computeAlphaGivenBreakFrequencyProperly(leftHipZTauBreakFrequency.getDoubleValue(),
                                                                                                               controllerToolbox.getControlDT());
      filteredLeftHipZTau = new AlphaFilteredYoVariable("filteredLeftHipZTau", registry, leftHipZTauAlphaProvider);

      rightHipZTauBreakFrequency = new YoDouble("rightHipZTauBreakFrequency", registry);
      rightHipZTauBreakFrequency.set(5.0);
      DoubleProvider rightHipZTauAlphaProvider = () -> AlphaFilterTools.computeAlphaGivenBreakFrequencyProperly(rightHipZTauBreakFrequency.getDoubleValue(),
                                                                                                                controllerToolbox.getControlDT());
      filteredRightHipZTau = new AlphaFilteredYoVariable("filteredRightHipZTau", registry, rightHipZTauAlphaProvider);

      spineZTauBreakFrequency = new YoDouble("spineZTauBreakFrequency", registry);
      spineZTauBreakFrequency.set(5.0);
      DoubleProvider spineZTauAlphaProvider = () -> AlphaFilterTools.computeAlphaGivenBreakFrequencyProperly(spineZTauBreakFrequency.getDoubleValue(),
                                                                                                             controllerToolbox.getControlDT());
      filteredSpineZTau = new AlphaFilteredYoVariable("filteredSpineZTau", registry, spineZTauAlphaProvider);

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

   public void writeRobotState(double currentTime, int hardwareStatus, int behaviorStatus, double heightZOffset)
   {
      setRobotState(heightZOffset);
      setRobotControl();
      yoRobotState.set(robotState);
      if (!externalControlImpl.updateRobotState(currentTime,
                                                robotState.data,
                                                robotState.getNumRows(),
                                                robotControl.data,
                                                robotControl.getNumRows(),
                                                leftInContact,
                                                rightInContact,
                                                feetPositions.data,
                                                feetPositions.getNumRows(),
                                                hardwareStatus,
                                                behaviorStatus))
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

      yoSolutionRobotState.set(solutionRobotState);
      yoSolutionTorque.set(solutionTorqueVector);
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

         // Filter torque for certain joints
         if (joints[i].getName().equals("LEFT_HIP_Z")) {
            filteredLeftHipZTau.update(data.torque);
            data.torque = filteredLeftHipZTau.getDoubleValue();
         }
         if (joints[i].getName().equals("RIGHT_HIP_Z")) {
            filteredRightHipZTau.update(data.torque);
            data.torque = filteredRightHipZTau.getDoubleValue();
         }
         if (joints[i].getName().equals("SPINE_Z")) {
            filteredSpineZTau.update(data.torque);
            data.torque = filteredSpineZTau.getDoubleValue();
         }
      }

      // Filter hip
      //      LogTools.info("returned stiffness" + solutionStiffnessVector);
   }

   public void readDebugData()
   {
      if (!externalControlImpl.getDebugData(solutionDebugData.data))
      {
         throw new RuntimeException("Failed to retrieve debug data.");
      }
   }

   public void startSocket()
   {
      externalControlImpl.startSocket();
   }

   public void stopSocket()
   {
      externalControlImpl.stopSocket();
   }

   public SolutionJointData getSolutionData(OneDoFJointReadOnly joint)
   {
      return solutionJointData.get(joint);
   }

   private void setRobotState(double heightZOffset)
   {
      basePose.setToZero(baseBody.getBodyFixedFrame());
      basePose.changeFrame(ReferenceFrame.getWorldFrame());

      // add a z offset to the base pose. This is meant to account for vertical drift
      basePose.getPosition().addZ(heightZOffset);

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

   public static final String[] STATE_ORDER = new String[] {
      "q_PELVIS_POSITION_X",
      "q_PELVIS_POSITION_Y",
      "q_PELVIS_POSITION_Z",
      "q_PELVIS_QUATERNION_X",
      "q_PELVIS_QUATERNION_Y",
      "q_PELVIS_QUATERNION_Z",
      "q_PELVIS_QUATERNION_W",
      "q_LEFT_HIP_Z",
      "q_LEFT_HIP_X",
      "q_LEFT_HIP_Y",
      "q_LEFT_KNEE_Y",
      "q_LEFT_ANKLE_Y",
      "q_LEFT_ANKLE_X",
      "q_RIGHT_HIP_Z",
      "q_RIGHT_HIP_X",
      "q_RIGHT_HIP_Y",
      "q_RIGHT_KNEE_Y",
      "q_RIGHT_ANKLE_Y",
      "q_RIGHT_ANKLE_X",
      "q_SPINE_Z",
      "q_SPINE_X",
      "q_SPINE_Y",
      "q_LEFT_SHOULDER_Y",
      "q_LEFT_SHOULDER_X",
      "q_LEFT_SHOULDER_Z",
      "q_LEFT_ELBOW_Y",
      "q_RIGHT_SHOULDER_Y",
      "q_RIGHT_SHOULDER_X",
      "q_RIGHT_SHOULDER_Z",
      "q_RIGHT_ELBOW_Y",
      "qd_PELVIS_LINEAR_VELOCITY_X",
      "qd_PELVIS_LINEAR_VELOCITY_Y",
      "qd_PELVIS_LINEAR_VELOCITY_Z",
      "qd_PELVIS_ANGULAR_VELOCITY_X",
      "qd_PELVIS_ANGULAR_VELOCITY_Y",
      "qd_PELVIS_ANGULAR_VELOCITY_Z",
      "qd_LEFT_HIP_Z",
      "qd_LEFT_HIP_X",
      "qd_LEFT_HIP_Y",
      "qd_LEFT_KNEE_Y",
      "qd_LEFT_ANKLE_Y",
      "qd_LEFT_ANKLE_X",
      "qd_RIGHT_HIP_Z",
      "qd_RIGHT_HIP_X",
      "qd_RIGHT_HIP_Y",
      "qd_RIGHT_KNEE_Y",
      "qd_RIGHT_ANKLE_Y",
      "qd_RIGHT_ANKLE_X",
      "qd_SPINE_Z",
      "qd_SPINE_X",
      "qd_SPINE_Y",
      "qd_LEFT_SHOULDER_Y",
      "qd_LEFT_SHOULDER_X",
      "qd_LEFT_SHOULDER_Z",
      "qd_LEFT_ELBOW_Y",
      "qd_RIGHT_SHOULDER_Y",
      "qd_RIGHT_SHOULDER_X",
      "qd_RIGHT_SHOULDER_Z",
      "qd_RIGHT_ELBOW_Y"};

   private static final String[] TORQUES_ORDER = new String[] {
      "tau_LEFT_HIP_Z",
      "tau_LEFT_HIP_X",
      "tau_LEFT_HIP_Y",
      "tau_LEFT_KNEE_Y",
      "tau_LEFT_ANKLE_Y",
      "tau_LEFT_ANKLE_X",
      "tau_RIGHT_HIP_Z",
      "tau_RIGHT_HIP_X",
      "tau_RIGHT_HIP_Y",
      "tau_RIGHT_KNEE_Y",
      "tau_RIGHT_ANKLE_Y",
      "tau_RIGHT_ANKLE_X",
      "tau_SPINE_Z",
      "tau_SPINE_X",
      "tau_SPINE_Y",
      "tau_LEFT_SHOULDER_Y",
      "tau_LEFT_SHOULDER_X",
      "tau_LEFT_SHOULDER_Z",
      "tau_LEFT_ELBOW_Y",
      "tau_RIGHT_SHOULDER_Y",
      "tau_RIGHT_SHOULDER_X",
      "tau_RIGHT_SHOULDER_Z",
      "tau_RIGHT_ELBOW_Y"};
}