package us.ihmc.externalControl;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.externalControl.global.ExternalControlWrapper;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.yoVariables.math.YoMatrix;
import us.ihmc.yoVariables.registry.YoRegistry;
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

   public ExternalControl(RigidBodyBasics baseBody,
                          WholeBodySetpointParameters homeConfiguration,
                          OneDoFJointReadOnly[] joints,
                          double defaultStiffness,
                          double defaultDamping,
                          YoRegistry parentRegistry)
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
      String[] stateNames = STATE_ORDER.keySet().toArray(new String[0]);
      yoSolutionRobotState = new YoMatrix("extSoln_", 2 * joints.length + 13, 1, stateNames, registry);
      yoRobotState = new YoMatrix("extRobotState_", 2 * joints.length + 13, 1, stateNames, registry);
      String[] torqueNames = TORQUES_ORDER.keySet().toArray(new String[0]);
      yoSolutionTorque = new YoMatrix("extSoln_", joints.length, 1, torqueNames, registry);



      DMatrixRMaj homeConfigurationVector = new DMatrixRMaj(joints.length, 1);
      for (int i = 0; i < joints.length; i++)
         homeConfigurationVector.set(i, 0, homeConfiguration.getSetpoint(joints[i].getName()));
      externalControlImpl.setHomeJointConfiguration(homeConfigurationVector.data, homeConfigurationVector.numRows);

      solutionDebugData = new DMatrixRMaj(externalControlImpl.getDebugDataSize(), 1);

      parentRegistry.addChild(registry);
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

   public void writeRobotState(double currentTime, int hardwareStatus, int behaviorStatus)
   {
      setRobotState();
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
      }
      //      LogTools.info("returned stiffness" + solutionStiffnessVector);
   }

   public void readDebugData()
   {
      if (!externalControlImpl.getDebugData(solutionDebugData.data))
      {
         throw new RuntimeException("Failed to retrieve debug data.");
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

   public static final Map<String, Integer> STATE_ORDER = new HashMap<>()  {{
      put("q_PELVIS_POSITION_X",   0);
      put("q_PELVIS_POSITION_Y",   1);
      put("q_PELVIS_POSITION_Z",   2);
      put("q_PELVIS_QUATERNION_X", 3);
      put("q_PELVIS_QUATERNION_Y", 4);
      put("q_PELVIS_QUATERNION_Z", 5);
      put("q_PELVIS_QUATERNION_W", 6);
      put("q_LEFT_HIP_Z",          7);
      put("q_LEFT_HIP_X",          8);
      put("q_LEFT_HIP_Y",          9);
      put("q_LEFT_KNEE_Y",         10);
      put("q_LEFT_ANKLE_Y",        11);
      put("q_LEFT_ANKLE_X",        12);
      put("q_RIGHT_HIP_Z",         13);
      put("q_RIGHT_HIP_X",         14);
      put("q_RIGHT_HIP_Y",         15);
      put("q_RIGHT_KNEE_Y",        16);
      put("q_RIGHT_ANKLE_Y",       17);
      put("q_RIGHT_ANKLE_X",       18);
      put("q_SPINE_Z",             19);
      put("q_SPINE_X",             20);
      put("q_SPINE_Y",             21);
      put("q_LEFT_SHOULDER_Y",     22);
      put("q_LEFT_SHOULDER_X",     23);
      put("q_LEFT_SHOULDER_Z",     24);
      put("q_LEFT_ELBOW_Y",        25);
      put("q_RIGHT_SHOULDER_Y",    26);
      put("q_RIGHT_SHOULDER_X",    27);
      put("q_RIGHT_SHOULDER_Z",    28);
      put("q_RIGHT_ELBOW_Y",       29);
      put("qd_PELVIS_LINEAR_VELOCITY_X",     30);
      put("qd_PELVIS_LINEAR_VELOCITY_Y",     31);
      put("qd_PELVIS_LINEAR_VELOCITY_Z",     32);
      put("qd_PELVIS_ANGULAR_VELOCITY_X",    33);
      put("qd_PELVIS_ANGULAR_VELOCITY_Y",    34);
      put("qd_PELVIS_ANGULAR_VELOCITY_Z",    35);
      put("qd_LEFT_HIP_Z",                   36);
      put("qd_LEFT_HIP_X",                   37);
      put("qd_LEFT_HIP_Y",                   38);
      put("qd_LEFT_KNEE_Y",                  39);
      put("qd_LEFT_ANKLE_Y",                 40);
      put("qd_LEFT_ANKLE_X",                 41);
      put("qd_RIGHT_HIP_Z",                  42);
      put("qd_RIGHT_HIP_X",                  43);
      put("qd_RIGHT_HIP_Y",                  44);
      put("qd_RIGHT_KNEE_Y",                 45);
      put("qd_RIGHT_ANKLE_Y",                46);
      put("qd_RIGHT_ANKLE_X",                47);
      put("qd_SPINE_Z",                      48);
      put("qd_SPINE_X",                      49);
      put("qd_SPINE_Y",                      50);
      put("qd_LEFT_SHOULDER_Y",              51);
      put("qd_LEFT_SHOULDER_X",              52);
      put("qd_LEFT_SHOULDER_Z",              53);
      put("qd_LEFT_ELBOW_Y",                 54);
      put("qd_RIGHT_SHOULDER_Y",             55);
      put("qd_RIGHT_SHOULDER_X",             56);
      put("qd_RIGHT_SHOULDER_Z",             57);
      put("qd_RIGHT_ELBOW_Y",                58);
   }};

   private static final Map<String, Integer> TORQUES_ORDER = new HashMap<>() {{
      put("tau_LEFT_HIP_Z",       0);
      put("tau_LEFT_HIP_X",       1);
      put("tau_LEFT_HIP_Y",       2);
      put("tau_LEFT_KNEE_Y",      3);
      put("tau_LEFT_ANKLE_Y",     4);
      put("tau_LEFT_ANKLE_X",     5);
      put("tau_RIGHT_HIP_Z",      6);
      put("tau_RIGHT_HIP_X",      7);
      put("tau_RIGHT_HIP_Y",      8);
      put("tau_RIGHT_KNEE_Y",     9);
      put("tau_RIGHT_ANKLE_Y",    10);
      put("tau_RIGHT_ANKLE_X",    11);
      put("tau_SPINE_Z",          12);
      put("tau_SPINE_X",          13);
      put("tau_SPINE_Y",          14);
      put("tau_LEFT_SHOULDER_Y",  15);
      put("tau_LEFT_SHOULDER_X",  16);
      put("tau_LEFT_SHOULDER_Z",  17);
      put("tau_LEFT_ELBOW_Y",     18);
      put("tau_RIGHT_SHOULDER_Y", 19);
      put("tau_RIGHT_SHOULDER_X", 20);
      put("tau_RIGHT_SHOULDER_Z", 21);
      put("tau_RIGHT_ELBOW_Y",    22);
   }};
}