package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.controllers.PDController;
import us.ihmc.robotics.controllers.ParameterizedPDController;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.sensors.ForceSensorData;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.HashMap;

public class RemoteControllerState extends HighLevelControllerState
{
   private static final HighLevelControllerName controllerState = HighLevelControllerName.REMOTE;
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder;
   private final RemoteControllerStateNetworkingThread networker;
   private final HashMap<String, Double> jointGains;

   private YoDouble[] desiredPositions;
   private PDController[] controllers;
   private double lastTime = 0;
   HighLevelHumanoidControllerToolbox controllerToolbox;

   public RemoteControllerState(OneDoFJointBasics[] controlledJoints,
                                HighLevelHumanoidControllerToolbox controllerToolbox,
                                HighLevelControllerParameters highLevelControllerParameters)
   {
      super(controllerState, highLevelControllerParameters, controlledJoints);
      lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder(controlledJoints.length);
      for (OneDoFJointBasics joint : controlledJoints)
      {
         System.out.println(
               joint.getName() + " Limit: " + joint.getJointLimitLower() + " - " + joint.getJointLimitUpper() + " Effort limit: " + joint.getEffortLimitLower()
               + " - " + joint.getEffortLimitUpper());
      }
      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledJoints);
      controllers = new PDController[controlledJoints.length];
      desiredPositions = new YoDouble[controlledJoints.length];
      for (int i = 0; i < desiredPositions.length; i++)
      {
         desiredPositions[i] = new YoDouble("desiredPositions" + controlledJoints[i].getName(), this.getYoRegistry());
      }
      this.controllerToolbox = controllerToolbox;

      jointGains = new HashMap<String, Double>()
      {{
         put("l_leg_aky", 100.0);
         put("r_leg_aky", 100.0);
         put("l_leg_akx", 100.0);
         put("r_leg_akx", 100.0);
         put("l_leg_kny", 1000.0);
         put("r_leg_kny", 1000.0);
         put("l_leg_hpz", 100.0);
         put("r_leg_hpz", 100.0);
         put("l_leg_hpx", 300.0);
         put("r_leg_hpx", 300.0);
         put("l_leg_hpy", 1000.0);
         put("r_leg_hpy", 1000.0);
         put("back_bkz", 500.0);
         put("back_bky", 500.0);
         put("back_bkx", 500.0);
         put("r_arm_shz", 100.0);
         put("l_arm_shx", 100.0);
         put("r_arm_shx", 100.0);
         put("l_arm_ely", 100.0);
         put("r_arm_ely", 100.0);
         put("l_arm_elx", 100.0);
         put("r_arm_elx", 100.0);
         put("l_arm_wry", 10.0);
         put("r_arm_wry", 10.0);
         put("l_arm_wrx", 10.0);
         put("r_arm_wrx", 10.0);
         put("l_arm_wry2", 10.0);
         put("r_arm_wry2", 10.0);
         put("l_arm_shz", 10.0);
         put("neck_ry", 10.0);
      }};

      for (int i = 0; i < controlledJoints.length; i++)
      {
         lowLevelOneDoFJointDesiredDataHolder.setJointControlMode(controlledJoints[i], JointDesiredControlMode.EFFORT);
         YoDouble proportionalGain = new YoDouble("proportionalGain" + controlledJoints[i].getName(), getYoRegistry());
         YoDouble derivateGain = new YoDouble("derivateGain" + controlledJoints[i].getName(), getYoRegistry());
         //         proportionalGain.set(jointGains.get(controlledJoints[i].getName()));
         proportionalGain.set(1.0);
         derivateGain.set(0.1 * proportionalGain.getValue());
         controllers[i] = new PDController(proportionalGain, derivateGain, "pdControllerJoint" + controlledJoints[i].getName(), getYoRegistry());
      }

      networker = new RemoteControllerStateNetworkingThread(controlledJoints.length);
      networker.start();
   }

   @Override
   public void doAction(double timeInState)
   {
      for (int i = 0; i < controlledJoints.length; i++)
      {
         double desiredAngle = networker.getDesiredAngle(controlledJoints[i].getName());
         desiredPositions[i].set(desiredAngle);
         double torque = controllers[i].computeForAngles(controlledJoints[i].getQ(), desiredPositions[i].getDoubleValue(), controlledJoints[i].getQd(), 0);
         lowLevelOneDoFJointDesiredDataHolder.setDesiredJointTorque(controlledJoints[i], torque);
//         lowLevelOneDoFJointDesiredDataHolder.setDesiredJointPosition(controlledJoints[i], desiredAngle);

         networker.setCurrentAngle(controlledJoints[i].getName(), controlledJoints[i].getQ());
         networker.setJointSpeed(controlledJoints[i].getName(), controlledJoints[i].getQd());
      }

      MovingReferenceFrame baseFrame = controllerToolbox.getFullRobotModel().getRootBody().getBodyFixedFrame();
      networker.setCenterOfMassFrame(controllerToolbox.getCenterOfMassFrame());
      networker.setBaseFrame(baseFrame);
      networker.setPelvisFrame(controllerToolbox.getPelvisZUpFrame());
      networker.setTime(timeInState);

      FramePoint2D copFramePoint = new FramePoint2D();
      controllerToolbox.getCoP(copFramePoint);
      networker.setCoP(copFramePoint);

      ForceSensorDataReadOnly dataLeft = controllerToolbox.getWristForceSensor(RobotSide.LEFT);
      networker.setWristLeft(dataLeft);
      ForceSensorDataReadOnly dataRight = controllerToolbox.getWristForceSensor(RobotSide.RIGHT);
      networker.setWristRight(dataRight);

      lastTime = timeInState;
      lowLevelOneDoFJointDesiredDataHolder.completeWith(getStateSpecificJointSettings());
   }

   @Override
   public void onEntry()
   {
      HashMap<String, Double> initialPoseCorrections = new HashMap<String, Double>()
      {{
         put("l_leg_aky", 0.3);
         put("r_leg_aky", 0.3);
         put("l_leg_akx", 0.0);
         put("r_leg_akx", 0.0);
         put("l_leg_kny", 0.0);
         put("r_leg_kny", 0.0);
         put("l_leg_hpz", 0.0);
         put("r_leg_hpz", 0.0);
         put("l_leg_hpx", 0.0);
         put("r_leg_hpx", 0.0);
         put("l_leg_hpy", 0.0);
         put("r_leg_hpy", 0.0);
         put("back_bkz", 0.0);
         put("back_bky", 0.0);
         put("back_bkx", 0.0);
         put("r_arm_shz", 0.0);
         put("l_arm_shx", -1.0);
         put("r_arm_shx", 1.0);
         put("l_arm_ely", 0.0);
         put("r_arm_ely", 0.0);
         put("l_arm_elx", 0.0);
         put("r_arm_elx", 0.0);
         put("l_arm_wry", 0.0);
         put("r_arm_wry", 0.0);
         put("l_arm_wrx", 0.0);
         put("r_arm_wrx", 0.0);
         put("l_arm_wry2", 0.0);
         put("r_arm_wry2", 0.0);
         put("l_arm_shz", 0.0);
         put("neck_ry", 0.0);
      }};
      // Do nothing
      for (int i = 0; i < controlledJoints.length; i++)
      {
         //         desiredPositions[i].set(initialPoseCorrections.get(controlledJoints[i].getName()));
         desiredPositions[i].set(controlledJoints[i].getQ());
         networker.setDesiredAngle(controlledJoints[i].getName(), desiredPositions[i].getDoubleValue());
      }
      lastTime = 0;
   }

   @Override
   public void onExit()
   {
      // Do nothing

   }

   @Override
   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }
}
