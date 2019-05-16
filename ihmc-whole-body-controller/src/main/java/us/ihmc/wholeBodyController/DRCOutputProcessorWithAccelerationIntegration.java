package us.ihmc.wholeBodyController;

import java.util.ArrayList;
import java.util.LinkedHashMap;

import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class DRCOutputProcessorWithAccelerationIntegration implements DRCOutputProcessor
{
   private final boolean runningOnRealRobot;
   private final DRCOutputProcessor drcOutputProcessor;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble alphaDesiredVelocity = new YoDouble(
         "alphaDesiredVelocity",
         "Filter for velocity control in order to achieve acceleration control. Zero means compliant, but poor acceleration. One means stiff, but good acceleration tracking",
         registry);
   private final YoDouble alphaDesiredPosition = new YoDouble(
         "alphaDesiredPosition",
         "Filter for position control in order to achieve acceleration control. Zero means compliant, but poor acceleration. One means stiff, but good acceleration tracking",
         registry);

   private final YoDouble alphaArmDesiredVelocity = new YoDouble(
         "alphaArmDesiredVelocity",
         "Filter for velocity control in order to achieve acceleration control. Zero means compliant, but poor acceleration. One means stiff, but good acceleration tracking (Arms only)",
         registry);
   private final YoDouble alphaArmDesiredPosition = new YoDouble(
         "alphaArmDesiredPosition",
         "Filter for position control in order to achieve acceleration control. Zero means compliant, but poor acceleration. One means stiff, but good acceleration tracking (Arms only)",
         registry);

   private final YoDouble kVelJointTorque = new YoDouble("kVelJointTorque",
         "Gain for velocity control in order to achieve acceleration control using additional joint torque", registry);

   private final YoDouble kPosJointTorque = new YoDouble("kPosJointTorque",
         "Gain for position control in order to achieve acceleration control using additional joint torque", registry);

   private final YoDouble kVelArmJointTorque = new YoDouble("kVelArmJointTorque",
         "Gain for velocity control in order to achieve acceleration control using additional joint torque (Arms only)", registry);

   private final YoDouble kPosArmJointTorque = new YoDouble("kPosArmJointTorque",
         "Gain for position control in order to achieve acceleration control using additional joint torque (Arms only)", registry);

   private PairList<OneDoFJointBasics, JointDesiredOutputBasics> jointStateAndData;
   private LinkedHashMap<JointDesiredOutputBasics, YoDouble> alphaDesiredVelocityMap;
   private LinkedHashMap<JointDesiredOutputBasics, YoDouble> alphaDesiredPositionMap;
   private LinkedHashMap<JointDesiredOutputBasics, YoDouble> velocityTorqueMap;
   private LinkedHashMap<JointDesiredOutputBasics, YoDouble> positionTorqueMap;
   private LinkedHashMap<JointDesiredOutputBasics, YoDouble> kVelJointTorqueMap;
   private LinkedHashMap<JointDesiredOutputBasics, YoDouble> kPosJointTorqueMap;
   private LinkedHashMap<JointDesiredOutputBasics, YoDouble> desiredVelocities;
   private LinkedHashMap<JointDesiredOutputBasics, YoDouble> desiredPositions;

   private final double updateDT;
   private final boolean conservative;

   // List of the joints for which we do integrate the desired accelerations by default
   private final LegJointName[] legJointsForIntegratingAcceleration;
   private final ArmJointName[] armJointsForIntegratingAcceleration;
   private final SpineJointName[] spineJointsForIntegratingAcceleration;

   public DRCOutputProcessorWithAccelerationIntegration(DRCOutputProcessor drcOutputWriter, LegJointName[] legJointToIntegrate,
         ArmJointName[] armJointToIntegrate, SpineJointName[] spineJointToIntegrate, double updateDT, boolean runningOnRealRobot)
   {
      this(drcOutputWriter, legJointToIntegrate, armJointToIntegrate, spineJointToIntegrate, updateDT, runningOnRealRobot, false);
   }

   public DRCOutputProcessorWithAccelerationIntegration(DRCOutputProcessor drcOutputWriter, LegJointName[] legJointToIntegrate,
         ArmJointName[] armJointToIntegrate, SpineJointName[] spineJointToIntegrate, double updateDT, boolean runningOnRealRobot, boolean conservative)
   {
      this.runningOnRealRobot = runningOnRealRobot;
      this.conservative = conservative;
      this.drcOutputProcessor = drcOutputWriter;
      this.updateDT = updateDT;
      if(drcOutputWriter != null)
      {
         registry.addChild(drcOutputWriter.getControllerYoVariableRegistry());
      }

      alphaDesiredVelocity.set(0.0);
      alphaDesiredPosition.set(0.0);
      kVelJointTorque.set(0.0);
      kPosJointTorque.set(0.0);

      alphaArmDesiredVelocity.set(0.0);
      alphaArmDesiredPosition.set(0.0);
      kVelArmJointTorque.set(0.0);
      kPosArmJointTorque.set(0.0);

      if (legJointToIntegrate == null)
         legJointsForIntegratingAcceleration = new LegJointName[0];
      else
         legJointsForIntegratingAcceleration = legJointToIntegrate;

      if (armJointToIntegrate == null)
         armJointsForIntegratingAcceleration = new ArmJointName[0];
      else
         armJointsForIntegratingAcceleration = armJointToIntegrate;

      if (spineJointToIntegrate == null)
         spineJointsForIntegratingAcceleration = new SpineJointName[0];
      else
         spineJointsForIntegratingAcceleration = spineJointToIntegrate;

   }

   public DRCOutputProcessorWithAccelerationIntegration(DRCOutputProcessor drcOutputWriter, double updateDT, boolean runningOnRealRobot)
   {
      this(drcOutputWriter,
            new LegJointName[] { LegJointName.HIP_PITCH, LegJointName.HIP_ROLL, LegJointName.HIP_YAW },
            new ArmJointName[] { ArmJointName.SHOULDER_PITCH, ArmJointName.SHOULDER_ROLL, ArmJointName.SHOULDER_YAW, ArmJointName.ELBOW_PITCH },
            new SpineJointName[] { SpineJointName.SPINE_PITCH, SpineJointName.SPINE_ROLL, SpineJointName.SPINE_YAW },
            updateDT, runningOnRealRobot);
   }

   public void setVelocityGains(double kVelJointTorque, double kVelArmJointTorque)
   {
      this.kVelJointTorque.set(kVelJointTorque);
      this.kVelArmJointTorque.set(kVelArmJointTorque);
   }

   public void setPositionGains(double kPosJointTorque, double kPosArmJointTorque)
   {
      this.kPosJointTorque.set(kPosJointTorque);
      this.kPosArmJointTorque.set(kPosArmJointTorque);
   }

   public void setAlphaDesiredVelocity(double alphaDesiredVelocity, double alphaArmDesiredVelocity)
   {
      this.alphaDesiredVelocity.set(alphaDesiredVelocity);
      this.alphaArmDesiredVelocity.set(alphaArmDesiredVelocity);
   }

   public void setAlphaDesiredPosition(double alphaDesiredPosition, double alphaArmDesiredPosition)
   {
      this.alphaDesiredPosition.set(alphaDesiredPosition);
      this.alphaArmDesiredPosition.set(alphaArmDesiredPosition);
   }

   @Override
   public void initialize()
   {
      if(drcOutputProcessor != null)
      {
         drcOutputProcessor.initialize();
      }
   }

   @Override
   public YoVariableRegistry getControllerYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public void processAfterController(long timestamp)
   {
      for (int i = 0; i < jointStateAndData.size(); i++)
      {
         JointDesiredOutputBasics jointData = jointStateAndData.second(i);
         OneDoFJointBasics jointState = jointStateAndData.first(i);
         YoDouble qd_d_joint = desiredVelocities.get(jointData);
         YoDouble q_d_joint = desiredPositions.get(jointData);

         // FIXME See Jira ticket EOD-965
         boolean integrateDesiredAccelerations = true; //jointState.getIntegrateDesiredAccelerations();
         // Don't call the listener there, we want to be able to set those both from the controller and SCS.
         doAccelerationIntegrationMap.get(jointData).set(integrateDesiredAccelerations, false);

         if (integrateDesiredAccelerations)
         {
            integrateAccelerationsToGetDesiredVelocities(jointState, jointData, qd_d_joint, q_d_joint);
            jointData.setDesiredVelocity(qd_d_joint.getDoubleValue());
            jointData.setDamping(0.0);
            jointData.setStiffness(0.0);

            double kVel = kVelJointTorqueMap.get(jointData).getDoubleValue();
            double kPos = kPosJointTorqueMap.get(jointData).getDoubleValue();
            double velocityTorque = kVel * (qd_d_joint.getDoubleValue() - jointState.getQd());
            double positionTorque = kPos * (q_d_joint.getDoubleValue() - jointState.getQ());

            velocityTorqueMap.get(jointData).set(velocityTorque);
            positionTorqueMap.get(jointData).set(positionTorque);

            jointData.setDesiredTorque(jointData.getDesiredTorque() + velocityTorque + positionTorque);
         }
         else
         {
            qd_d_joint.set(jointState.getQd());
            q_d_joint.set(jointState.getQ());
         }
      }

      if(drcOutputProcessor != null)
      {
         drcOutputProcessor.processAfterController(timestamp);
      }
   }

   private void integrateAccelerationsToGetDesiredVelocities(OneDoFJointBasics jointState, JointDesiredOutputBasics lowLevelJointData, YoDouble qd_d_joint, YoDouble q_d_joint)
   {
      double currentPosition = jointState.getQ();
      double currentVelocity = jointState.getQd();

      if (lowLevelJointData.pollResetIntegratorsRequest())
      {
         //       qd_d_joint.set(currentVelocity);
         qd_d_joint.set(0.0);
         q_d_joint.set(currentPosition);

         return;
      }

      double previousDesiredVelocity = qd_d_joint.getDoubleValue();
      double previousDesiredPosition = q_d_joint.getDoubleValue();

      double desiredAcceleration = lowLevelJointData.getDesiredAcceleration();
      double alphaVel = alphaDesiredVelocityMap.get(lowLevelJointData).getDoubleValue();
      double alphaPos = alphaDesiredPositionMap.get(lowLevelJointData).getDoubleValue();
      double desiredVelocity = doCleverIntegration(previousDesiredVelocity, desiredAcceleration, currentVelocity, alphaVel);
      double desiredPosition = doCleverIntegration(previousDesiredPosition, desiredVelocity, currentPosition, alphaPos);

      qd_d_joint.set(desiredVelocity);
      q_d_joint.set(desiredPosition);
   }

   private double doCleverIntegration(double previousDesiredValue, double desiredValueRate, double currentValue, double alpha)
   {
      return alpha * (previousDesiredValue + desiredValueRate * updateDT) + (1.0 - alpha) * currentValue;
   }

   private final LinkedHashMap<JointDesiredOutputBasics, YoBoolean> doAccelerationIntegrationMap = new LinkedHashMap<>();

   @Override
   public void setLowLevelControllerCoreOutput(FullHumanoidRobotModel controllerRobotModel, JointDesiredOutputListBasics lowLevelControllerCoreOutput)
   {
      if(drcOutputProcessor != null)
      {
         drcOutputProcessor.setLowLevelControllerCoreOutput(controllerRobotModel, lowLevelControllerCoreOutput);
      }

      jointStateAndData = new PairList<>();

      if (conservative)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            for (LegJointName jointName : legJointsForIntegratingAcceleration)
               jointStateAndData.add(controllerRobotModel.getLegJoint(robotSide, jointName), lowLevelControllerCoreOutput.getJointDesiredOutput(controllerRobotModel.getLegJoint(robotSide, jointName)));
            for (ArmJointName jointName : armJointsForIntegratingAcceleration)
               jointStateAndData.add(controllerRobotModel.getArmJoint(robotSide, jointName), lowLevelControllerCoreOutput.getJointDesiredOutput(controllerRobotModel.getArmJoint(robotSide, jointName)));
         }
         
         for (SpineJointName jointName : spineJointsForIntegratingAcceleration)
            jointStateAndData.add(controllerRobotModel.getSpineJoint(jointName), lowLevelControllerCoreOutput.getJointDesiredOutput(controllerRobotModel.getSpineJoint(jointName)));
      }
      else
      {
         for(int i = 0; i < lowLevelControllerCoreOutput.getNumberOfJointsWithDesiredOutput(); i++)
         {
            jointStateAndData.add(lowLevelControllerCoreOutput.getOneDoFJoint(i), lowLevelControllerCoreOutput.getJointDesiredOutput(lowLevelControllerCoreOutput.getOneDoFJoint(i)));
         }
      }

      ArrayList<OneDoFJointBasics> armOneDoFJoints = new ArrayList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics hand = controllerRobotModel.getHand(robotSide);
         if (hand != null)
         {
            JointBasics[] armJoints = MultiBodySystemTools.createJointPath(controllerRobotModel.getChest(), hand);
            OneDoFJointBasics[] filterArmJoints = MultiBodySystemTools.filterJoints(armJoints, OneDoFJointBasics.class);
            for (OneDoFJointBasics armJoint : filterArmJoints)
               armOneDoFJoints.add(armJoint);
         }
      }

      desiredVelocities = new LinkedHashMap<>();
      desiredPositions = new LinkedHashMap<>();

      alphaDesiredVelocityMap = new LinkedHashMap<>();
      alphaDesiredPositionMap = new LinkedHashMap<>();

      kVelJointTorqueMap = new LinkedHashMap<>();
      kPosJointTorqueMap = new LinkedHashMap<>();

      velocityTorqueMap = new LinkedHashMap<>();
      positionTorqueMap = new LinkedHashMap<>();

      for (int i = 0; i < jointStateAndData.size(); i++)
      {
         final OneDoFJointBasics jointState = jointStateAndData.first(i);
         final JointDesiredOutputBasics jointData = jointStateAndData.second(i);
         
         final YoBoolean doAccelerationIntegration = new YoBoolean("doAccelerationIntegration_" + jointState.getName(), registry);
         YoDouble desiredVelocity = new YoDouble("qd_d_" + jointState.getName(), registry);
         YoDouble desiredPosition = new YoDouble("q_d_" + jointState.getName(), registry);

         desiredVelocities.put(jointData, desiredVelocity);
         desiredPositions.put(jointData, desiredPosition);

         YoDouble velocityTorque = new YoDouble("tau_vel_" + jointState.getName(), registry);
         YoDouble positionTorque = new YoDouble("tau_pos_" + jointState.getName(), registry);

         velocityTorqueMap.put(jointData, velocityTorque);
         positionTorqueMap.put(jointData, positionTorque);

         if (armOneDoFJoints.contains(jointState))
         {
            alphaDesiredVelocityMap.put(jointData, alphaArmDesiredVelocity);
            alphaDesiredPositionMap.put(jointData, alphaArmDesiredPosition);
            kVelJointTorqueMap.put(jointData, kVelArmJointTorque);
            kPosJointTorqueMap.put(jointData, kPosArmJointTorque);
         }
         else
         {
            alphaDesiredVelocityMap.put(jointData, alphaDesiredVelocity);
            alphaDesiredPositionMap.put(jointData, alphaDesiredPosition);
            kVelJointTorqueMap.put(jointData, kVelJointTorque);
            kPosJointTorqueMap.put(jointData, kPosJointTorque);
         }

         // We want to be able to set those both from the controller and SCS.
         doAccelerationIntegration.addVariableChangedListener(new VariableChangedListener()
         {
            @Override
            public void notifyOfVariableChange(YoVariable<?> v)
            {
               // FIXME
//               jointState.setIntegrateDesiredAccelerations(doAccelerationIntegration.getBooleanValue());
            }
         });

         doAccelerationIntegration.set(false);
         doAccelerationIntegration.notifyVariableChangedListeners();

         doAccelerationIntegrationMap.put(jointData, doAccelerationIntegration);
      }

      // FIXME
//      if (runningOnRealRobot)
//      {
//         for (RobotSide robotSide : RobotSide.values)
//         {
//            for (LegJointName legJointName : legJointsForIntegratingAcceleration)
//               controllerRobotModel.getLegJoint(robotSide, legJointName).setIntegrateDesiredAccelerations(true);
//
//            for (ArmJointName armJointName : armJointsForIntegratingAcceleration)
//            {
//               OneDoFJoint armJoint = controllerRobotModel.getArmJoint(robotSide, armJointName);
//               if(armJoint!=null)
//                  armJoint.setIntegrateDesiredAccelerations(true);
//            }
//         }
//
//         for (SpineJointName spineJointName : spineJointsForIntegratingAcceleration)
//            controllerRobotModel.getSpineJoint(spineJointName).setIntegrateDesiredAccelerations(true);
//      }
   }

   @Override
   public void setForceSensorDataHolderForController(ForceSensorDataHolderReadOnly forceSensorDataHolderForEstimator)
   {
      if(drcOutputProcessor != null)
      {
         drcOutputProcessor.setForceSensorDataHolderForController(forceSensorDataHolderForEstimator);
      }
   }
}
