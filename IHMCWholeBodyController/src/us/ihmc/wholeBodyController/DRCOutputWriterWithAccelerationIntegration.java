package us.ihmc.wholeBodyController;

import java.util.ArrayList;
import java.util.LinkedHashMap;

import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;

public class DRCOutputWriterWithAccelerationIntegration implements DRCOutputWriter
{
   private final boolean runningOnRealRobot;
   private final DRCOutputWriter drcOutputWriter;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleYoVariable alphaDesiredVelocity = new DoubleYoVariable(
         "alphaDesiredVelocity",
         "Filter for velocity control in order to achieve acceleration control. Zero means compliant, but poor acceleration. One means stiff, but good acceleration tracking",
         registry);
   private final DoubleYoVariable alphaDesiredPosition = new DoubleYoVariable(
         "alphaDesiredPosition",
         "Filter for position control in order to achieve acceleration control. Zero means compliant, but poor acceleration. One means stiff, but good acceleration tracking",
         registry);

   private final DoubleYoVariable alphaArmDesiredVelocity = new DoubleYoVariable(
         "alphaArmDesiredVelocity",
         "Filter for velocity control in order to achieve acceleration control. Zero means compliant, but poor acceleration. One means stiff, but good acceleration tracking (Arms only)",
         registry);
   private final DoubleYoVariable alphaArmDesiredPosition = new DoubleYoVariable(
         "alphaArmDesiredPosition",
         "Filter for position control in order to achieve acceleration control. Zero means compliant, but poor acceleration. One means stiff, but good acceleration tracking (Arms only)",
         registry);

   private final DoubleYoVariable kVelJointTorque = new DoubleYoVariable("kVelJointTorque",
         "Gain for velocity control in order to achieve acceleration control using additional joint torque", registry);

   private final DoubleYoVariable kPosJointTorque = new DoubleYoVariable("kPosJointTorque",
         "Gain for position control in order to achieve acceleration control using additional joint torque", registry);

   private final DoubleYoVariable kVelArmJointTorque = new DoubleYoVariable("kVelArmJointTorque",
         "Gain for velocity control in order to achieve acceleration control using additional joint torque (Arms only)", registry);

   private final DoubleYoVariable kPosArmJointTorque = new DoubleYoVariable("kPosArmJointTorque",
         "Gain for position control in order to achieve acceleration control using additional joint torque (Arms only)", registry);

   private ArrayList<OneDoFJoint> oneDoFJoints;
   private LinkedHashMap<OneDoFJoint, DoubleYoVariable> alphaDesiredVelocityMap;
   private LinkedHashMap<OneDoFJoint, DoubleYoVariable> alphaDesiredPositionMap;
   private LinkedHashMap<OneDoFJoint, DoubleYoVariable> velocityTorqueMap;
   private LinkedHashMap<OneDoFJoint, DoubleYoVariable> positionTorqueMap;
   private LinkedHashMap<OneDoFJoint, DoubleYoVariable> kVelJointTorqueMap;
   private LinkedHashMap<OneDoFJoint, DoubleYoVariable> kPosJointTorqueMap;
   private LinkedHashMap<OneDoFJoint, DoubleYoVariable> desiredVelocities;
   private LinkedHashMap<OneDoFJoint, DoubleYoVariable> desiredPositions;

   private final double updateDT;
   private final boolean conservative;

   // List of the joints for which we do integrate the desired accelerations by default
   private final LegJointName[] legJointsForIntegratingAcceleration;
   private final ArmJointName[] armJointsForIntegratingAcceleration;
   private final SpineJointName[] spineJointsForIntegratingAcceleration;

   public DRCOutputWriterWithAccelerationIntegration(DRCOutputWriter drcOutputWriter, LegJointName[] legJointToIntegrate,
         ArmJointName[] armJointToIntegrate, SpineJointName[] spineJointToIntegrate, double updateDT, boolean runningOnRealRobot)
   {
      this(drcOutputWriter, legJointToIntegrate, armJointToIntegrate, spineJointToIntegrate, updateDT, runningOnRealRobot, false);
   }

   public DRCOutputWriterWithAccelerationIntegration(DRCOutputWriter drcOutputWriter, LegJointName[] legJointToIntegrate,
         ArmJointName[] armJointToIntegrate, SpineJointName[] spineJointToIntegrate, double updateDT, boolean runningOnRealRobot, boolean conservative)
   {
      this.runningOnRealRobot = runningOnRealRobot;
      this.conservative = conservative;
      this.drcOutputWriter = drcOutputWriter;
      this.updateDT = updateDT;
      registry.addChild(drcOutputWriter.getControllerYoVariableRegistry());

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

   public DRCOutputWriterWithAccelerationIntegration(DRCOutputWriter drcOutputWriter, double updateDT, boolean runningOnRealRobot)
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
      drcOutputWriter.initialize();
   }

   @Override
   public YoVariableRegistry getControllerYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public void writeAfterController(long timestamp)
   {
      for (int i = 0; i < oneDoFJoints.size(); i++)
      {
         OneDoFJoint oneDoFJoint = oneDoFJoints.get(i);
         DoubleYoVariable qd_d_joint = desiredVelocities.get(oneDoFJoint);
         DoubleYoVariable q_d_joint = desiredPositions.get(oneDoFJoint);

         boolean integrateDesiredAccelerations = oneDoFJoint.getIntegrateDesiredAccelerations();
         // Don't call the listener there, we want to be able to set those both from the controller and SCS.
         doAccelerationIntegrationMap.get(oneDoFJoint).set(integrateDesiredAccelerations, false);

         if (integrateDesiredAccelerations)
         {
            integrateAccelerationsToGetDesiredVelocities(oneDoFJoint, qd_d_joint, q_d_joint);
            oneDoFJoint.setQdDesired(qd_d_joint.getDoubleValue());
            oneDoFJoint.setKd(0.0);
            oneDoFJoint.setKp(0.0);

            double kVel = kVelJointTorqueMap.get(oneDoFJoint).getDoubleValue();
            double kPos = kPosJointTorqueMap.get(oneDoFJoint).getDoubleValue();
            double velocityTorque = kVel * (qd_d_joint.getDoubleValue() - oneDoFJoint.getQd());
            double positionTorque = kPos * (q_d_joint.getDoubleValue() - oneDoFJoint.getQ());

            velocityTorqueMap.get(oneDoFJoint).set(velocityTorque);
            positionTorqueMap.get(oneDoFJoint).set(positionTorque);

            oneDoFJoint.setTau(oneDoFJoint.getTau() + velocityTorque + positionTorque);
         }
         else
         {
            qd_d_joint.set(oneDoFJoint.getQd());
            q_d_joint.set(oneDoFJoint.getQ());
         }
      }

      drcOutputWriter.writeAfterController(timestamp);
   }

   private void integrateAccelerationsToGetDesiredVelocities(OneDoFJoint oneDoFJoint, DoubleYoVariable qd_d_joint, DoubleYoVariable q_d_joint)
   {
      double currentPosition = oneDoFJoint.getQ();
      double currentVelocity = oneDoFJoint.getQd();

      if (oneDoFJoint.getResetDesiredAccelerationIntegrator())
      {
         //       qd_d_joint.set(currentVelocity);
         qd_d_joint.set(0.0);
         q_d_joint.set(currentPosition);

         return;
      }

      double previousDesiredVelocity = qd_d_joint.getDoubleValue();
      double previousDesiredPosition = q_d_joint.getDoubleValue();

      double desiredAcceleration = oneDoFJoint.getQddDesired();
      double alphaVel = alphaDesiredVelocityMap.get(oneDoFJoint).getDoubleValue();
      double alphaPos = alphaDesiredPositionMap.get(oneDoFJoint).getDoubleValue();
      double desiredVelocity = doCleverIntegration(previousDesiredVelocity, desiredAcceleration, currentVelocity, alphaVel);
      double desiredPosition = doCleverIntegration(previousDesiredPosition, desiredVelocity, currentPosition, alphaPos);

      qd_d_joint.set(desiredVelocity);
      q_d_joint.set(desiredPosition);
   }

   private double doCleverIntegration(double previousDesiredValue, double desiredValueRate, double currentValue, double alpha)
   {
      return alpha * (previousDesiredValue + desiredValueRate * updateDT) + (1.0 - alpha) * currentValue;
   }

   private final LinkedHashMap<OneDoFJoint, BooleanYoVariable> doAccelerationIntegrationMap = new LinkedHashMap<>();

   @Override
   public void setFullRobotModel(FullHumanoidRobotModel controllerModel, RawJointSensorDataHolderMap rawJointSensorDataHolderMap)
   {
      drcOutputWriter.setFullRobotModel(controllerModel, rawJointSensorDataHolderMap);

      oneDoFJoints = new ArrayList<OneDoFJoint>();

      if (conservative)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            for (LegJointName jointName : legJointsForIntegratingAcceleration)
               oneDoFJoints.add(controllerModel.getLegJoint(robotSide, jointName));
            for (ArmJointName jointName : armJointsForIntegratingAcceleration)
               oneDoFJoints.add(controllerModel.getArmJoint(robotSide, jointName));
         }
         
         for (SpineJointName jointName : spineJointsForIntegratingAcceleration)
            oneDoFJoints.add(controllerModel.getSpineJoint(jointName));
      }
      else
      {
         controllerModel.getOneDoFJoints(oneDoFJoints);
      }

      ArrayList<OneDoFJoint> armOneDoFJoints = new ArrayList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody hand = controllerModel.getHand(robotSide);
         if (hand != null)
         {
            InverseDynamicsJoint[] armJoints = ScrewTools.createJointPath(controllerModel.getChest(), hand);
            OneDoFJoint[] filterArmJoints = ScrewTools.filterJoints(armJoints, OneDoFJoint.class);
            for (OneDoFJoint armJoint : filterArmJoints)
               armOneDoFJoints.add(armJoint);
         }
      }

      desiredVelocities = new LinkedHashMap<OneDoFJoint, DoubleYoVariable>();
      desiredPositions = new LinkedHashMap<OneDoFJoint, DoubleYoVariable>();

      alphaDesiredVelocityMap = new LinkedHashMap<>();
      alphaDesiredPositionMap = new LinkedHashMap<>();

      kVelJointTorqueMap = new LinkedHashMap<OneDoFJoint, DoubleYoVariable>();
      kPosJointTorqueMap = new LinkedHashMap<OneDoFJoint, DoubleYoVariable>();

      velocityTorqueMap = new LinkedHashMap<>();
      positionTorqueMap = new LinkedHashMap<>();

      for (int i = 0; i < oneDoFJoints.size(); i++)
      {
         final OneDoFJoint oneDoFJoint = oneDoFJoints.get(i);
         final BooleanYoVariable doAccelerationIntegration = new BooleanYoVariable("doAccelerationIntegration_" + oneDoFJoint.getName(), registry);
         DoubleYoVariable desiredVelocity = new DoubleYoVariable("qd_d_" + oneDoFJoint.getName(), registry);
         DoubleYoVariable desiredPosition = new DoubleYoVariable("q_d_" + oneDoFJoint.getName(), registry);

         desiredVelocities.put(oneDoFJoint, desiredVelocity);
         desiredPositions.put(oneDoFJoint, desiredPosition);

         DoubleYoVariable velocityTorque = new DoubleYoVariable("tau_vel_" + oneDoFJoint.getName(), registry);
         DoubleYoVariable positionTorque = new DoubleYoVariable("tau_pos_" + oneDoFJoint.getName(), registry);

         velocityTorqueMap.put(oneDoFJoint, velocityTorque);
         positionTorqueMap.put(oneDoFJoint, positionTorque);

         if (armOneDoFJoints.contains(oneDoFJoint))
         {
            alphaDesiredVelocityMap.put(oneDoFJoint, alphaArmDesiredVelocity);
            alphaDesiredPositionMap.put(oneDoFJoint, alphaArmDesiredPosition);
            kVelJointTorqueMap.put(oneDoFJoint, kVelArmJointTorque);
            kPosJointTorqueMap.put(oneDoFJoint, kPosArmJointTorque);
         }
         else
         {
            alphaDesiredVelocityMap.put(oneDoFJoint, alphaDesiredVelocity);
            alphaDesiredPositionMap.put(oneDoFJoint, alphaDesiredPosition);
            kVelJointTorqueMap.put(oneDoFJoint, kVelJointTorque);
            kPosJointTorqueMap.put(oneDoFJoint, kPosJointTorque);
         }

         // We want to be able to set those both from the controller and SCS.
         doAccelerationIntegration.addVariableChangedListener(new VariableChangedListener()
         {
            @Override
            public void variableChanged(YoVariable<?> v)
            {
               oneDoFJoint.setIntegrateDesiredAccelerations(doAccelerationIntegration.getBooleanValue());
            }
         });

         doAccelerationIntegration.set(false);
         doAccelerationIntegration.notifyVariableChangedListeners();

         doAccelerationIntegrationMap.put(oneDoFJoint, doAccelerationIntegration);
      }

      if (runningOnRealRobot)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            for (LegJointName legJointName : legJointsForIntegratingAcceleration)
               controllerModel.getLegJoint(robotSide, legJointName).setIntegrateDesiredAccelerations(true);

            for (ArmJointName armJointName : armJointsForIntegratingAcceleration)
            {
               OneDoFJoint armJoint = controllerModel.getArmJoint(robotSide, armJointName);
               if(armJoint!=null)
                  armJoint.setIntegrateDesiredAccelerations(true);
            }
         }

         for (SpineJointName spineJointName : spineJointsForIntegratingAcceleration)
            controllerModel.getSpineJoint(spineJointName).setIntegrateDesiredAccelerations(true);
      }
   }

   @Override
   public void setForceSensorDataHolderForController(ForceSensorDataHolderReadOnly forceSensorDataHolderForEstimator)
   {
      drcOutputWriter.setForceSensorDataHolderForController(forceSensorDataHolderForEstimator);
   }
}
