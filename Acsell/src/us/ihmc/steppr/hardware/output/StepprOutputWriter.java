package us.ihmc.steppr.hardware.output;

import java.util.EnumMap;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolder;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.steppr.hardware.StepprJoint;
import us.ihmc.steppr.hardware.StepprUtil;
import us.ihmc.steppr.hardware.command.StepprCommand;
import us.ihmc.steppr.hardware.command.StepprJointCommand;
import us.ihmc.steppr.hardware.command.UDPStepprOutputWriter;
import us.ihmc.steppr.hardware.controllers.StepprStandPrep;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.wholeBodyController.DRCOutputWriter;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;

public class StepprOutputWriter implements DRCOutputWriter
{
   private final YoVariableRegistry registry = new YoVariableRegistry("StepprOutputWriter");

   private final StepprCommand command = new StepprCommand(registry);

   private EnumMap<StepprJoint, OneDoFJoint> wholeBodyControlJoints;
   private EnumMap<StepprJoint, DoubleYoVariable> tauControllerOutput;
   private final EnumMap<StepprJoint, OneDoFJoint> standPrepJoints;
   private final StepprStandPrep standPrep = new StepprStandPrep();

   private final DoubleYoVariable controlRatio = new DoubleYoVariable("controlRatio", registry);

   private boolean outputEnabled;
   private final BooleanYoVariable enableOutput = new BooleanYoVariable("enableOutput", registry);

   private RawJointSensorDataHolderMap rawJointSensorDataHolderMap;

   private final UDPStepprOutputWriter outputWriter;
   private final EnumMap<StepprJoint, DoubleYoVariable> yoTauSpring = new EnumMap<StepprJoint, DoubleYoVariable>(StepprJoint.class);
   private final EnumMap<StepprJoint, DoubleYoVariable> yoReflectedMotorInertia = new EnumMap<StepprJoint, DoubleYoVariable>(StepprJoint.class);
   private final EnumMap<StepprJoint, DoubleYoVariable> yoTauInertiaViz = new EnumMap<StepprJoint, DoubleYoVariable>(StepprJoint.class);
   private final EnumMap<StepprJoint, DoubleYoVariable> yoMotorDamping = new EnumMap<StepprJoint, DoubleYoVariable>(StepprJoint.class);
   private final EnumMap<StepprJoint, DoubleYoVariable> desiredQddFeedForwardGain = new EnumMap<StepprJoint, DoubleYoVariable>(StepprJoint.class);
   
   private final DoubleYoVariable masterMotorDamping = new DoubleYoVariable("masterMotorDamping", registry);

   enum JointControlMode
   {
      TORQUE_CONTROL, POSITION_CONTROL;
   };

   private final EnumMap<StepprJoint, EnumYoVariable<JointControlMode>> jointControlMode = new EnumMap<StepprJoint, EnumYoVariable<JointControlMode>>(
         StepprJoint.class);

   public StepprOutputWriter(DRCRobotModel robotModel)
   {

      SDFFullRobotModel standPrepFullRobotModel = robotModel.createFullRobotModel();
      standPrepJoints = StepprUtil.createJointMap(standPrepFullRobotModel.getOneDoFJoints());

      tauControllerOutput = new EnumMap<StepprJoint, DoubleYoVariable>(StepprJoint.class);
      for (StepprJoint joint : StepprJoint.values)
      {
         tauControllerOutput.put(joint, new DoubleYoVariable(joint.getSdfName() + "tauControllerOutput", registry));
         yoMotorDamping.put(joint, new DoubleYoVariable(joint.getSdfName() + "motorDamping", registry));
         
         
         DoubleYoVariable inertia = new DoubleYoVariable(joint.getSdfName()+"ReflectedMotorInertia", registry);
         inertia.set(joint.getActuators()[0].getMotorInertia()*joint.getRatio()*joint.getRatio()); //hacky
         yoReflectedMotorInertia.put(joint, inertia);  
         yoTauInertiaViz.put(joint, new DoubleYoVariable(joint.getSdfName()+"TauInertia", registry));
         desiredQddFeedForwardGain.put(joint, new DoubleYoVariable(joint.getSdfName()+"QddFeedForwardGain", registry));         
      }

      yoTauSpring.put(StepprJoint.LEFT_HIP_X, new DoubleYoVariable(StepprJoint.LEFT_HIP_X.getSdfName() + "_tauSpringCorrection", registry));
      yoTauSpring.put(StepprJoint.RIGHT_HIP_X, new DoubleYoVariable(StepprJoint.RIGHT_HIP_X.getSdfName() + "_tauSpringCorrection", registry));

      initializeJointControlMode(robotModel.getWalkingControllerParameters().getJointsToIgnoreInController());
      initializeMotorDamping();
      initializeFeedForwardTorqueFromDesiredAcceleration();

      outputWriter = new UDPStepprOutputWriter(command);

      standPrep.setFullRobotModel(standPrepFullRobotModel);
      registry.addChild(standPrep.getYoVariableRegistry());

      outputWriter.connect();

   }

   private void initializeMotorDamping()
   {
      yoMotorDamping.get(StepprJoint.LEFT_ANKLE_X).set(5.0);
      yoMotorDamping.get(StepprJoint.LEFT_ANKLE_Y).set(5.0);
      yoMotorDamping.get(StepprJoint.LEFT_KNEE_Y).set(1.0);
      yoMotorDamping.get(StepprJoint.LEFT_HIP_Y).set(1.0);
      yoMotorDamping.get(StepprJoint.LEFT_HIP_X).set(10.0);
      yoMotorDamping.get(StepprJoint.RIGHT_ANKLE_X).set(5.0);
      yoMotorDamping.get(StepprJoint.RIGHT_ANKLE_Y).set(5.0);
      yoMotorDamping.get(StepprJoint.RIGHT_KNEE_Y).set(1.0);
      yoMotorDamping.get(StepprJoint.RIGHT_HIP_Y).set(1.0);
      yoMotorDamping.get(StepprJoint.RIGHT_HIP_X).set(10.0);
      masterMotorDamping.set(1.0);
      
   }
   
   private void initializeFeedForwardTorqueFromDesiredAcceleration()
   {
      desiredQddFeedForwardGain.get(StepprJoint.LEFT_ANKLE_Y).set(0.5);
      desiredQddFeedForwardGain.get(StepprJoint.RIGHT_ANKLE_Y).set(0.5);
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void writeAfterController(long timestamp)
   {
      if (!outputEnabled)
      {
         if (enableOutput.getBooleanValue())
         {
            standPrep.initialize(timestamp);
            command.enableActuators();
            controlRatio.set(0.0);
            outputEnabled = true;
         }
      }
      else
      {

         /*
          * StandPrep
          */
         for (StepprJoint joint : StepprJoint.values)
         {
            OneDoFJoint wholeBodyControlJoint = wholeBodyControlJoints.get(joint);
            OneDoFJoint standPrepJoint = standPrepJoints.get(joint);
            RawJointSensorDataHolder rawSensorData = rawJointSensorDataHolderMap.get(wholeBodyControlJoint);

            standPrepJoint.setQ(rawSensorData.getQ_raw());
            standPrepJoint.setQd(rawSensorData.getQd_raw());
         }

         standPrep.doControl(timestamp);

         /*
          * IHMC Control
          */
         for (StepprJoint joint : StepprJoint.values)
         {
            OneDoFJoint wholeBodyControlJoint = wholeBodyControlJoints.get(joint);
            OneDoFJoint standPrepJoint = standPrepJoints.get(joint);
            RawJointSensorDataHolder rawSensorData = rawJointSensorDataHolderMap.get(wholeBodyControlJoint);

            double controlRatio = getControlRatioByJointControlMode(joint);
            
            double desiredAcceleration = wholeBodyControlJoint.getQddDesired();
            double motorReflectedInertia = yoReflectedMotorInertia.get(joint).getDoubleValue();
            double motorInertiaTorque = motorReflectedInertia * desiredAcceleration;
            double desiredQddFeedForwardTorque = desiredQddFeedForwardGain.get(joint).getDoubleValue()*desiredAcceleration;
            yoTauInertiaViz.get(joint).set(motorInertiaTorque);

            double kd = (wholeBodyControlJoint.getKd()+masterMotorDamping.getDoubleValue()*yoMotorDamping.get(joint).getDoubleValue()) * controlRatio + standPrepJoint.getKd() * (1.0 - controlRatio);
            double tau = (wholeBodyControlJoint.getTau()+ motorInertiaTorque+desiredQddFeedForwardTorque) * controlRatio + standPrepJoint.getTau() * (1.0 - controlRatio);

            StepprJointCommand jointCommand = command.getStepprJointCommand(joint);



            tauControllerOutput.get(joint).set(tau);
            boolean USE_SPRING = false;
            if(USE_SPRING)
            {
               double tauSpring = 0;
               if (yoTauSpring.get(joint) != null)
               {
                  tauSpring = calcSpringTorque(joint, rawSensorData.getQ_raw());
                  yoTauSpring.get(joint).set(tauSpring);
               } 
               jointCommand.setTauDesired(tau - tauSpring, wholeBodyControlJoint.getQddDesired(), rawSensorData); 
            }
            else
            {
               jointCommand.setTauDesired(tau, wholeBodyControlJoint.getQddDesired(), rawSensorData); 
            }
            jointCommand.setDamping(kd);

         }
      }

      outputWriter.write();

   }

   private void initializeJointControlMode(String[] jointNameToIgnore)
   {
      for (StepprJoint joint : StepprJoint.values)
      {
         EnumYoVariable<JointControlMode> controlMode = new EnumYoVariable<StepprOutputWriter.JointControlMode>(joint.getSdfName()
               + JointControlMode.class.getSimpleName(), registry, JointControlMode.class, false);

         controlMode.set(JointControlMode.TORQUE_CONTROL);
         for (String jointName : jointNameToIgnore)
         {
            if (joint.getSdfName().equals(jointName))
            {
               controlMode.set(JointControlMode.POSITION_CONTROL);
               break;
            }
         }
         jointControlMode.put(joint, controlMode);
      }
   }

   private double getControlRatioByJointControlMode(StepprJoint joint)
   {
      double ratio;
      switch (jointControlMode.get(joint).getEnumValue())
      {
      case POSITION_CONTROL:
         ratio = 0.0;
         break;
      case TORQUE_CONTROL:
         ratio = controlRatio.getDoubleValue();
         break;
      default:
         jointControlMode.get(joint).set(JointControlMode.POSITION_CONTROL);
         ratio = 0.0;
      }
      return ratio;
   }

   private double calcSpringTorque(StepprJoint joint, double q)
   {
      final double springConstant = 350; //400.0;
      final double springEngageJointAngle = 0.085; //0.05;
      switch (joint)
      {
      case LEFT_HIP_X:
         return springConstant * Math.max(-springEngageJointAngle - q, 0.0);
      case RIGHT_HIP_X:
         return -springConstant * Math.max(q - springEngageJointAngle, 0.0);
      default:
         return 0;
      }
   }

   @Override
   public void setFullRobotModel(SDFFullRobotModel controllerModel, RawJointSensorDataHolderMap rawJointSensorDataHolderMap)
   {
      wholeBodyControlJoints = StepprUtil.createJointMap(controllerModel.getOneDoFJoints());
      this.rawJointSensorDataHolderMap = rawJointSensorDataHolderMap;
   }

   @Override
   public void setForceSensorDataHolderForController(ForceSensorDataHolder forceSensorDataHolderForController)
   {

   }

   @Override
   public YoVariableRegistry getControllerYoVariableRegistry()
   {
      return registry;
   }

}
