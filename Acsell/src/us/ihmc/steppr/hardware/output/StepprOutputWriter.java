package us.ihmc.steppr.hardware.output;

import java.util.EnumMap;

import us.ihmc.acsell.hardware.command.AcsellJointCommand;
import us.ihmc.acsell.hardware.command.UDPAcsellOutputWriter;
import us.ihmc.acsell.springs.HystereticSpringCalculator;
import us.ihmc.acsell.springs.HystereticSpringProperties;
import us.ihmc.acsell.springs.LinearSpringCalculator;
import us.ihmc.acsell.springs.SpringCalculator;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.controllers.ControllerFailureListener;
import us.ihmc.robotics.controllers.ControllerStateChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolder;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.steppr.hardware.StepprJoint;
import us.ihmc.steppr.hardware.StepprUtil;
import us.ihmc.steppr.hardware.command.StepprCommand;
import us.ihmc.steppr.hardware.configuration.StepprLeftAnkleSpringProperties;
import us.ihmc.steppr.hardware.configuration.StepprLeftHipXSpringProperties;
import us.ihmc.steppr.hardware.configuration.StepprNetworkParameters;
import us.ihmc.steppr.hardware.configuration.StepprRightAnkleSpringProperties;
import us.ihmc.steppr.hardware.configuration.StepprRightHipXSpringProperties;
import us.ihmc.steppr.hardware.controllers.StepprStandPrep;
import us.ihmc.wholeBodyController.DRCOutputWriter;

public class StepprOutputWriter implements DRCOutputWriter, ControllerStateChangedListener,  ControllerFailureListener
{
   boolean USE_LEFT_HIP_X_SPRING = true;
   boolean USE_RIGHT_HIP_X_SPRING = true;
   boolean USE_LEFT_ANKLE_SPRING = true;
   boolean USE_RIGHT_ANKLE_SPRING = true;

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

   private final UDPAcsellOutputWriter outputWriter;
   private final EnumMap<StepprJoint, DoubleYoVariable> yoTauSpringCorrection = new EnumMap<StepprJoint, DoubleYoVariable>(StepprJoint.class);
   private final EnumMap<StepprJoint, DoubleYoVariable> yoTauTotal = new EnumMap<StepprJoint, DoubleYoVariable>(StepprJoint.class);
   private final EnumMap<StepprJoint, DoubleYoVariable> yoAngleSpring = new EnumMap<StepprJoint, DoubleYoVariable>(StepprJoint.class);
   private final EnumMap<StepprJoint, DoubleYoVariable> yoReflectedMotorInertia = new EnumMap<StepprJoint, DoubleYoVariable>(StepprJoint.class);
   private final EnumMap<StepprJoint, DoubleYoVariable> yoTauInertiaViz = new EnumMap<StepprJoint, DoubleYoVariable>(StepprJoint.class);
   private final EnumMap<StepprJoint, DoubleYoVariable> yoMotorDamping = new EnumMap<StepprJoint, DoubleYoVariable>(StepprJoint.class);
   private final EnumMap<StepprJoint, DoubleYoVariable> desiredQddFeedForwardGain = new EnumMap<StepprJoint, DoubleYoVariable>(StepprJoint.class);
   private final EnumMap<StepprJoint, DoubleYoVariable> desiredJointQ = new EnumMap<StepprJoint, DoubleYoVariable>(StepprJoint.class);
   private final EnumYoVariable<WalkingStateEnum>  yoWalkingState = new EnumYoVariable<WalkingStateEnum>("sow_walkingState", registry, WalkingStateEnum.class);

   private final DoubleYoVariable masterMotorDamping = new DoubleYoVariable("masterMotorDamping", registry);

   private final HystereticSpringProperties leftHipXSpringProperties = new StepprLeftHipXSpringProperties();
   private final HystereticSpringProperties rightHipXSpringProperties = new StepprRightHipXSpringProperties();
   private final SpringCalculator leftHipXSpringCalculator;
   private final SpringCalculator rightHipXSpringCalculator;
   private final HystereticSpringProperties leftAnkleSpringProperties = new StepprLeftAnkleSpringProperties();
   private final HystereticSpringProperties rightAnkleSpringProperties = new StepprRightAnkleSpringProperties();
   private final SpringCalculator leftAnkleSpringCalculator;
   private final SpringCalculator rightAnkleSpringCalculator;


   enum JointControlMode
   {
      TORQUE_CONTROL, POSITION_CONTROL;
   };

   private final EnumMap<StepprJoint, EnumYoVariable<JointControlMode>> jointControlMode = new EnumMap<StepprJoint, EnumYoVariable<JointControlMode>>(
         StepprJoint.class);

   private WalkingStateEnum currentWalkingState;

   public StepprOutputWriter(DRCRobotModel robotModel)
   {

      FullRobotModel standPrepFullRobotModel = robotModel.createFullRobotModel();
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
         desiredJointQ.put(joint, new DoubleYoVariable(joint.getSdfName()+"_Q_desired",registry));
      }

      yoTauSpringCorrection.put(StepprJoint.LEFT_HIP_X, new DoubleYoVariable(StepprJoint.LEFT_HIP_X.getSdfName() + "_tauSpringCorrection", registry));
      yoTauSpringCorrection.put(StepprJoint.RIGHT_HIP_X, new DoubleYoVariable(StepprJoint.RIGHT_HIP_X.getSdfName() + "_tauSpringCorrection", registry));
      yoTauSpringCorrection.put(StepprJoint.LEFT_ANKLE_Y, new DoubleYoVariable(StepprJoint.LEFT_ANKLE_Y.getSdfName() + "_tauSpringCorrection", registry));
      yoTauSpringCorrection.put(StepprJoint.RIGHT_ANKLE_Y, new DoubleYoVariable(StepprJoint.RIGHT_ANKLE_Y.getSdfName() + "_tauSpringCorrection", registry));
      yoTauTotal.put(StepprJoint.LEFT_HIP_X, new DoubleYoVariable(StepprJoint.LEFT_HIP_X.getSdfName() + "_tauSpringTotalDesired", registry));
      yoTauTotal.put(StepprJoint.RIGHT_HIP_X, new DoubleYoVariable(StepprJoint.RIGHT_HIP_X.getSdfName() + "_tauSpringTotalDesired", registry));
      yoTauTotal.put(StepprJoint.LEFT_ANKLE_Y, new DoubleYoVariable(StepprJoint.LEFT_ANKLE_Y.getSdfName() + "_tauSpringTotalDesired", registry));
      yoTauTotal.put(StepprJoint.RIGHT_ANKLE_Y, new DoubleYoVariable(StepprJoint.RIGHT_ANKLE_Y.getSdfName() + "_tauSpringTotalDesired", registry));
      yoAngleSpring.put(StepprJoint.LEFT_HIP_X, new DoubleYoVariable(StepprJoint.LEFT_HIP_X.getSdfName() + "_q_Spring", registry));
      yoAngleSpring.put(StepprJoint.RIGHT_HIP_X, new DoubleYoVariable(StepprJoint.RIGHT_HIP_X.getSdfName() + "_q_Spring", registry));
      yoAngleSpring.put(StepprJoint.LEFT_ANKLE_Y, new DoubleYoVariable(StepprJoint.LEFT_ANKLE_Y.getSdfName() + "_q_Spring", registry));
      yoAngleSpring.put(StepprJoint.RIGHT_ANKLE_Y, new DoubleYoVariable(StepprJoint.RIGHT_ANKLE_Y.getSdfName() + "_q_Spring", registry));

      leftHipXSpringCalculator = new HystereticSpringCalculator(leftHipXSpringProperties,StepprJoint.LEFT_HIP_X.getSdfName(),registry);
      rightHipXSpringCalculator = new HystereticSpringCalculator(rightHipXSpringProperties,StepprJoint.RIGHT_HIP_X.getSdfName(),registry);
      leftAnkleSpringCalculator = new LinearSpringCalculator(leftAnkleSpringProperties);
      rightAnkleSpringCalculator = new LinearSpringCalculator(rightAnkleSpringProperties);

      initializeJointControlMode(robotModel.getWalkingControllerParameters().getJointsToIgnoreInController());
      initializeMotorDamping();
      initializeFeedForwardTorqueFromDesiredAcceleration();

      outputWriter = new UDPAcsellOutputWriter(command);

      standPrep.setFullRobotModel(standPrepFullRobotModel);
      registry.addChild(standPrep.getYoVariableRegistry());

      outputWriter.connect(new StepprNetworkParameters());

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
         if(!enableOutput.getBooleanValue())
         {
            standPrep.initialize(timestamp);
            command.disableActuators();
            controlRatio.set(0.0);
            outputEnabled = false;
         }
         computeOutputCommand(timestamp);

      }

      outputWriter.write();

   }

   private void computeOutputCommand(long timestamp)
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

            AcsellJointCommand jointCommand = command.getAcsellJointCommand(joint);

            desiredJointQ.get(joint).set(wholeBodyControlJoint.getqDesired());

            tauControllerOutput.get(joint).set(tau);
            double tauSpring = 0;
            if (yoTauSpringCorrection.get(joint) != null)
            {
               tauSpring = calcSpringTorque(joint, rawSensorData.getQ_raw());
               yoTauSpringCorrection.get(joint).set(tauSpring);
               yoTauTotal.get(joint).set(tau);
            }
            jointCommand.setTauDesired(tau - tauSpring, wholeBodyControlJoint.getQddDesired(), rawSensorData);

            jointCommand.setDamping(kd);

         }
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
     if (standPrep.getStandPrepState() != StepprStandPrep.StandPrepState.EXECUTE)
       return 0.0;
     switch (joint)
     {
      case LEFT_HIP_X:
      {
        yoAngleSpring.get(joint).set(q);
        leftHipXSpringCalculator.update(q);
        return USE_LEFT_HIP_X_SPRING ? leftHipXSpringCalculator.getSpringForce() : 0.0;
      }
      case RIGHT_HIP_X:
      {
        yoAngleSpring.get(joint).set(q);
        rightHipXSpringCalculator.update(q);
        return USE_RIGHT_HIP_X_SPRING ? rightHipXSpringCalculator.getSpringForce() : 0.0;
      }
      case LEFT_ANKLE_Y:
      {
        yoAngleSpring.get(joint).set(q);
        leftAnkleSpringCalculator.update(q);
        return (USE_LEFT_ANKLE_SPRING && (currentWalkingState != WalkingStateEnum.WALKING_RIGHT_SUPPORT)) ?
              leftAnkleSpringCalculator.getSpringForce() :
              leftAnkleSpringCalculator.getSpringForce()*0.75;
      }
      case RIGHT_ANKLE_Y:
      {
        yoAngleSpring.get(joint).set(q);
        rightAnkleSpringCalculator.update(q);
        return (USE_RIGHT_ANKLE_SPRING && (currentWalkingState != WalkingStateEnum.WALKING_LEFT_SUPPORT)) ?
              rightAnkleSpringCalculator.getSpringForce() :
              rightAnkleSpringCalculator.getSpringForce()*0.75;
      }
      default:
         return 0.0;
     }
   }

   @Override
   public void setFullRobotModel(FullHumanoidRobotModel controllerModel, RawJointSensorDataHolderMap rawJointSensorDataHolderMap)
   {
      wholeBodyControlJoints = StepprUtil.createJointMap(controllerModel.getOneDoFJoints());
      this.rawJointSensorDataHolderMap = rawJointSensorDataHolderMap;
   }

   @Override
   public void setForceSensorDataHolderForController(ForceSensorDataHolderReadOnly forceSensorDataHolderForController)
   {

   }

   @Override
   public YoVariableRegistry getControllerYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public void controllerStateHasChanged(Enum<?> oldState, Enum<?> newState)
   {
      if(newState instanceof WalkingStateEnum)
      {
         currentWalkingState = (WalkingStateEnum)newState;
         yoWalkingState.set(currentWalkingState);
      }
   }

   @Override
   public void controllerFailed(FrameVector2d framevector)
   {
      enableOutput.set(false);
      yoWalkingState.set(WalkingStateEnum.TO_STANDING);
      //((EnumYoVariable<WalkingState>)registry.getVariable("WalkingHighLevelHumanoidController", "walkingState")).setValue(yoWalkingState,true);
      //((BooleanYoVariable)registry.getVariable("DesiredFootstepCalculatorFootstepProviderWrapper","walk")).set(false);
   }


}