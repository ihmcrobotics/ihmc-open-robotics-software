package us.ihmc.wanderer.hardware.output;

import java.util.EnumMap;

import us.ihmc.acsell.hardware.command.AcsellJointCommand;
import us.ihmc.acsell.hardware.command.UDPAcsellOutputWriter;
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
import us.ihmc.wanderer.hardware.WandererJoint;
import us.ihmc.wanderer.hardware.WandererUtil;
import us.ihmc.wanderer.hardware.command.WandererCommand;
import us.ihmc.wanderer.hardware.configuration.WandererLeftAnkleSpringProperties;
import us.ihmc.wanderer.hardware.configuration.WandererLeftHipXSpringProperties;
import us.ihmc.wanderer.hardware.configuration.WandererNetworkParameters;
import us.ihmc.wanderer.hardware.configuration.WandererRightAnkleSpringProperties;
import us.ihmc.wanderer.hardware.configuration.WandererRightHipXSpringProperties;
import us.ihmc.wanderer.hardware.controllers.WandererStandPrep;
import us.ihmc.wholeBodyController.DRCOutputWriter;

public class WandererOutputWriter implements DRCOutputWriter, ControllerStateChangedListener,  ControllerFailureListener
{
   boolean USE_LEFT_HIP_X_SPRING = true;
   boolean USE_RIGHT_HIP_X_SPRING = true;
   boolean USE_LEFT_ANKLE_SPRING = true;
   boolean USE_RIGHT_ANKLE_SPRING = true;

   private final YoVariableRegistry registry = new YoVariableRegistry("WandererOutputWriter");

   private final WandererCommand command = new WandererCommand(registry);

   private EnumMap<WandererJoint, OneDoFJoint> wholeBodyControlJoints;
   private EnumMap<WandererJoint, DoubleYoVariable> tauControllerOutput;
   private final EnumMap<WandererJoint, OneDoFJoint> standPrepJoints;
   private final WandererStandPrep standPrep = new WandererStandPrep();

   private final DoubleYoVariable controlRatio = new DoubleYoVariable("controlRatio", registry);

   private boolean outputEnabled;
   private final BooleanYoVariable enableOutput = new BooleanYoVariable("enableOutput", registry);

   private RawJointSensorDataHolderMap rawJointSensorDataHolderMap;

   private final UDPAcsellOutputWriter outputWriter;
   private final EnumMap<WandererJoint, DoubleYoVariable> yoTauSpringCorrection = new EnumMap<WandererJoint, DoubleYoVariable>(WandererJoint.class);
   private final EnumMap<WandererJoint, DoubleYoVariable> yoTauTotal = new EnumMap<WandererJoint, DoubleYoVariable>(WandererJoint.class);
   private final EnumMap<WandererJoint, DoubleYoVariable> yoAngleSpring = new EnumMap<WandererJoint, DoubleYoVariable>(WandererJoint.class);
   private final EnumMap<WandererJoint, DoubleYoVariable> yoReflectedMotorInertia = new EnumMap<WandererJoint, DoubleYoVariable>(WandererJoint.class);
   private final EnumMap<WandererJoint, DoubleYoVariable> yoTauInertiaViz = new EnumMap<WandererJoint, DoubleYoVariable>(WandererJoint.class);
   private final EnumMap<WandererJoint, DoubleYoVariable> yoMotorDamping = new EnumMap<WandererJoint, DoubleYoVariable>(WandererJoint.class);
   private final EnumMap<WandererJoint, DoubleYoVariable> desiredQddFeedForwardGain = new EnumMap<WandererJoint, DoubleYoVariable>(WandererJoint.class);
   private final EnumMap<WandererJoint, DoubleYoVariable> desiredJointQ = new EnumMap<WandererJoint, DoubleYoVariable>(WandererJoint.class);
   private final EnumYoVariable<WalkingStateEnum>  yoWalkingState = new EnumYoVariable<WalkingStateEnum>("sow_walkingState", registry, WalkingStateEnum.class);

   private final DoubleYoVariable masterMotorDamping = new DoubleYoVariable("masterMotorDamping", registry);

   private final HystereticSpringProperties leftHipXSpringProperties = new WandererLeftHipXSpringProperties();
   private final HystereticSpringProperties rightHipXSpringProperties = new WandererRightHipXSpringProperties();
   private final SpringCalculator leftHipXSpringCalculator;
   private final SpringCalculator rightHipXSpringCalculator;
   private final HystereticSpringProperties leftAnkleSpringProperties = new WandererLeftAnkleSpringProperties();
   private final HystereticSpringProperties rightAnkleSpringProperties = new WandererRightAnkleSpringProperties();
   private final SpringCalculator leftAnkleSpringCalculator;
   private final SpringCalculator rightAnkleSpringCalculator;


   enum JointControlMode
   {
      TORQUE_CONTROL, POSITION_CONTROL;
   };

   private final EnumMap<WandererJoint, EnumYoVariable<JointControlMode>> jointControlMode = new EnumMap<WandererJoint, EnumYoVariable<JointControlMode>>(
         WandererJoint.class);

   private WalkingStateEnum currentWalkingState;

   public WandererOutputWriter(DRCRobotModel robotModel)
   {

      FullRobotModel standPrepFullRobotModel = robotModel.createFullRobotModel();
      standPrepJoints = WandererUtil.createJointMap(standPrepFullRobotModel.getOneDoFJoints());

      tauControllerOutput = new EnumMap<WandererJoint, DoubleYoVariable>(WandererJoint.class);
      for (WandererJoint joint : WandererJoint.values)
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

      yoTauSpringCorrection.put(WandererJoint.LEFT_HIP_X, new DoubleYoVariable(WandererJoint.LEFT_HIP_X.getSdfName() + "_tauSpringCorrection", registry));
      yoTauSpringCorrection.put(WandererJoint.RIGHT_HIP_X, new DoubleYoVariable(WandererJoint.RIGHT_HIP_X.getSdfName() + "_tauSpringCorrection", registry));
      yoTauSpringCorrection.put(WandererJoint.LEFT_ANKLE_Y, new DoubleYoVariable(WandererJoint.LEFT_ANKLE_Y.getSdfName() + "_tauSpringCorrection", registry));
      yoTauSpringCorrection.put(WandererJoint.RIGHT_ANKLE_Y, new DoubleYoVariable(WandererJoint.RIGHT_ANKLE_Y.getSdfName() + "_tauSpringCorrection", registry));
      yoTauTotal.put(WandererJoint.LEFT_HIP_X, new DoubleYoVariable(WandererJoint.LEFT_HIP_X.getSdfName() + "_tauSpringTotalDesired", registry));
      yoTauTotal.put(WandererJoint.RIGHT_HIP_X, new DoubleYoVariable(WandererJoint.RIGHT_HIP_X.getSdfName() + "_tauSpringTotalDesired", registry));
      yoTauTotal.put(WandererJoint.LEFT_ANKLE_Y, new DoubleYoVariable(WandererJoint.LEFT_ANKLE_Y.getSdfName() + "_tauSpringTotalDesired", registry));
      yoTauTotal.put(WandererJoint.RIGHT_ANKLE_Y, new DoubleYoVariable(WandererJoint.RIGHT_ANKLE_Y.getSdfName() + "_tauSpringTotalDesired", registry));
      yoAngleSpring.put(WandererJoint.LEFT_HIP_X, new DoubleYoVariable(WandererJoint.LEFT_HIP_X.getSdfName() + "_q_Spring", registry));
      yoAngleSpring.put(WandererJoint.RIGHT_HIP_X, new DoubleYoVariable(WandererJoint.RIGHT_HIP_X.getSdfName() + "_q_Spring", registry));
      yoAngleSpring.put(WandererJoint.LEFT_ANKLE_Y, new DoubleYoVariable(WandererJoint.LEFT_ANKLE_Y.getSdfName() + "_q_Spring", registry));
      yoAngleSpring.put(WandererJoint.RIGHT_ANKLE_Y, new DoubleYoVariable(WandererJoint.RIGHT_ANKLE_Y.getSdfName() + "_q_Spring", registry));

      leftHipXSpringCalculator = new LinearSpringCalculator(leftHipXSpringProperties);
      rightHipXSpringCalculator = new LinearSpringCalculator(rightHipXSpringProperties);
      leftAnkleSpringCalculator = new LinearSpringCalculator(leftAnkleSpringProperties);
      rightAnkleSpringCalculator = new LinearSpringCalculator(rightAnkleSpringProperties);

      initializeJointControlMode(robotModel.getWalkingControllerParameters().getJointsToIgnoreInController());
      initializeMotorDamping();
      initializeFeedForwardTorqueFromDesiredAcceleration();

      outputWriter = new UDPAcsellOutputWriter(command);

      standPrep.setFullRobotModel(standPrepFullRobotModel);
      registry.addChild(standPrep.getYoVariableRegistry());

      outputWriter.connect(new WandererNetworkParameters());

   }

   private void initializeMotorDamping()
   {
      //TODO: Update For Wanderer
      yoMotorDamping.get(WandererJoint.LEFT_ANKLE_X).set(5.0*1.152);//0.14
      yoMotorDamping.get(WandererJoint.LEFT_ANKLE_Y).set(5.0*1.152);
      yoMotorDamping.get(WandererJoint.LEFT_KNEE_Y).set(1.0*0.612);//0.01
      yoMotorDamping.get(WandererJoint.LEFT_HIP_Y).set(1.0*0.612);//0.01
      yoMotorDamping.get(WandererJoint.LEFT_HIP_X).set(10.0*2.286);//0.1
      yoMotorDamping.get(WandererJoint.RIGHT_ANKLE_X).set(5.0*1.152);
      yoMotorDamping.get(WandererJoint.RIGHT_ANKLE_Y).set(5.0*1.152);
      yoMotorDamping.get(WandererJoint.RIGHT_KNEE_Y).set(1.0*0.612);
      yoMotorDamping.get(WandererJoint.RIGHT_HIP_Y).set(1.0*0.612);
      yoMotorDamping.get(WandererJoint.RIGHT_HIP_X).set(10.0*2.286);
      masterMotorDamping.set(1.0);

   }

   private void initializeFeedForwardTorqueFromDesiredAcceleration()
   {
      desiredQddFeedForwardGain.get(WandererJoint.LEFT_ANKLE_Y).set(0.5);
      desiredQddFeedForwardGain.get(WandererJoint.RIGHT_ANKLE_Y).set(0.5);
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
         for (WandererJoint joint : WandererJoint.values)
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
         for (WandererJoint joint : WandererJoint.values)
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

            // Slightly hackish but won't change any ATLAS code.
            wholeBodyControlJoint.setTau(tau);

         }
   }

   private void initializeJointControlMode(String[] jointNameToIgnore)
   {
      for (WandererJoint joint : WandererJoint.values)
      {
         EnumYoVariable<JointControlMode> controlMode = new EnumYoVariable<WandererOutputWriter.JointControlMode>(joint.getSdfName()
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

   private double getControlRatioByJointControlMode(WandererJoint joint)
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

   private double calcSpringTorque(WandererJoint joint, double q)
   {
     if (standPrep.getStandPrepState() != WandererStandPrep.StandPrepState.EXECUTE)
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
        if(USE_LEFT_ANKLE_SPRING)
           return (currentWalkingState != WalkingStateEnum.WALKING_RIGHT_SUPPORT) ?
              leftAnkleSpringCalculator.getSpringForce() :
              leftAnkleSpringCalculator.getSpringForce()*0.75;//0.75 at DRC
        else
           return 0.0;
      }
      case RIGHT_ANKLE_Y:
      {
        yoAngleSpring.get(joint).set(q);
        rightAnkleSpringCalculator.update(q);
        if(USE_RIGHT_ANKLE_SPRING)
           return (currentWalkingState != WalkingStateEnum.WALKING_LEFT_SUPPORT) ?
              rightAnkleSpringCalculator.getSpringForce() :
              rightAnkleSpringCalculator.getSpringForce()*0.75;//0.75 at DRC
        else
           return 0.0;
      }
      default:
         return 0.0;
     }
   }

   @Override
   public void setFullRobotModel(FullHumanoidRobotModel controllerModel, RawJointSensorDataHolderMap rawJointSensorDataHolderMap)
   {
      wholeBodyControlJoints = WandererUtil.createJointMap(controllerModel.getOneDoFJoints());
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
   public void controllerFailed(FrameVector2d fallingDirection)
   {
      enableOutput.set(false);
      yoWalkingState.set(WalkingStateEnum.TO_STANDING);
      //((EnumYoVariable<WalkingState>)registry.getVariable("WalkingHighLevelHumanoidController", "walkingState")).setValue(yoWalkingState,true);
      //((BooleanYoVariable)registry.getVariable("DesiredFootstepCalculatorFootstepProviderWrapper","walk")).set(false);
   }

}