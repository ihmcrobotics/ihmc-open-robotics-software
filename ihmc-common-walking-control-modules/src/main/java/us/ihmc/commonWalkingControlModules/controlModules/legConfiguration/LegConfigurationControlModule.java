package us.ihmc.commonWalkingControlModules.controlModules.legConfiguration;

import us.ihmc.commonWalkingControlModules.configurations.LegConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedJointSpaceCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class LegConfigurationControlModule
{
   private final YoRegistry registry;

   private final PrivilegedJointSpaceCommand privilegedAccelerationCommand = new PrivilegedJointSpaceCommand();

   private final YoDouble lowPrivilegedWeight;

   private final YoDouble bentJointSpacePositionGain;
   private final YoDouble bentJointSpaceVelocityGain;

   private final YoDouble kneePitchPrivilegedError;

   private final YoDouble jointSpacePAction;
   private final YoDouble jointSpaceDAction;
   private final YoDouble jointSpaceAction;

   private final YoDouble privilegedMaxAcceleration;

   private final OneDoFJointBasics kneePitchJoint;

   private static final int hipPitchJointIndex = 0;
   private static final int kneePitchJointIndex = 1;
   private static final int anklePitchJointIndex = 2;

   private final double kneeRangeOfMotion;
   private final double kneeSquareRangeOfMotion;
   private final double kneeMidRangeOfMotion;


   public LegConfigurationControlModule(RobotSide robotSide, HighLevelHumanoidControllerToolbox controllerToolbox,
                                        LegConfigurationParameters legConfigurationParameters, YoRegistry parentRegistry)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      registry = new YoRegistry(sidePrefix + getClass().getSimpleName());

      kneePitchJoint = controllerToolbox.getFullRobotModel().getLegJoint(robotSide, LegJointName.KNEE_PITCH);
      double kneeLimitUpper = kneePitchJoint.getJointLimitUpper();
      if (Double.isNaN(kneeLimitUpper) || Double.isInfinite(kneeLimitUpper))
         kneeLimitUpper = Math.PI;
      double kneeLimitLower = kneePitchJoint.getJointLimitLower();
      if (Double.isNaN(kneeLimitLower) || Double.isInfinite(kneeLimitLower))
         kneeLimitLower = -Math.PI;
      kneeSquareRangeOfMotion = MathTools.square(kneeLimitUpper - kneeLimitLower);
      kneeRangeOfMotion = kneeLimitUpper - kneeLimitLower;
      kneeMidRangeOfMotion = 0.5 * (kneeLimitUpper + kneeLimitLower);

      OneDoFJointBasics hipPitchJoint = controllerToolbox.getFullRobotModel().getLegJoint(robotSide, LegJointName.HIP_PITCH);
      OneDoFJointBasics anklePitchJoint = controllerToolbox.getFullRobotModel().getLegJoint(robotSide, LegJointName.ANKLE_PITCH);
      privilegedAccelerationCommand.addJoint(hipPitchJoint, Double.NaN);
      privilegedAccelerationCommand.addJoint(kneePitchJoint, Double.NaN);
      privilegedAccelerationCommand.addJoint(anklePitchJoint, Double.NaN);

      lowPrivilegedWeight = new YoDouble(sidePrefix + "LowPrivilegedWeight", registry);

      bentJointSpacePositionGain = new YoDouble(sidePrefix + "BentLegJointSpaceKp", registry);
      bentJointSpaceVelocityGain = new YoDouble(sidePrefix + "BentLegJointSpaceKv", registry);

      privilegedMaxAcceleration = new YoDouble(sidePrefix + "LegPrivilegedMaxAcceleration", registry);

      kneePitchPrivilegedError = new YoDouble(sidePrefix + "KneePitchPrivilegedError", registry);
      jointSpacePAction = new YoDouble(sidePrefix + "KneePrivilegedJointSpacePAction", registry);
      jointSpaceDAction = new YoDouble(sidePrefix + "KneePrivilegedJointSpaceDAction", registry);
      jointSpaceAction = new YoDouble(sidePrefix + "KneePrivilegedJointSpaceAction", registry);

      lowPrivilegedWeight.set(legConfigurationParameters.getLegPrivilegedLowWeight());

      LegConfigurationGains bentLegGains = legConfigurationParameters.getBentLegGains();


      bentJointSpacePositionGain.set(bentLegGains.getJointSpaceKp());
      bentJointSpaceVelocityGain.set(bentLegGains.getJointSpaceKd());

      privilegedMaxAcceleration.set(legConfigurationParameters.getPrivilegedMaxAcceleration());

      parentRegistry.addChild(registry);
   }


   public void initialize()
   {
   }

   public void doControl()
   {
      double kneePitchPrivilegedConfigurationWeight = lowPrivilegedWeight.getDoubleValue();

      double privilegedKneeAcceleration = computeKneeAcceleration();
      double privilegedHipPitchAcceleration = -0.5 * privilegedKneeAcceleration;
      double privilegedAnklePitchAcceleration = -0.5 * privilegedKneeAcceleration;

      privilegedAccelerationCommand.setOneDoFJoint(hipPitchJointIndex, privilegedHipPitchAcceleration);
      privilegedAccelerationCommand.setOneDoFJoint(kneePitchJointIndex, privilegedKneeAcceleration);
      privilegedAccelerationCommand.setOneDoFJoint(anklePitchJointIndex, privilegedAnklePitchAcceleration);

      privilegedAccelerationCommand.setWeight(hipPitchJointIndex, kneePitchPrivilegedConfigurationWeight);
      privilegedAccelerationCommand.setWeight(kneePitchJointIndex, kneePitchPrivilegedConfigurationWeight);
      privilegedAccelerationCommand.setWeight(anklePitchJointIndex, kneePitchPrivilegedConfigurationWeight);
   }

   private double computeKneeAcceleration()
   {
      double currentPosition = kneePitchJoint.getQ();

      double jointError = kneeRangeOfMotion - currentPosition;
      kneePitchPrivilegedError.set(jointError);

      double jointSpaceAction = computeJointSpaceAction();

      return MathTools.clamp(jointSpaceAction, privilegedMaxAcceleration.getDoubleValue());
   }

   private double computeJointSpaceAction()
   {
      double jointError = kneeMidRangeOfMotion - kneePitchJoint.getQ();
      double jointSpaceKp = 2.0 * bentJointSpacePositionGain.getDoubleValue() / kneeSquareRangeOfMotion;

      double jointSpacePAction = Double.isNaN(jointSpaceKp) ? 0.0 : jointSpaceKp * jointError;
      double jointSpaceDAction = Double.isNaN(bentJointSpaceVelocityGain.getDoubleValue()) ? 0.0 : bentJointSpaceVelocityGain.getDoubleValue() * -kneePitchJoint.getQd();

      this.jointSpacePAction.set(jointSpacePAction);
      this.jointSpaceDAction.set(jointSpaceDAction);

      jointSpaceAction.set(jointSpacePAction + jointSpaceDAction);

      return jointSpacePAction + jointSpaceDAction;
   }


   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return privilegedAccelerationCommand;
   }
}
