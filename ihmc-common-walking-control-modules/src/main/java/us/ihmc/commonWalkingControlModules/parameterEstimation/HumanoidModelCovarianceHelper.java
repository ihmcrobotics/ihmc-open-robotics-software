package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrix;
import us.ihmc.commonWalkingControlModules.configurations.InertialEstimationParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.InertialParameterManagerFactory.EstimatorType;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.parameterEstimation.inertial.RigidBodyInertialParameters;
import us.ihmc.parameterEstimation.inertial.RigidBodyInertialParametersTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.math.YoMatrix;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.HashMap;
import java.util.Map;

public class HumanoidModelCovarianceHelper
{
   private final YoRegistry registry;

//   private final List<String> legJointNames;
//   private final List<String> spineJointNames;
//   private final List<String> armJointNames;
//
//   private final String[] trunkBodyNames;

   private final Map<RigidBodyReadOnly, RobotSection> bodyToSectionMap = new HashMap<>();
   private final Map<JointReadOnly, MeasurementChannel> jointToChannelMap = new HashMap<>();

   private final Map<RobotSection, YoMatrix> sectionToProcessCovarianceMap = new HashMap<>();
   private final Map<MeasurementChannel, YoDouble> channelToMeasurementCovarianceMap = new HashMap<>();

   HumanoidModelCovarianceHelper(FullHumanoidRobotModel model, InertialEstimationParameters parameters, YoRegistry parentRegistry)
   {
      registry = new YoRegistry(getClass().getSimpleName());
      parentRegistry.addChild(registry);

      createSectionToProcessCovarianceMap(parameters);
      createChannelToMeasurementCovarianceMap(parameters);
   }

   private void createChannelToMeasurementCovarianceMap(InertialEstimationParameters parameters)
   {
      for (MeasurementChannel channel : MeasurementChannel.values())
      {
         channelToMeasurementCovarianceMap.put(channel, new YoDouble(channel.name() + "MeasurementCovariance", registry));
         switch (channel)
         {
            case FLOATING_BASE:
               channelToMeasurementCovarianceMap.get(channel).set(parameters.getFloatingBaseMeasurementCovariance());
               break;
            case SPINE:
               channelToMeasurementCovarianceMap.get(channel).set(parameters.getSpineMeasurementCovariance());
               break;
            case ARM:
               channelToMeasurementCovarianceMap.get(channel).set(parameters.getArmMeasurementCovariance());
               break;
            case LEG:
               channelToMeasurementCovarianceMap.get(channel).set(parameters.getLegMeasurementCovariance());
               break;
         }
      }
   }

   private void createSectionToProcessCovarianceMap(InertialEstimationParameters parameters)
   {
      String[] rowNames;
      if (parameters.getTypeOfEstimatorToUse() == EstimatorType.PHYSICALLY_CONSISTENT_EKF)
         rowNames = RigidBodyInertialParametersTools.getNamesForThetaBasis();
      else
         rowNames = RigidBodyInertialParametersTools.getNamesForPiBasis();
      for (RobotSection section : RobotSection.values())
      {
         sectionToProcessCovarianceMap.put(section, new YoMatrix(section.name() + "ProcessCovariance",
                                                                 RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY, 1,
                                                                 rowNames, null,
                                                                 registry));
         double[] defaultProcessCovariance = parameters.getProcessModelCovarianceForBody();
         for (int i = 0; i < RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY; i++)
            sectionToProcessCovarianceMap.get(section).set(i, 0, defaultProcessCovariance[i]);
      }
   }

   public DMatrix getProcessCovarianceForBody(RigidBodyReadOnly body)
   {
      return sectionToProcessCovarianceMap.get(bodyToSectionMap.get(body));
   }

   public double getMeasurementCovarianceForJoint(JointReadOnly joint)
   {
      return channelToMeasurementCovarianceMap.get(jointToChannelMap.get(joint)).getValue();
   }

   private enum RobotSection
   {
      TRUNK,
      LEFT_ARM, LEFT_LEG,
      RIGHT_ARM, RIGHT_LEG;

      public static RobotSection getLeg(RobotSide side)
      {
         if (side == RobotSide.LEFT)
            return LEFT_LEG;
         else
            return RIGHT_LEG;
      }

      public static RobotSection getArm(RobotSide side)
      {
         if (side == RobotSide.LEFT)
            return LEFT_ARM;
         else
            return RIGHT_ARM;
      }
   }

   private enum MeasurementChannel
   {
      FLOATING_BASE,
      SPINE,
      ARM,
      LEG;
   }
}
