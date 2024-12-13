package us.ihmc.commonWalkingControlModules.controllerCore.command;

import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotMPCContextData;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.JointHashCodeResolver;
import us.ihmc.robotModels.RigidBodyHashCodeResolver;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class MPCCrossRobotCommandResolver extends CrossRobotCommandResolver
{
   public MPCCrossRobotCommandResolver(FullHumanoidRobotModel fullHumanoidRobotModel)
   {
      super(fullHumanoidRobotModel);
   }
   public MPCCrossRobotCommandResolver(ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver,
                                       RigidBodyHashCodeResolver rigidBodyHashCodeResolver,
                                       JointHashCodeResolver jointHashCodeResolver)
   {
      super(referenceFrameHashCodeResolver, rigidBodyHashCodeResolver, jointHashCodeResolver);
   }

   public void resolveHumanoidRobotContextDataScheduler(HumanoidRobotMPCContextData in, HumanoidRobotMPCContextData out)
   {
      resolveSensorDataContext(in.getSensorDataContext(), out.getSensorDataContext());
      out.setTimestamp(in.getTimestamp());
      out.setSchedulerTick(in.getSchedulerTick());
   }
   public void resolveHumanoidRobotContextDataController(HumanoidRobotMPCContextData in, HumanoidRobotMPCContextData out)
   {
      resolveCenterOfPressureDataHolder(in.getCenterOfPressureDataHolder(), out.getCenterOfPressureDataHolder());
      resolveRobotMotionStatusHolder(in.getRobotMotionStatusHolder(), out.getRobotMotionStatusHolder());
      resolveLowLevelOneDoFJointDesiredDataHolder(in.getJointDesiredOutputList(), out.getJointDesiredOutputList());
      out.setControllerRan(in.getControllerRan());
   }
   public void resolveHumanoidRobotContextDataEstimator(HumanoidRobotMPCContextData in, HumanoidRobotMPCContextData out)
   {
      resolveHumanoidRobotContextJointData(in.getProcessedJointData(), out.getProcessedJointData());
      resolveForceSensorDataHolder(in.getForceSensorDataHolder(), out.getForceSensorDataHolder());
      resolveCenterOfMassDataHolder(in.getCenterOfMassDataHolder(), out.getCenterOfMassDataHolder());
      out.setEstimatorRan(in.getEstimatorRan());
   }

}
