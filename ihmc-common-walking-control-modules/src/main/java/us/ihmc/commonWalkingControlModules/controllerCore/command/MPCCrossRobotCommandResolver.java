package us.ihmc.commonWalkingControlModules.controllerCore.command;

import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotMPCContextData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.ControllerCoreOutputDataHolder;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.JointHashCodeResolver;
import us.ihmc.robotModels.RigidBodyHashCodeResolver;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

/**
 * The objective of this class is to help the passing commands between two instances of the same
 * robot.
 * <p>
 * The main use-case is for passing commands from one thread to another. In such context, each
 * thread has its own instance of the robot and the corresponding reference frame tree.
 * </p>
 * <p>
 * The main challenge when passing commands is to retrieve the joints, rigid-bodies, and reference
 * frames properly.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class MPCCrossRobotCommandResolver extends CrossRobotCommandResolver
{
   public MPCCrossRobotCommandResolver(FullHumanoidRobotModel fullRobotModel)
   {
      super(fullRobotModel);
   }

   public MPCCrossRobotCommandResolver(ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver,
                                       RigidBodyHashCodeResolver rigidBodyHashCodeResolver,
                                       JointHashCodeResolver jointHashCodeResolver)
   {
      super(referenceFrameHashCodeResolver, rigidBodyHashCodeResolver, jointHashCodeResolver);
   }

   /**
    * Resolve command for controllerCoreCommand.
    */
   public void resolveControllerCoreCommandDataHolder(ControllerCoreCommandDataHolder in, ControllerCoreCommandDataHolder out)
   {
      out.clear();
      out.set(in);
      //      if(in.isReinitializationRequested())
      //         out.requestReinitialization();
   }

   public void resolveControllerCoreOutputDataHolder(ControllerCoreOutputDataHolder in, ControllerCoreOutputDataHolder out)
   {

      out.setControllerCoreOutputDataHolder(in);
   }

   public void resolveHumanoidRobotContextData(HumanoidRobotMPCContextData in, HumanoidRobotMPCContextData out)
   {
      resolveHumanoidRobotContextDataScheduler(in, out);
      resolveHumanoidRobotContextDataController(in, out);
      resolveHumanoidRobotContextDataEstimator(in, out);
      resolveHumanoidRobotContextDataWholeBodyControllerCore(in, out);
      resolveHumanoidRobotContextDataPerception(in, out);
   }

   /**
    * Resolves only the part of the context data that is updated by the scheduler thread.
    */
   public void resolveHumanoidRobotContextDataScheduler(HumanoidRobotMPCContextData in, HumanoidRobotMPCContextData out)
   {
      resolveSensorDataContext(in.getSensorDataContext(), out.getSensorDataContext());
      out.setTimestamp(in.getTimestamp());
      out.setSchedulerTick(in.getSchedulerTick());
   }

   /**
    * Resolves only the part of the context data that is updated by the controller thread.
    */
   public void resolveHumanoidRobotContextDataController(HumanoidRobotMPCContextData in, HumanoidRobotMPCContextData out)
   {
      resolveCenterOfPressureDataHolder(in.getCenterOfPressureDataHolder(), out.getCenterOfPressureDataHolder());
      resolveRobotMotionStatusHolder(in.getRobotMotionStatusHolder(), out.getRobotMotionStatusHolder());
      resolveControllerCoreCommandDataHolder(in.getControllerCoreCommandDataHolder(), out.getControllerCoreCommandDataHolder());
      resolveLowLevelOneDoFJointDesiredDataHolder(in.getWholeBodyControllerCoreDesiredOutPutList(), out.getWholeBodyControllerCoreDesiredOutPutList());

      out.setControllerRan(in.getControllerRan());
   }

   /**
    * Resolves only the part of the context data that is updated by the wholeBodyControllerCore thread
    * This resolves the ControllerCoreOutput and desiredJointOutput
    */
   public void resolveHumanoidRobotContextDataWholeBodyControllerCore(HumanoidRobotMPCContextData in, HumanoidRobotMPCContextData out)
   {
      resolveLowLevelOneDoFJointDesiredDataHolder(in.getJointDesiredOutputList(), out.getJointDesiredOutputList());
      //      resolveLowLevelOneDoFJointDesiredDataHolder(in.getWholeBodyControllerCoreDesiredOutPutList(), out.getWholeBodyControllerCoreDesiredOutPutList());
      resolveControllerCoreOutputDataHolder(in.getControllerCoreOutPutDataHolder(), out.getControllerCoreOutPutDataHolder());
      out.setWholeBodyControllerCoreRan(in.getWholeBodyControllerCoreRan());
   }

   /**
    * Resolves only the part of the context data that is updated by the perception thread.
    */
   public void resolveHumanoidRobotContextDataPerception(HumanoidRobotMPCContextData in, HumanoidRobotMPCContextData out)
   {
      out.setPerceptionRan(in.getPerceptionRan());
   }

   /**
    * Resolves only the part of the context data that is updated by the estimator thread.
    */
   public void resolveHumanoidRobotContextDataEstimator(HumanoidRobotMPCContextData in, HumanoidRobotMPCContextData out)
   {
      resolveHumanoidRobotContextJointData(in.getProcessedJointData(), out.getProcessedJointData());
      resolveForceSensorDataHolder(in.getForceSensorDataHolder(), out.getForceSensorDataHolder());
      resolveCenterOfMassDataHolder(in.getCenterOfMassDataHolder(), out.getCenterOfMassDataHolder());
      out.setEstimatorRan(in.getEstimatorRan());
   }
}