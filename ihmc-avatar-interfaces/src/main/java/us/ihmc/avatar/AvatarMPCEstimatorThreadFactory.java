package us.ihmc.avatar;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotMPCContextData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotMPCContextDataFactory;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.ControllerCoreOutputDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.BarrierScheduler;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataPublisher;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;
import us.ihmc.sensorProcessing.sensorProcessors.RobotJointLimitWatcher;
import us.ihmc.stateEstimation.humanoid.StateEstimatorControllerFactory;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.factories.RequiredFactoryField;
import us.ihmc.wholeBodyController.parameters.ParameterLoaderHelper;

import java.util.List;

public class AvatarMPCEstimatorThreadFactory extends AvatarEstimatorThreadFactory
{
   private final RequiredFactoryField<HumanoidRobotMPCContextDataFactory> humanoidRobotContextDataFactoryField = new RequiredFactoryField<>(
         "humanoidRobotContextDataFactory");

   private final OptionalFactoryField<List<StateEstimatorControllerFactory>> secondaryStateEstimatorFactoriesField = new OptionalFactoryField<>(
         "secondaryEstimatorControllerFactories");

   private final OptionalFactoryField<HumanoidRobotMPCContextData> humanoidRobotContextDataField = new OptionalFactoryField<>("humanoidRobotContextData");
   private final OptionalFactoryField<LowLevelOneDoFJointDesiredDataHolder> wbccDesiredJointDataHolderField = new OptionalFactoryField<>(
         "desiredJointDataHolder");

   private final OptionalFactoryField<ControllerCoreOutputDataHolder> controllerCoreOutputDataHolderField = new OptionalFactoryField<>(
         "wholeBodyControllerCoreDataHolder");
   private final OptionalFactoryField<ControllerCoreCommandDataHolder> controllerCoreCommandDataHolderField = new OptionalFactoryField<>(
         "controllerCoreCommandDataHolder");

   private final OptionalFactoryField<JointDesiredOutputWriter> jointDesiredOutputWriterField = new OptionalFactoryField<>("jointDesiredOutputWriter");

   /**
    * Creates a new factory to create {@link AvatarEstimatorThread}.
    * <p>
    * Example for configuring this factory:
    *
    * <pre>
    * AvatarEstimatorThreadFactory avatarEstimatorThreadFactory = new AvatarEstimatorThreadFactory();
    * avatarEstimatorThreadFactory.setROS2Info(realtimeROS2Node, robotName);
    * avatarEstimatorThreadFactory.configureWithDRCRobotModel(robotModel);
    * avatarEstimatorThreadFactory.setSensorReaderFactory(sensorReaderFactory);
    * avatarEstimatorThreadFactory.setHumanoidRobotContextDataFactory(contextDataFactory);
    * avatarEstimatorThreadFactory.setExternalPelvisCorrectorSubscriber(pelvisPoseCorrectionCommunicator);
    * avatarEstimatorThreadFactory.setJointDesiredOutputWriter(simulationOutputWriter);
    * avatarEstimatorThreadFactory.setGravity(gravity);
    * AvatarEstimatorThread estimatorThread = avatarEstimatorThreadFactory.createAvatarEstimatorThread();
    * </pre>
    *
    * Where {@code robotModel} is an implementation of {@link DRCRobotModel}.
    * </p>
    */
   public AvatarMPCEstimatorThreadFactory()
   {
      super();
   }

   /**
    * The factory to create the context for the state estimator needed to run with the
    * {@link BarrierScheduler}.
    *
    * @param contextDataFactory the context factory.
    */
   public void setHumanoidRobotContextDataFactory(HumanoidRobotMPCContextDataFactory contextDataFactory)
   {
      humanoidRobotContextDataFactoryField.set(contextDataFactory);
   }

   public AvatarEstimatorThread createAvatarEstimatorThread()
   {
      if (jointDesiredOutputWriterField.hasValue())
      {
         jointDesiredOutputWriterField.get().setJointDesiredOutputList(getDesiredJointDataHolder());
         getEstimatorRegistry().addChild(jointDesiredOutputWriterField.get().getYoVariableRegistry());
      }
      if (secondaryStateEstimatorFactoriesField.hasValue())
      {
         for (StateEstimatorControllerFactory stateEstimatorControllerFactory : secondaryStateEstimatorFactoriesField.get())
            addSecondaryStateEstimator(stateEstimatorControllerFactory.createStateEstimator(getEstimatorFullRobotModel(), getSensorReader()));
      }

      AvatarEstimatorThread avatarEstimatorThread = new AvatarEstimatorThread(getSensorReader(),
                                                                              getEstimatorFullRobotModel(),
                                                                              getHumanoidRobotContextData(),
                                                                              getMainStateEstimator(),
                                                                              getSecondaryStateEstimators(),
                                                                              createControllerCrashPublisher(),
                                                                              getEstimatorRegistry(),
                                                                              getYoGraphicsListRegistry());

      avatarEstimatorThread.addRobotController(new RobotJointLimitWatcher(getEstimatorFullRobotModel().getOneDoFJoints(), getRawSensorOutputMap()));
      RobotConfigurationDataPublisher robotConfigurationDataPublisher = getRobotConfigurationDataPublisher();
      if (robotConfigurationDataPublisher != null)
      {
         avatarEstimatorThread.setRawOutputWriter(robotConfigurationDataPublisher);
      }
      ParameterLoaderHelper.loadParameters(this, getControllerParameters(), getEstimatorRegistry());

      FactoryTools.disposeFactory(this);
      return avatarEstimatorThread;
   }

   @Override
   public HumanoidRobotMPCContextDataFactory getHumanoidRobotContextDataFactory()
   {
      return humanoidRobotContextDataFactoryField.get();
   }

   @Override
   public HumanoidRobotMPCContextData getHumanoidRobotContextData()
   {
      if (!humanoidRobotContextDataField.hasValue())
      {
         HumanoidRobotMPCContextDataFactory contextDataFactory = getHumanoidRobotContextDataFactory();
         contextDataFactory.setForceSensorDataHolder(getForceSensorDataHolder());
         contextDataFactory.setCenterOfMassDataHolder(getCenterOfMassDataHolder());
         contextDataFactory.setCenterOfPressureDataHolder(getCenterOfPressureDataHolderFromController());
         contextDataFactory.setRobotMotionStatusHolder(getRobotMotionStatusFromController());
         contextDataFactory.setJointDesiredOutputList(getDesiredJointDataHolder());
         contextDataFactory.setProcessedJointData(getHumanoidRobotContextJointData());
         contextDataFactory.setSensorDataContext(getSensorDataContext());
         contextDataFactory.setWBCCJointDesiredOutputList(getWBCCDesiredJointDataHolder());
         contextDataFactory.setControllerCoreOutputDataHolder(getControllerCoreOutPutDataHolder());
         contextDataFactory.setControllerCoreCommandDataHolder(getControllerCoreCommandDataHolder());
         humanoidRobotContextDataField.set(contextDataFactory.createHumanoidRobotContextData());
      }
      return humanoidRobotContextDataField.get();
   }

   public LowLevelOneDoFJointDesiredDataHolder getWBCCDesiredJointDataHolder()
   {
      if (!wbccDesiredJointDataHolderField.hasValue())
         wbccDesiredJointDataHolderField.set(new LowLevelOneDoFJointDesiredDataHolder(getControllableOneDoFJoints()));
      return wbccDesiredJointDataHolderField.get();
   }

   public ControllerCoreOutputDataHolder getControllerCoreOutPutDataHolder()
   {
      if (!controllerCoreOutputDataHolderField.hasValue())
         controllerCoreOutputDataHolderField.set(new ControllerCoreOutputDataHolder(getControllableOneDoFJoints()));
      return controllerCoreOutputDataHolderField.get();
   }

   public ControllerCoreCommandDataHolder getControllerCoreCommandDataHolder()
   {
      if (!controllerCoreCommandDataHolderField.hasValue())
         controllerCoreCommandDataHolderField.set(new ControllerCoreCommandDataHolder());
      return controllerCoreCommandDataHolderField.get();
   }
}