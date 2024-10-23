package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.ControllerCoreOutputDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.YoLowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.WholeBodyControllerCoreFactory;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.sensors.CenterOfMassDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicListDefinition;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class HumanoidWholeBodyControllerCoreManager implements RobotController, SCS2YoGraphicHolder
{
   private final YoRegistry registry = new YoRegistry("WholeBodyControllerCoreThreadManager");

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final JointDesiredOutputListBasics lowLevelControllerOutput;
   private final YoLowLevelOneDoFJointDesiredDataHolder yoLowLevelOneDoFJointDesiredDataHolder;
   private final JointBasics[] controlledJoint;
   //   private final HighLevelControllerFactoryHelper controllerFactoryHelper;
   private final ControllerCoreOutputDataHolder controllerCoreOutPutDataHolder;
   private final ControllerCoreCommandDataHolder controllerCoreCommandDataHolder;
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand();
   private final WholeBodyControllerCore controllerCore;
   private final RootJointDesiredConfigurationData rootJointDesiredConfiguration = new RootJointDesiredConfigurationData();

   // This constructor has similar role to the getController of @HighLevelHumanoidControllerFactory

   public HumanoidWholeBodyControllerCoreManager(FullHumanoidRobotModel fullRobotModel,
                                                 double controlDT,
                                                 double gravity,
                                                 boolean kinematicsSimulation,
                                                 YoDouble yoTime,
                                                 ForceSensorDataHolderReadOnly forceSensorDataHolder,
                                                 CenterOfMassDataHolderReadOnly centerOfMassDataHolderForControllerCore,
                                                 JointDesiredOutputListBasics lowLevelControllerOutput,
                                                 ControllerCoreOutputDataHolder controllerCoreOutputDataHolder,
                                                 ControllerCoreCommandDataHolder controllerCoreCommandDataHolder,
                                                 WholeBodyControllerCoreFactory controllerCoreFactory,
                                                 JointBasics... jointsToIgnore)
   {
      //      YoBoolean usingEstimatorCoMPosition = new YoBoolean("usingEstimatorCoMPositionInControllerCoreThread", registry);
      //      YoBoolean usingEstimatorCoMVelocity = new YoBoolean("usingEstimatorCoMVelocityInControllerCoreThread", registry);
      //
      this.controllerCore = controllerCoreFactory.getOrCreateWholeBodyControllerCore();
      this.lowLevelControllerOutput = lowLevelControllerOutput;
      this.controllerCoreOutPutDataHolder = controllerCoreOutputDataHolder;
      this.controllerCoreCommandDataHolder = controllerCoreCommandDataHolder;

      controlledJoint = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullRobotModel, jointsToIgnore);

      controllerCoreCommand.setControllerCoreMode(controllerCoreCommandDataHolder.getControllerCoreMode());

      OneDoFJointBasics[] controlledOneDoFJoints = MultiBodySystemTools.filterJoints(controlledJoint, OneDoFJointBasics.class);
      yoLowLevelOneDoFJointDesiredDataHolder = new YoLowLevelOneDoFJointDesiredDataHolder(controlledOneDoFJoints, registry);

   }

   private final YoGraphicListDefinition scs2AdditionalYoGraphics = new YoGraphicListDefinition();

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(scs2AdditionalYoGraphics);
      //      WholeBodyControllerCoreFactory wholeBodyControllerCoreFactory = controllerFactoryHelper.getWholeBodyControllerCoreFactory();
      //      WholeBodyControllerCore wholeBodyControllerCore = wholeBodyControllerCoreFactory.getWholeBodyControllerCore();
      //      if (wholeBodyControllerCore != null)
      //         group.addChild(wholeBodyControllerCore.getSCS2YoGraphics());
      //      LinearMomentumRateControlModule linearMomentumRateControlModule = wholeBodyControllerCoreFactory.getLinearMomentumRateControlModule();
      //      if (linearMomentumRateControlModule != null)
      //         group.addChild(linearMomentumRateControlModule.getSCS2YoGraphics());
      //      group.addChild(controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getSCS2YoGraphics());
      //      group.addChild(controllerFactoryHelper.getManagerFactory().getSCS2YoGraphics());

      return group;
   }

   @Override
   public void doControl()
   {
      controllerCoreCommand.set(controllerCoreCommandDataHolder);

      if (controllerCoreCommand.getControllerCoreMode() == null)
         controllerCoreCommand.setControllerCoreMode(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);

      try
      {
         // This should be pass by some kind of CASE statements.
         // Not all the highLevelControllerState use the WholeBodyControllerCore
         // But WholeBodyControllerCore updates the joint input here.
         // So, the output of other highLevelStates should be called in here to be delivered to copyJointDesiredsToJoints
         controllerCore.submitControllerCoreCommand(controllerCoreCommand);
         controllerCore.compute();
      }
      catch (Exception e)
      {
         e.printStackTrace();
         // Are these below commands needed?
         // Below commands are written in the HumanoidHighLevelControllerManager for the controlThread
         //         statusMessageOutputManager.reportStatusMessage(MessageTools.createControllerCrashNotificationPacket(ControllerCrashLocation.CONTROLLER_RUN, e));
         //         controllerToolbox.reportControllerFailureToListeners(null);
      }

      copyJointDesiredsToJoints();
      reportControllerCoreOutputDataHolderForController();
   }

   @Override
   public void initialize()
   {

   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   private void copyJointDesiredsToJoints()
   {
      // TODO should replace the output of the WholeBodyControllerCore Outputs from the ControllerThread.

      // The output of the controllCore in this Manager will be saved into loweLevelControllerOutput,
      // which is written to the joints
      JointDesiredOutputListReadOnly lowLevelOneDoFJointDesiredDataHolder = controllerCore.getControllerCoreOutput().getLowLevelOneDoFJointDesiredDataHolder();
      //      yoLowLevelOneDoFJointDesiredDataHolder.overwriteWith(lowLevelOneDoFJointDesiredDataHolder);
      lowLevelControllerOutput.overwriteWith(lowLevelOneDoFJointDesiredDataHolder);

      // TODO This will be running on the ControllerThread.
      // There is no place returning this value.
      //      RootJointDesiredConfigurationDataReadOnly rootJointDesiredConfiguration = stateMachine.getCurrentState().getOutputForRootJoint();
      //      if (rootJointDesiredConfiguration != null)
      //      {
      //         this.rootJointDesiredConfiguration.set(rootJointDesiredConfiguration);
      //      }
   }

   private void reportControllerCoreOutputDataHolderForController()
   {
      controllerCoreOutPutDataHolder.setControllerCoreOutputDataHolder(controllerCore.getControllerCoreOutput());
   }
}

