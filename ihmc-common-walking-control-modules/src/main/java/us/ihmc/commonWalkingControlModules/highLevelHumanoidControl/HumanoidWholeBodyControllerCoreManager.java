package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl;

import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.YoLowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicListDefinition;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;

public class HumanoidWholeBodyControllerCoreManager implements RobotController, SCS2YoGraphicHolder
{
   private final YoRegistry registry = new YoRegistry("WholeBodyControllerCoreThreadManager");
   private final JointDesiredOutputListBasics lowLevelControllerOutput;
   private final JointDesiredOutputListBasics wholeBodyControllerCoreOutput;
   private final YoLowLevelOneDoFJointDesiredDataHolder yoLowLevelOneDoFJointDesiredDataHolder;
//   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final JointBasics[] controlledJoint;
   private final HighLevelControllerFactoryHelper controllerFactoryHelper;
   private final RootJointDesiredConfigurationData rootJointDesiredConfiguration = new RootJointDesiredConfigurationData();

   public HumanoidWholeBodyControllerCoreManager(FullHumanoidRobotModel fullRobotModel,
                                                 JointDesiredOutputListBasics wholeBodyControllerCoreOutput,
                                                 JointDesiredOutputListBasics lowLevelControllerOutput,
                                                 JointBasics... jointsToIgnore)
   {


      this.lowLevelControllerOutput = lowLevelControllerOutput;
//      this.controllerToolbox = controllerToolbox;
      this.wholeBodyControllerCoreOutput = wholeBodyControllerCoreOutput;

      controllerFactoryHelper = new HighLevelControllerFactoryHelper();
      controllerFactoryHelper.setLowLevelControllerOutput(lowLevelControllerOutput);
      controlledJoint = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullRobotModel, jointsToIgnore );

//      registry.addChild(controllerToolbox.getYoVariableRegistry());

      OneDoFJointBasics[] controlledOneDoFJoints = MultiBodySystemTools.filterJoints(controlledJoint, OneDoFJointBasics.class);
      yoLowLevelOneDoFJointDesiredDataHolder = new YoLowLevelOneDoFJointDesiredDataHolder(controlledOneDoFJoints, registry);

      yoLowLevelOneDoFJointDesiredDataHolder.overwriteWith(wholeBodyControllerCoreOutput);


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
      copyJointDesiredsToJoints();
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
      // JointDesiredOutputListReadOnly lowLevelOneDoFJointDesiredDataHolder = stateMachine.getCurrentState().getOutputForLowLevelController();
      JointDesiredOutputListReadOnly lowLevelOneDoFJointDesiredDataHolder = wholeBodyControllerCoreOutput;
//
//      for (int jointIndex = 0; jointIndex < lowLevelOneDoFJointDesiredDataHolder.getNumberOfJointsWithDesiredOutput(); jointIndex++)
//      {
//         OneDoFJointReadOnly controlledJoint = lowLevelOneDoFJointDesiredDataHolder.getOneDoFJoint(jointIndex);
//         JointDesiredOutputReadOnly lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(controlledJoint);
//
//         if (!lowLevelJointData.hasControlMode())
//            throw new NullPointerException("Joint: " + controlledJoint.getName() + " has no control mode.");
//      }

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
}

