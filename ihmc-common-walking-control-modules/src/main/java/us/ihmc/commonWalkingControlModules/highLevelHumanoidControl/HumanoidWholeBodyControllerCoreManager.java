package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl;

import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.YoLowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;

public class HumanoidWholeBodyControllerCoreManager implements RobotController, SCS2YoGraphicHolder
{
   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);
   private final JointDesiredOutputListBasics lowLevelControllerOutput;
   private final JointDesiredOutputListBasics wholeBodyControllerCoreOutput;
   private final YoLowLevelOneDoFJointDesiredDataHolder yoLowLevelOneDoFJointDesiredDataHolder;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final RootJointDesiredConfigurationData rootJointDesiredConfiguration = new RootJointDesiredConfigurationData();




   public HumanoidWholeBodyControllerCoreManager(HighLevelHumanoidControllerToolbox controllerToolbox,
         JointDesiredOutputListBasics wholeBodyControllerCoreOutput,
         JointDesiredOutputListBasics lowLevelControllerOutput)
   {

      this.lowLevelControllerOutput = lowLevelControllerOutput;
      this.controllerToolbox = controllerToolbox;
      this.wholeBodyControllerCoreOutput = wholeBodyControllerCoreOutput;

      registry.addChild(controllerToolbox.getYoVariableRegistry());

      OneDoFJointBasics[] controlledOneDoFJoints = MultiBodySystemTools.filterJoints(controllerToolbox.getControlledJoints(), OneDoFJointBasics.class);
      yoLowLevelOneDoFJointDesiredDataHolder = new YoLowLevelOneDoFJointDesiredDataHolder(controlledOneDoFJoints, registry);

   }
   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      return null;
   }

   @Override
   public void doControl()
   {


   }

   @Override
   public void initialize()
   {

   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return null;
   }
}
   private void copyJointDesiredsToJoints()
   {
      // TODO should replace the output of the WholeBodyControllerCore Outputs from the ControllerThread.
//      JointDesiredOutputListReadOnly lowLevelOneDoFJointDesiredDataHolder = stateMachine.getCurrentState().getOutputForLowLevelController();
      JointDesiredOutputListReadOnly lowLevelOneDoFJointDesiredDataHolder = wholeBodyControllerCoreOutput.getJointDesiredOutput();

      for (int jointIndex = 0; jointIndex < lowLevelOneDoFJointDesiredDataHolder.getNumberOfJointsWithDesiredOutput(); jointIndex++)
      {
         OneDoFJointReadOnly controlledJoint = lowLevelOneDoFJointDesiredDataHolder.getOneDoFJoint(jointIndex);
         JointDesiredOutputReadOnly lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(controlledJoint);

         if (!lowLevelJointData.hasControlMode())
            throw new NullPointerException("Joint: " + controlledJoint.getName() + " has no control mode.");
      }

      yoLowLevelOneDoFJointDesiredDataHolder.overwriteWith(lowLevelOneDoFJointDesiredDataHolder);
      lowLevelControllerOutput.overwriteWith(lowLevelOneDoFJointDesiredDataHolder);

      // TODO This will be running on the ControllerThread.
      // There is no place returning this value.
//      RootJointDesiredConfigurationDataReadOnly rootJointDesiredConfiguration = stateMachine.getCurrentState().getOutputForRootJoint();
//      if (rootJointDesiredConfiguration != null)
//      {
//         this.rootJointDesiredConfiguration.set(rootJointDesiredConfiguration);
//      }
   }
