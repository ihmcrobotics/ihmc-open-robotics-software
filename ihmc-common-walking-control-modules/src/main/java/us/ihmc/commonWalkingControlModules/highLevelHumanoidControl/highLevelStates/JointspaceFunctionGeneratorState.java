package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGeneratorMode;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGeneratorNew;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class JointspaceFunctionGeneratorState extends HighLevelControllerState
{
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   private final PairList<OneDoFJointBasics, YoFunctionGeneratorNew> jointFunctionGenerators = new PairList<>();

   public JointspaceFunctionGeneratorState(HighLevelControllerName stateEnum,
                                           OneDoFJointBasics[] controlledJoints,
                                           HighLevelControllerParameters highLevelControllerParameters,
                                           DoubleProvider yoTime)
   {
      super(stateEnum, highLevelControllerParameters, controlledJoints);

      String nameSuffix = "_" + stateEnum.name();

      for (OneDoFJointBasics controlledJoint : controlledJoints)
      {
         String jointName = controlledJoint.getName();

         YoRegistry jointRegistry = new YoRegistry(jointName + "FuncGen");
         YoFunctionGeneratorNew jointFunctionGenerator = new YoFunctionGeneratorNew(jointName + "FuncGen", yoTime, jointRegistry);
         registry.addChild(jointRegistry);
         
         jointFunctionGenerators.add(controlledJoint, jointFunctionGenerator);
      }

      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledJoints);
   }

   @Override
   public void onEntry()
   {
      for (int jointIndex = 0; jointIndex < jointFunctionGenerators.size(); jointIndex++)
      {
         YoFunctionGeneratorNew generator = jointFunctionGenerators.get(jointIndex).getRight();

         generator.setAmplitude(0.0);
         generator.setFrequency(0.0);
         generator.setOffset(0.0);
         generator.setMode(YoFunctionGeneratorMode.OFF);
      }
   }

   @Override
   public void doAction(double timeInState)
   {
      for (int jointIndex = 0; jointIndex < jointFunctionGenerators.size(); jointIndex++)
      {
         OneDoFJointBasics joint = jointFunctionGenerators.get(jointIndex).getLeft();
         YoFunctionGeneratorNew generator = jointFunctionGenerators.get(jointIndex).getRight();

         generator.update();

         JointDesiredOutputBasics lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(joint);
         lowLevelJointData.clear();
         lowLevelJointData.setDesiredPosition(generator.getValue());
         lowLevelJointData.setDesiredVelocity(generator.getValueDot());
         lowLevelJointData.setDesiredAcceleration(0.0);
         lowLevelJointData.setDesiredTorque(0.0);
      }

      lowLevelOneDoFJointDesiredDataHolder.completeWith(getStateSpecificJointSettings());
   }

   @Override
   public void onExit(double timeInState)
   {

   }

   @Override
   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }
}
