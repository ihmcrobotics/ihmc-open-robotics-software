package us.ihmc.externalControl;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.JointControlBlender;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.PairList;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.externalControl.library.ExternalControlNativeLibrary;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.HashMap;

public class ExternalControllerState extends HighLevelControllerState
{
   public static boolean REDUCE_YOVARIABLES = false;

   private enum DesiredMode {HOLD_POSITION, REXXXXXXX}

   private final YoDouble blendRatioCurrentValue;

   private final PairList<OneDoFJointBasics, JointControlBlender> jointCommandBlenders = new PairList<>();
   private final JointDesiredOutputListReadOnly highLevelControllerOutput;

   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();
   private final LowLevelOneDoFJointDesiredDataHolder frozenJointDataHolder = new LowLevelOneDoFJointDesiredDataHolder();
   private final LowLevelOneDoFJointDesiredDataHolder externalJointDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   private final YoEnum<DesiredMode> desiredMode = new YoEnum<>("DesiredExternalMode", registry, DesiredMode.class);

   private final CommandInputManager commandInputManager;

   private final PairList<OneDoFJointBasics, YoDouble> frozenJointDesireds = new PairList<>();

   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   private final ExternalControl externalControl;

   private final HashMap<String, YoDouble> debugData = new HashMap<>();

   public ExternalControllerState(HighLevelControllerParameters highLevelControllerParameters,
                                  HighLevelHumanoidControllerToolbox controllerToolbox,
                                  OneDoFJointBasics[] controlledJoints,
                                  JointDesiredOutputListReadOnly highLevelControllerOutput,
                                  CommandInputManager commandInputManager)
   {
      super("externalControl", HighLevelControllerName.EXTERNAL, highLevelControllerParameters, controlledJoints);

      this.highLevelControllerOutput = highLevelControllerOutput;
      this.controllerToolbox = controllerToolbox;

      this.commandInputManager = commandInputManager;

      blendRatioCurrentValue = new YoDouble("ExternalControlBlendRatioCurrentValue", registry);

      ExternalControlNativeLibrary.load();

      desiredMode.set(DesiredMode.HOLD_POSITION);
      externalControl = new ExternalControl(controllerToolbox.getFullRobotModel().getRootBody(), highLevelControllerParameters.getStandPrepParameters(),
                                            controlledJoints, 1000.0, 5.0);


      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledJoints);
      frozenJointDataHolder.registerJointsWithEmptyData(controlledJoints);
      externalJointDataHolder.registerJointsWithEmptyData(controlledJoints);

      YoRegistry registryForBlenders = REDUCE_YOVARIABLES ? null : registry;

      for (OneDoFJointBasics controlledJoint : controlledJoints)
      {
         String jointName = controlledJoint.getName();

         YoDouble freezePosition = new YoDouble(jointName + "_frozen_qDesired", registry);
         freezePosition.setToNaN();
         frozenJointDesireds.add(controlledJoint, freezePosition);

         JointControlBlender jointControlBlender = new JointControlBlender("_ExternalBlender", controlledJoint, registryForBlenders);
         jointCommandBlenders.add(controlledJoint, jointControlBlender);
      }

      for (String name : externalControl.debugDataNames) {
         debugData.put(name, new YoDouble("zmq_mpc_" + name, registry));
      }
   }

   @Override
   public void onEntry()
   {
      for (int jointIndex = 0; jointIndex < frozenJointDesireds.size(); jointIndex++)
      {
         OneDoFJointBasics joint = frozenJointDesireds.get(jointIndex).getLeft();
         YoDouble setpoint = frozenJointDesireds.get(jointIndex).getRight();
         JointDesiredOutputReadOnly lowLevelJointData = highLevelControllerOutput.getJointDesiredOutput(joint);
         if (lowLevelJointData != null && lowLevelJointData.hasDesiredPosition())
            setpoint.set(lowLevelJointData.getDesiredPosition());
         else
            setpoint.set(joint.getQ());
      }

      blendRatioCurrentValue.set(0.0);

      commandInputManager.clearAllCommands();
      commandInputManager.setEnabled(false);
   }

   @Override
   public void doAction(double timeInState)
   {
      controllerToolbox.update();

      externalControl.setFootStates(controllerToolbox.getReferenceFrames().getSoleFrames(),
                                    controllerToolbox.getFootSwitches().get(RobotSide.LEFT).hasFootHitGroundFiltered(),
                                    controllerToolbox.getFootSwitches().get(RobotSide.RIGHT).hasFootHitGroundFiltered());
      externalControl.writeRobotState(controllerToolbox.getYoTime().getDoubleValue(), desiredMode.getOrdinal());

      externalControl.readControlSolution();

      double gainRatio = blendRatioCurrentValue.getValue();

      if (Double.isNaN(gainRatio))
         gainRatio = 0.0;
      else
         gainRatio = MathTools.clamp(gainRatio, 0.0, 1.0);

      for (int jointIndex = 0; jointIndex < jointCommandBlenders.size(); jointIndex++)
      {
         OneDoFJointBasics joint = jointCommandBlenders.get(jointIndex).getLeft();

         YoDouble frozenDesiredPosition = frozenJointDesireds.get(jointIndex).getRight();

         JointDesiredOutputBasics frozenJointData = frozenJointDataHolder.getJointDesiredOutput(joint);
         frozenJointData.clear();
         frozenJointData.setDesiredPosition(frozenDesiredPosition.getDoubleValue());
         frozenJointData.setDesiredVelocity(0.0);
         frozenJointData.setDesiredAcceleration(0.0);
      }
      frozenJointDataHolder.completeWith(getStateSpecificJointSettings());

      for (int jointIndex = 0; jointIndex < jointCommandBlenders.size(); jointIndex++)
      {
         OneDoFJointBasics joint = jointCommandBlenders.get(jointIndex).getLeft();

         JointDesiredOutputBasics externalJointData = externalJointDataHolder.getJointDesiredOutput(joint);
         externalControl.getSolutionData(joint).getJointDesiredOutput(externalJointData);

         JointDesiredOutputBasics frozenJointData = frozenJointDataHolder.getJointDesiredOutput(joint);

         JointDesiredOutputBasics lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(joint);
         lowLevelJointData.clear();

         JointControlBlender jointControlBlender = jointCommandBlenders.get(jointIndex).getRight();
         jointControlBlender.computeAndUpdateJointControl(lowLevelJointData,
                                                          frozenJointData,
                                                          externalJointData,
                                                          gainRatio);
      }

      lowLevelOneDoFJointDesiredDataHolder.completeWith(getStateSpecificJointSettings());

      externalControl.readDebugData();

      for (int i = 0; i < externalControl.solutionDebugData.numRows; i++)
      {
         debugData.get(externalControl.debugDataNames[i]).set(externalControl.solutionDebugData.get(i));
      }

   }

   @Override
   public void onExit(double timeInState)
   {
      commandInputManager.setEnabled(true);
   }

   @Override
   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }
}
