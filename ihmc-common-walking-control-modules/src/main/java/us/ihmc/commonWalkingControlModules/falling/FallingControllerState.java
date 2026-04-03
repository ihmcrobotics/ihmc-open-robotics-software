package us.ihmc.commonWalkingControlModules.falling;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotics.trajectories.yoVariables.YoPolynomial;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;

public class FallingControllerState extends HighLevelControllerState
{
   private static final HighLevelControllerName controllerState = HighLevelControllerName.FALLING_STATE;
   private static final double fallTransitionDuration = 0.25;

   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder;
   private final YoPolynomial trajectory = new YoPolynomial("fallingTrajectory", 4, registry);

   private final double[] initialJointPositions;
   private final double[] capturedJointPositions;
   private final double[] capturedJointVelocities;
   private final WholeBodySetpointParameters fallingSetpoints;

   public FallingControllerState(CommandInputManager commandInputManager,
                                 StatusMessageOutputManager statusOutputManager,
                                 HighLevelControlManagerFactory managerFactory,
                                 HighLevelHumanoidControllerToolbox controllerToolbox,
                                 HighLevelControllerParameters highLevelControllerParameters,
                                 WalkingControllerParameters walkingControllerParameters)
   {
      this(commandInputManager,
           statusOutputManager,
           managerFactory,
           controllerToolbox,
           highLevelControllerParameters,
           walkingControllerParameters,
           highLevelControllerParameters.getFallingControllerParameters());
   }

   public FallingControllerState(CommandInputManager commandInputManager,
                                 StatusMessageOutputManager statusOutputManager,
                                 HighLevelControlManagerFactory managerFactory,
                                 HighLevelHumanoidControllerToolbox controllerToolbox,
                                 HighLevelControllerParameters highLevelControllerParameters,
                                 WalkingControllerParameters walkingControllerParameters,
                                 WholeBodySetpointParameters fallingSetpoints)
   {
      super(controllerState, highLevelControllerParameters, controllerToolbox.getControlledOneDoFJoints());
      lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder(controlledJoints.length);
      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledJoints);

      initialJointPositions = new double[controlledJoints.length];
      capturedJointPositions = new double[controlledJoints.length];
      capturedJointVelocities = new double[controlledJoints.length];
      this.fallingSetpoints = fallingSetpoints;
   }

   @Override
   public void doAction(double timeInState)
   {
      double timeInTrajectory = MathTools.clamp(timeInState, 0.0, fallTransitionDuration);
      trajectory.compute(timeInTrajectory);
      double alphaPosition = trajectory.getValue();
      double alphaVelocity = trajectory.getVelocity();

      for (int i = 0; i < controlledJoints.length; i++)
      {
         controlledJoints[i].setTau(0.0);
         lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(controlledJoints[i]).clear();
         lowLevelOneDoFJointDesiredDataHolder.setDesiredJointTorque(controlledJoints[i], 0.0);

         double desiredPosition = (1.0 - alphaPosition) * initialJointPositions[i] + alphaPosition * capturedJointPositions[i];
         double desiredVelocity = alphaVelocity * (capturedJointPositions[i] - initialJointPositions[i]);

         lowLevelOneDoFJointDesiredDataHolder.setDesiredJointPosition(controlledJoints[i], desiredPosition);
         lowLevelOneDoFJointDesiredDataHolder.setDesiredJointVelocity(controlledJoints[i], desiredVelocity);
      }

      lowLevelOneDoFJointDesiredDataHolder.completeWith(getStateSpecificJointSettings());
   }

   @Override
   public void onEntry()
   {
      trajectory.setCubic(0.0, fallTransitionDuration, 0.0, 0.0, 1.0, 0.0);

      for (int i = 0; i < controlledJoints.length; i++)
      {
         initialJointPositions[i] = controlledJoints[i].getQ();

         if (fallingSetpoints != null)
         {
            capturedJointPositions[i] = fallingSetpoints.getSetpoint(controlledJoints[i].getName());
            capturedJointVelocities[i] = 0.0;
         }
         else
         {
            capturedJointPositions[i] = controlledJoints[i].getQ();
            capturedJointVelocities[i] = controlledJoints[i].getQd();
         }
      }
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
