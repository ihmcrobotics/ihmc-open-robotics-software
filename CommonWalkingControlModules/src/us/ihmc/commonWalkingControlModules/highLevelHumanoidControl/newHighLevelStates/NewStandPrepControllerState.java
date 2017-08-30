package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelJointControlMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.HashMap;

public class NewStandPrepControllerState extends NewHighLevelControllerState
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.OFF);
   private final HashMap<InverseDynamicsJoint, YoPolynomial> trajectoriesStandPose = new HashMap<>();

   private final OneDoFJoint[] controlledJoints;
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();
   private final StandPrepSetpoints standPrepSetpoints;

   private static final double TIME_TO_SPLINE_TO_STAND_POSE = 4.0;

   public NewStandPrepControllerState(HighLevelHumanoidControllerToolbox controllerToolbox, StandPrepSetpoints standPrepSetpoints)
   {
      super(NewHighLevelControllerStates.STAND_PREP_STATE);

      this.standPrepSetpoints = standPrepSetpoints;

      controlledJoints = controllerToolbox.getFullRobotModel().getOneDoFJoints();

      for (InverseDynamicsJoint controlledJoint : controlledJoints)
      {
         String jointName = controlledJoint.getName();
         YoPolynomial trajectory = new YoPolynomial(jointName + "_StandPrepTrajectory", 4, registry);
         trajectoriesStandPose.put(controlledJoint, trajectory);
      }

      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledJoints);
   }

   @Override
   public void setControllerCoreOutput(ControllerCoreOutputReadOnly controllerCoreOutput)
   {
   }

   @Override
   public void doTransitionIntoAction()
   {
      for (int jointIndex = 0; jointIndex < controlledJoints.length; jointIndex++)
      {
         OneDoFJoint joint = controlledJoints[jointIndex];
         YoPolynomial trajectory = trajectoriesStandPose.get(joint);

         double desiredAngle = standPrepSetpoints.get(joint.getName());
         double desiredVelocity = 0.0;

         double currentAngle = joint.getQ();
         double currentVelocity = joint.getQd();

         trajectory.setCubic(0.0, TIME_TO_SPLINE_TO_STAND_POSE, currentAngle, currentVelocity, desiredAngle, desiredVelocity);
      }
   }

   @Override
   public void doAction()
   {
      double timeInTrajectory = MathTools.clamp(getTimeInCurrentState(), 0.0, TIME_TO_SPLINE_TO_STAND_POSE);
      boolean doneWithTrajectory = timeInTrajectory >= TIME_TO_SPLINE_TO_STAND_POSE;

      if (!doneWithTrajectory)
      {
         for (int jointIndex = 0; jointIndex < controlledJoints.length; jointIndex++)
         {
            OneDoFJoint joint = controlledJoints[jointIndex];
            YoPolynomial trajectory = trajectoriesStandPose.get(joint);
            trajectory.compute(timeInTrajectory);

            double qDesired = trajectory.getPosition();
            double qdDesired = trajectory.getVelocity();

            lowLevelOneDoFJointDesiredDataHolder.setDesiredJointPosition(joint, qDesired);
            lowLevelOneDoFJointDesiredDataHolder.setDesiredJointVelocity(joint, qdDesired);
         }
      }

      controllerCoreCommand.completeLowLevelJointData(lowLevelOneDoFJointDesiredDataHolder);
   }

   @Override
   public void doTransitionOutOfAction()
   {
      // Do nothing

   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public ControllerCoreCommand getControllerCoreCommand()
   {
      return controllerCoreCommand;
   }

}
