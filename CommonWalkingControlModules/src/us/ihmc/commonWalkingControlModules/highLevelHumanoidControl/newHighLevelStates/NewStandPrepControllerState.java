package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.HashMap;

public class NewStandPrepControllerState extends NewHighLevelControllerState
{
   private static final double TIME_TO_SPLINE_TO_STAND_POSE = 4.0;
   private static final double MINIMUM_TIME_DONE_WITH_STAND_PREP = 1.0;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.OFF);
   private final HashMap<InverseDynamicsJoint, YoPolynomial> trajectoriesStandPose = new HashMap<>();

   private final OneDoFJoint[] controlledJoints;
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();
   private final StandPrepParameters standPrepSetpoints;

   private final YoDouble timeToPrepareForStanding = new YoDouble("timeToPrepareForStanding", registry);
   private final YoDouble minimumTimeDoneWithStandPrep = new YoDouble("minimumTimeDoneWithStandPrep", registry);

   public NewStandPrepControllerState(HighLevelHumanoidControllerToolbox controllerToolbox, StandPrepParameters standPrepSetpoints)
   {
      this(controllerToolbox, standPrepSetpoints, TIME_TO_SPLINE_TO_STAND_POSE, MINIMUM_TIME_DONE_WITH_STAND_PREP);
   }

   public NewStandPrepControllerState(HighLevelHumanoidControllerToolbox controllerToolbox, StandPrepParameters standPrepSetpoints,
                                      double timeToPrepareForStanding, double minimumTimeDoneWithStandPrep)
   {
      super(NewHighLevelControllerStates.STAND_PREP_STATE);

      this.standPrepSetpoints = standPrepSetpoints;
      this.timeToPrepareForStanding.set(timeToPrepareForStanding);
      this.minimumTimeDoneWithStandPrep.set(minimumTimeDoneWithStandPrep);

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

         double desiredAngle = standPrepSetpoints.getSetpoint(joint.getName());
         double desiredVelocity = 0.0;

         double currentAngle = joint.getQ();
         double currentVelocity = joint.getQd();

         trajectory.setCubic(0.0, timeToPrepareForStanding.getDoubleValue(), currentAngle, currentVelocity, desiredAngle, desiredVelocity);
      }
   }

   @Override
   public void doAction()
   {
      double timeInTrajectory = MathTools.clamp(getTimeInCurrentState(), 0.0, timeToPrepareForStanding.getDoubleValue());

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

      controllerCoreCommand.completeLowLevelJointData(lowLevelOneDoFJointDesiredDataHolder);
   }

   @Override
   public void doTransitionOutOfAction()
   {
      // Do nothing

   }

   @Override
   public boolean isDone()
   {
      return getTimeInCurrentState() > (timeToPrepareForStanding.getDoubleValue() + minimumTimeDoneWithStandPrep.getDoubleValue());
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
