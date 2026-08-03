package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import com.google.common.base.CaseFormat;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commons.MathTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.trajectories.yoVariables.YoPolynomial;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.commons.lists.PairList;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class GroundPrepControllerState extends HighLevelControllerState
{
   private static final HighLevelControllerName controllerState = HighLevelControllerName.GROUND_PREP_STATE;

   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   private final PairList<OneDoFJointBasics, TrajectoryData> jointsData = new PairList<>();

   private final YoPolynomial trajectory;

   private final YoBoolean reinitialize = new YoBoolean("groundPrepReinitialize", registry);
   private final YoBoolean continuousUpdate = new YoBoolean("groundPrepContinuousUpdate", registry);
   private final YoDouble splineStartTime = new YoDouble("groundPrepSplineStartTime", registry);
   private final YoDouble timeToPrepareForGround = new YoDouble("timeToPrepareForGround", registry);
   private final YoDouble minimumTimeDoneWithGroundPrep = new YoDouble("minimumTimeDoneWithGroundPrep", registry);
   private final JointDesiredOutputListReadOnly highLevelControlOutput;

   private final DoubleProvider timeProvider;

   public GroundPrepControllerState(OneDoFJointBasics[] controlledJoints,
                                    HighLevelControllerParameters highLevelControllerParameters,
                                    JointDesiredOutputListReadOnly highLevelControlOutput,
                                    DoubleProvider timeProvider)
   {
      super(controllerState, highLevelControllerParameters, controlledJoints);
      this.highLevelControlOutput = highLevelControlOutput;
      this.timeProvider = timeProvider;

      this.timeToPrepareForGround.set(highLevelControllerParameters.getTimeToMoveInStandPrep());

      WholeBodySetpointParameters groundPrepParameters = highLevelControllerParameters.getGroundPrepParameters();
      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledJoints);

      trajectory = new YoPolynomial("GroundPrepTrajectory", 4, registry);

      for (OneDoFJointBasics controlledJoint : controlledJoints)
      {
         String jointName = controlledJoint.getName();
         String namePrefix = CaseFormat.UPPER_UNDERSCORE.to(CaseFormat.LOWER_CAMEL, jointName);

         YoDouble groundPrepInitialConfiguration = new YoDouble(namePrefix + "GroundPrepStartPosition", registry);
         DoubleProvider groundPrepFinalConfiguration = new DoubleParameter(namePrefix + "GroundPrepFinalPosition",
                                                                           registry,
                                                                           groundPrepParameters.getSetpoint(jointName));
         YoDouble groundPrepDesiredConfiguration = new YoDouble(namePrefix + "GroundPrepCurrent", registry);
         YoDouble groundPrepDesiredVelocity = new YoDouble(namePrefix + "GroundPrepCurrentVelocity", registry);

         TrajectoryData jointData = new TrajectoryData(groundPrepInitialConfiguration,
                                                       groundPrepFinalConfiguration,
                                                       groundPrepDesiredConfiguration,
                                                       groundPrepDesiredVelocity);
         jointsData.add(controlledJoint, jointData);
      }
   }

   public void setMinimumTimeDoneWithGroundPrep(double minimumTimeDoneWithGroundPrep)
   {
      this.minimumTimeDoneWithGroundPrep.set(minimumTimeDoneWithGroundPrep);
   }

   @Override
   public void onEntry()
   {
      continuousUpdate.set(false);
      reinitialize.set(false);
      if (timeProvider != null)
         initializeSplines(timeProvider.getValue());
      else
         initializeSplines(0.0);
   }

   public void initializeSplines(double startTime)
   {
      splineStartTime.set(startTime);

      for (int jointIndex = 0; jointIndex < jointsData.size(); jointIndex++)
      {
         OneDoFJointBasics joint = jointsData.get(jointIndex).getLeft();
         TrajectoryData trajectoryData = jointsData.get(jointIndex).getRight();

         JointDesiredOutputReadOnly jointDesiredOutput = highLevelControlOutput.getJointDesiredOutput(joint);
         double startAngle;
         if (jointDesiredOutput != null && jointDesiredOutput.hasDesiredPosition())
            startAngle = jointDesiredOutput.getDesiredPosition();
         else
            startAngle = joint.getQ();

         trajectoryData.getInitialJointConfiguration().set(startAngle);
      }

      trajectory.setCubic(0.0, timeToPrepareForGround.getDoubleValue(), 0, 0, 1, 0);
   }

   @Override
   public void doAction(double timeInState)
   {
      double time;
      if (timeProvider != null)
         time = timeProvider.getValue();
      else
         time = timeInState;

      if (continuousUpdate.getValue())
      {
         reinitialize.set(false);
         initializeSplines(splineStartTime.getValue());
      }
      else if (reinitialize.getValue())
      {
         reinitialize.set(false);
         initializeSplines(time);
      }

      double timeInTrajectory = MathTools.clamp(time - splineStartTime.getValue(), 0.0, timeToPrepareForGround.getDoubleValue());

      trajectory.compute(timeInTrajectory);
      double alphaPosition = trajectory.getValue();
      double alphaVelocity = trajectory.getVelocity();

      for (int jointIndex = 0; jointIndex < jointsData.size(); jointIndex++)
      {
         OneDoFJointBasics joint = jointsData.get(jointIndex).getLeft();
         TrajectoryData trajectoryData = jointsData.get(jointIndex).getRight();

         double q_initial = trajectoryData.getInitialJointConfiguration().getValue();
         double q_final = trajectoryData.getFinalJointConfiguration().getValue();

         JointDesiredOutputBasics lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(joint);
         lowLevelJointData.clear();

         if (timeInTrajectory < timeToPrepareForGround.getDoubleValue())
         {
            double jointPosition = ((1 - alphaPosition) * q_initial) + (alphaPosition * q_final);
            double jointVelocity = alphaVelocity * (q_final - q_initial);

            trajectoryData.getDesiredJointConfiguration().set(jointPosition);
            trajectoryData.getDesiredJointVelocity().set(jointVelocity);
         }

         lowLevelJointData.setDesiredPosition(trajectoryData.getDesiredJointConfiguration().getValue());
         lowLevelJointData.setDesiredVelocity(trajectoryData.getDesiredJointVelocity().getValue());
      }

      lowLevelOneDoFJointDesiredDataHolder.completeWith(getStateSpecificJointSettings());
   }

   @Override
   public void onExit(double timeInState)
   {
      // Do nothing
   }

   @Override
   public boolean isDone(double timeInState)
   {
      return timeInState > (timeToPrepareForGround.getDoubleValue() + minimumTimeDoneWithGroundPrep.getDoubleValue());
   }

   @Override
   public LowLevelOneDoFJointDesiredDataHolder getOutputForLowLevelController()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }

   private class TrajectoryData
   {
      private final YoDouble initialJointConfiguration;
      private final DoubleProvider finalJointConfiguration;
      private final YoDouble desiredJointConfiguration;
      private final YoDouble desiredJointVelocity;

      public TrajectoryData(YoDouble initialJointConfiguration,
                            DoubleProvider finalJointConfiguration,
                            YoDouble desiredJointConfiguration,
                            YoDouble desiredJointVelocity)
      {
         this.initialJointConfiguration = initialJointConfiguration;
         this.finalJointConfiguration = finalJointConfiguration;
         this.desiredJointConfiguration = desiredJointConfiguration;
         this.desiredJointVelocity = desiredJointVelocity;
      }

      public YoDouble getInitialJointConfiguration()
      {
         return initialJointConfiguration;
      }

      public DoubleProvider getFinalJointConfiguration()
      {
         return finalJointConfiguration;
      }

      public YoDouble getDesiredJointConfiguration()
      {
         return desiredJointConfiguration;
      }

      public YoDouble getDesiredJointVelocity()
      {
         return desiredJointVelocity;
      }
   }
}
