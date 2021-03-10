package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import com.google.common.base.CaseFormat;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.math.trajectories.yoVariables.YoPolynomial;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class StandPrepControllerState extends HighLevelControllerState
{
   private static final HighLevelControllerName controllerState = HighLevelControllerName.STAND_PREP_STATE;
   private static final double MINIMUM_TIME_DONE_WITH_STAND_PREP = 0.0;

   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   private final PairList<OneDoFJointBasics, TrajectoryData> jointsData = new PairList<>();

   private final YoBoolean reinitialize = new YoBoolean("standPrepReinitialize", registry);
   private final YoBoolean continuousUpdate = new YoBoolean("standPrepContinuousUpdate", registry);
   private final YoDouble splineStartTime = new YoDouble("standPrepSplineStartTime", registry);
   private final YoDouble timeToPrepareForStanding = new YoDouble("timeToPrepareForStanding", registry);
   private final YoDouble minimumTimeDoneWithStandPrep = new YoDouble("minimumTimeDoneWithStandPrep", registry);
   private final JointDesiredOutputListReadOnly highLevelControlOutput;

   private final YoPolynomial transitionRatioTrajectory = new YoPolynomial("StandPrepBlendingTrajectory", 4, registry);


   public StandPrepControllerState(OneDoFJointBasics[] controlledJoints, HighLevelControllerParameters highLevelControllerParameters,
                                   JointDesiredOutputListReadOnly highLevelControlOutput)
   {
      this(controlledJoints, highLevelControllerParameters, highLevelControlOutput, MINIMUM_TIME_DONE_WITH_STAND_PREP);
   }

   public StandPrepControllerState(OneDoFJointBasics[] controlledJoints, HighLevelControllerParameters highLevelControllerParameters,
                                   JointDesiredOutputListReadOnly highLevelControlOutput, double minimumTimeDoneWithStandPrep)
   {
      super(controllerState, highLevelControllerParameters, controlledJoints);
      this.highLevelControlOutput = highLevelControlOutput;

      this.timeToPrepareForStanding.set(highLevelControllerParameters.getTimeToMoveInStandPrep());
      this.minimumTimeDoneWithStandPrep.set(minimumTimeDoneWithStandPrep);

      WholeBodySetpointParameters standPrepParameters = highLevelControllerParameters.getStandPrepParameters();
      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledJoints);

      for (OneDoFJointBasics controlledJoint : controlledJoints)
      {
         String jointName = controlledJoint.getName();
         String namePrefix = CaseFormat.UPPER_UNDERSCORE.to(CaseFormat.LOWER_CAMEL, jointName);

         DoubleParameter standPrepFinalConfiguration = new DoubleParameter(namePrefix + "StandPrepPosition",
                                                                          registry,
                                                                          standPrepParameters.getSetpoint(jointName));
         YoDouble standPrepDesiredConfiguration = new YoDouble(namePrefix + "StandPrepCurrentDesired", registry);
         YoDouble standPrepInitialConfiguration = new YoDouble(namePrefix + "StandPrepInitialPosition", registry);

         standPrepFinalConfiguration.addListener(v -> reinitialize.set(true));

         TrajectoryData jointData = new TrajectoryData(standPrepInitialConfiguration, standPrepFinalConfiguration, standPrepDesiredConfiguration);
         jointsData.add(controlledJoint, jointData);
      }

   }

   @Override
   public void onEntry()
   {
      continuousUpdate.set(false);
      reinitialize.set(false);

      initializeBlendingSpline(0.0);
      initializeSplineBoundaries();
   }

   private void initializeBlendingSpline(double startTime)
   {
      splineStartTime.set(startTime);

      transitionRatioTrajectory.setCubic(0.0, timeToPrepareForStanding.getDoubleValue(), 0.0, 0.0, 1.0, 0.0);
   }

   private void initializeSplineBoundaries()
   {
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

         trajectoryData.getDesiredJointConfiguration().set(startAngle);
      }
   }

   @Override
   public void doAction(double timeInState)
   {
      if (continuousUpdate.getValue())
      {
         reinitialize.set(false);
         initializeBlendingSpline(splineStartTime.getValue());
         initializeSplineBoundaries();
      }
      else if (reinitialize.getValue())
      {
         reinitialize.set(false);
         initializeBlendingSpline(timeInState);
         initializeSplineBoundaries();
      }

      double timeInTrajectory = MathTools.clamp(timeInState - splineStartTime.getValue(), 0.0, timeToPrepareForStanding.getDoubleValue());
      transitionRatioTrajectory.compute(timeInTrajectory);

      double blendingAlpha = transitionRatioTrajectory.getValue();
      double blendingRate = transitionRatioTrajectory.getVelocity();
      double blendingAcceleration = transitionRatioTrajectory.getAcceleration();

      for (int jointIndex = 0; jointIndex < jointsData.size(); jointIndex++)
      {
         OneDoFJointBasics joint = jointsData.get(jointIndex).getLeft();
         TrajectoryData trajectoryData = jointsData.get(jointIndex).getRight();

         YoDouble desiredPosition = trajectoryData.getDesiredJointConfiguration();

         double initialPosition = trajectoryData.getInitialJointConfiguration().getDoubleValue();
         double finalPosition = trajectoryData.getFinalJointConfiguration().getValue();
         double positionDelta = finalPosition - initialPosition;

         desiredPosition.set(InterpolationTools.linearInterpolate(initialPosition, finalPosition, blendingAlpha));

         // TODO check these values
         double desiredVelocity = positionDelta * blendingRate;
         double desiredAcceleration = positionDelta * blendingAcceleration;

         JointDesiredOutputBasics lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(joint);
         lowLevelJointData.clear();
         lowLevelJointData.setDesiredPosition(desiredPosition.getDoubleValue());
         lowLevelJointData.setDesiredVelocity(desiredVelocity);
         lowLevelJointData.setDesiredAcceleration(desiredAcceleration);
      }

      lowLevelOneDoFJointDesiredDataHolder.completeWith(getStateSpecificJointSettings());
   }

   @Override
   public void onExit()
   {
      // Do nothing

   }

   @Override
   public boolean isDone(double timeInState)
   {
      return timeInState > (timeToPrepareForStanding.getDoubleValue() + minimumTimeDoneWithStandPrep.getDoubleValue());
   }

   @Override
   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }

   private class TrajectoryData
   {
      private final YoDouble initialJointConfiguration;
      private final DoubleParameter finalJointConfiguration;
      private final YoDouble desiredJointConfiguration;

      public TrajectoryData(YoDouble initialJointConfiguration, DoubleParameter finalJointConfiguration, YoDouble desiredJointConfiguration)
      {
         this.initialJointConfiguration = initialJointConfiguration;
         this.finalJointConfiguration = finalJointConfiguration;
         this.desiredJointConfiguration = desiredJointConfiguration;
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
   }

}
