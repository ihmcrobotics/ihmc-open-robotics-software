package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.input;

import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KSTTools;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxParameters;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxCenterOfMassCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFramePoint;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFramePose3D;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

public class KSTInputFirstOrderStateEstimator implements KSTInputStateEstimator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final Map<RigidBodyReadOnly, SingleEndEffectorEstimator> inputPoseEstimators = new HashMap<>();
   private final SingleEndEffectorEstimator[] inputPoseEstimatorsArray;
   private final CenterOfMassEstimator centerOfMassEstimator;
   private final YoDouble inputVelocityDecayDuration = new YoDouble("inputVelocityDecayDuration", registry);
   private final YoDouble inputsFilterBreakFrequency = new YoDouble("inputsFilterBreakFrequency", registry);
   private final double updateDT;

   public KSTInputFirstOrderStateEstimator(Collection<? extends RigidBodyReadOnly> endEffectors,
                                           KinematicsStreamingToolboxParameters parameters,
                                           double updateDT,
                                           YoRegistry parentRegistry)
   {
      this.updateDT = updateDT;
      DoubleProvider inputsAlphaProvider = () -> AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(inputsFilterBreakFrequency.getValue(),
                                                                                                                 updateDT);

      for (RigidBodyReadOnly endEffector : endEffectors)
      {
         inputPoseEstimators.put(endEffector, new SingleEndEffectorEstimator(endEffector, inputsAlphaProvider, registry));
      }
      inputPoseEstimatorsArray = inputPoseEstimators.values().toArray(new SingleEndEffectorEstimator[0]);
      centerOfMassEstimator = new CenterOfMassEstimator(inputsAlphaProvider, registry);

      inputsFilterBreakFrequency.set(parameters.getInputPoseLPFBreakFrequency());
      inputVelocityDecayDuration.set(parameters.getInputVelocityDecayDuration());

      parentRegistry.addChild(registry);
   }

   @Override
   public void reset()
   {
      for (SingleEndEffectorEstimator inputPoseEstimator : inputPoseEstimatorsArray)
      {
         inputPoseEstimator.reset();
      }
      centerOfMassEstimator.reset();
   }

   /**
    * Update the state estimator with the latest input command.
    * <p>
    * If the input command is new, the state estimator will update, otherwise it will extrapolate the input.
    * </p>
    *
    * @param time
    * @param isNewInput                   whether the input command is new or not.
    * @param defaultLinearRateLimitation  default limit to use for the linear velocity.
    * @param defaultAngularRateLimitation default limit to use for the angular velocity.
    * @param latestInputCommand           the latest input command. Modified when {@code isNewInput} is {@code false}.
    */
   @Override
   public void update(double time,
                      boolean isNewInput,
                      double defaultLinearRateLimitation,
                      double defaultAngularRateLimitation,
                      KinematicsStreamingToolboxInputCommand latestInputCommand,
                      KinematicsStreamingToolboxInputCommand previousRawInputCommand)
   {
      updateVelocity(isNewInput, latestInputCommand, previousRawInputCommand);
      updatePose(isNewInput, latestInputCommand);
   }

   @Override
   public FramePose3DReadOnly getEstimatedPose(RigidBodyReadOnly endEffector)
   {
      SingleEndEffectorEstimator inputPoseEstimator = inputPoseEstimators.get(endEffector);
      if (inputPoseEstimator == null)
         return null;

      return inputPoseEstimator.getOutputPose();
   }

   @Override
   public FramePoint3DReadOnly getEstimatedCoMPosition()
   {
      return centerOfMassEstimator.getOutputPosition();
   }

   private void updatePose(boolean isNewInput, KinematicsStreamingToolboxInputCommand inputCommandToFilter)
   {
      if (isNewInput)
      {
         for (int i = 0; i < inputCommandToFilter.getNumberOfInputs(); i++)
         { // This is just for viz purpose
            KinematicsToolboxRigidBodyCommand input = inputCommandToFilter.getInput(i);

            SingleEndEffectorEstimator inputPoseEstimator = inputPoseEstimators.get(input.getEndEffector());
            if (inputPoseEstimator != null)
               inputPoseEstimator.updateInput(input.getDesiredPose());
         }

         if (inputCommandToFilter.hasCenterOfMassInput())
            centerOfMassEstimator.updateInput(inputCommandToFilter.getCenterOfMassInput().getDesiredPosition());
      }
      else
      {
         for (int i = 0; i < inputCommandToFilter.getNumberOfInputs(); i++)
         {
            KinematicsToolboxRigidBodyCommand input = inputCommandToFilter.getInput(i);
            SingleEndEffectorEstimator inputPoseEstimator = inputPoseEstimators.get(input.getEndEffector());

            if (inputPoseEstimator != null)
               inputPoseEstimator.extrapolateInput(updateDT);
         }

         if (inputCommandToFilter.hasCenterOfMassInput())
            centerOfMassEstimator.extrapolateInput(updateDT);
      }
   }

   private void updateVelocity(boolean isNewInput,
                               KinematicsStreamingToolboxInputCommand currentRawInputCommand,
                               KinematicsStreamingToolboxInputCommand previousRawInputCommand)
   {
      // First update velocities
      if (previousRawInputCommand == null)
      { // Can't compute velocity, resetting
         for (SingleEndEffectorEstimator singleEndEffectorEstimator : inputPoseEstimatorsArray)
         {
            singleEndEffectorEstimator.resetVelocity();
         }
         centerOfMassEstimator.resetVelocity();
      }
      else if (isNewInput)
      { // New input, let's update the velocities
         double timeInterval = Conversions.nanosecondsToSeconds(currentRawInputCommand.getTimestamp() - previousRawInputCommand.getTimestamp());

         for (int i = 0; i < currentRawInputCommand.getNumberOfInputs(); i++)
         {
            KinematicsToolboxRigidBodyCommand currentInput = currentRawInputCommand.getInput(i);

            SingleEndEffectorEstimator inputPoseEstimator = inputPoseEstimators.get(currentInput.getEndEffector());
            if (inputPoseEstimator == null)
               continue;

            KinematicsToolboxRigidBodyCommand previousInput = previousRawInputCommand.getInputFor(currentInput.getEndEffector());
            if (previousInput == null)
            { // Can't compute velocity, resetting
               inputPoseEstimator.resetVelocity();
               continue;
            }

            inputPoseEstimator.estimateVelocity(timeInterval, previousInput.getDesiredPose(), currentInput.getDesiredPose());
         }

         // Let's loop through the previous inputs to see if there are any that are not in the current input
         for (int i = 0; i < previousRawInputCommand.getNumberOfInputs(); i++)
         {
            KinematicsToolboxRigidBodyCommand previousInput = previousRawInputCommand.getInput(i);
            if (currentRawInputCommand.getInputFor(previousInput.getEndEffector()) == null)
            {
               SingleEndEffectorEstimator inputPoseEstimator = inputPoseEstimators.get(previousInput.getEndEffector());
               if (inputPoseEstimator != null)
                  inputPoseEstimator.resetVelocity();
            }
         }

         if (currentRawInputCommand.hasCenterOfMassInput() && previousRawInputCommand.hasCenterOfMassInput())
         {
            KinematicsToolboxCenterOfMassCommand currentCoMInput = currentRawInputCommand.getCenterOfMassInput();
            KinematicsToolboxCenterOfMassCommand previousCoMInput = previousRawInputCommand.getCenterOfMassInput();
            centerOfMassEstimator.estimateVelocity(timeInterval, previousCoMInput.getDesiredPosition(), currentCoMInput.getDesiredPosition());
         }
         else
         {
            centerOfMassEstimator.resetVelocity();
         }
      }
      else
      { // No new input, let's decay the velocities
         for (SingleEndEffectorEstimator singleEndEffectorEstimator : inputPoseEstimatorsArray)
         {
            singleEndEffectorEstimator.decayInputVelocity(updateDT / inputVelocityDecayDuration.getValue());
         }
         centerOfMassEstimator.decayInputVelocity(updateDT / inputVelocityDecayDuration.getValue());
      }
   }

   private static class SingleEndEffectorEstimator
   {
      private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      private final YoFramePose3D rawInputPose;
      private final YoFramePose3D rawExtrapolatedInputPose;
      private final AlphaFilteredYoFramePose3D filteredExtrapolatedInputPose;

      private final YoFixedFrameSpatialVector rawInputSpatialVelocity;
      private final YoFixedFrameSpatialVector decayingInputSpatialVelocity;
      private final YoDouble inputVelocityDecayFactor;

      public SingleEndEffectorEstimator(RigidBodyReadOnly endEffector, DoubleProvider inputsAlphaProvider, YoRegistry registry)
      {
         String namePrefix = endEffector.getName() + "Input_FOF_";

         rawInputPose = new YoFramePose3D(new YoFramePoint3D(namePrefix + "RawPosition", worldFrame, registry),
                                          new YoFrameQuaternion(namePrefix + "RawOrientation", worldFrame, registry));
         rawExtrapolatedInputPose = new YoFramePose3D(new YoFramePoint3D(namePrefix + "RawExtrapolatedPosition", worldFrame, registry),
                                                      new YoFrameQuaternion(namePrefix + "RawExtrapolatedOrientation", worldFrame, registry));
         filteredExtrapolatedInputPose = new AlphaFilteredYoFramePose3D(new YoFramePoint3D(namePrefix + "FilteredExtrapolatedPosition", worldFrame, registry),
                                                                        new YoFrameQuaternion(namePrefix + "FilteredExtrapolatedOrientation",
                                                                                              worldFrame,
                                                                                              registry),
                                                                        rawExtrapolatedInputPose,
                                                                        inputsAlphaProvider,
                                                                        registry);

         rawInputSpatialVelocity = new YoFixedFrameSpatialVector(new YoFrameVector3D(namePrefix + "RawAngularVelocity", worldFrame, registry),
                                                                 new YoFrameVector3D(namePrefix + "RawLinearVelocity", worldFrame, registry));
         decayingInputSpatialVelocity = new YoFixedFrameSpatialVector(new YoFrameVector3D(namePrefix + "DecayingAngularVelocity", worldFrame, registry),
                                                                      new YoFrameVector3D(namePrefix + "DecayingLinearVelocity", worldFrame, registry));
         inputVelocityDecayFactor = new YoDouble(namePrefix + "InputVelocityDecayFactor", registry);
      }

      public void reset()
      {
         rawInputPose.setToNaN();
         rawExtrapolatedInputPose.setToNaN();
         filteredExtrapolatedInputPose.setToNaN();
         filteredExtrapolatedInputPose.reset();

         resetVelocity();
      }

      public void resetVelocity()
      {
         rawInputSpatialVelocity.setToZero();
         decayingInputSpatialVelocity.setToZero();
         inputVelocityDecayFactor.set(0.0);
      }

      public void updateInput(FramePose3DReadOnly pose)
      {
         rawInputPose.set(pose);
         rawExtrapolatedInputPose.set(pose);
         filteredExtrapolatedInputPose.update();
      }

      public void extrapolateInput(double integrationDT)
      {
         KSTTools.integrateLinearVelocity(integrationDT,
                                          rawExtrapolatedInputPose.getPosition(),
                                          decayingInputSpatialVelocity.getLinearPart(),
                                          rawExtrapolatedInputPose.getPosition());
         KSTTools.integrateAngularVelocity(integrationDT,
                                           rawExtrapolatedInputPose.getOrientation(),
                                           decayingInputSpatialVelocity.getAngularPart(),
                                           false,
                                           rawExtrapolatedInputPose.getOrientation());
         filteredExtrapolatedInputPose.update();
      }

      public void decayInputVelocity(double decayIncrement)
      {
         double alpha = Math.min(1.0, inputVelocityDecayFactor.getDoubleValue() + decayIncrement);
         inputVelocityDecayFactor.set(alpha);
         decayingInputSpatialVelocity.getLinearPart().interpolate(rawInputSpatialVelocity.getLinearPart(), EuclidCoreTools.zeroVector3D, alpha);
         decayingInputSpatialVelocity.getAngularPart().interpolate(rawInputSpatialVelocity.getAngularPart(), EuclidCoreTools.zeroVector3D, alpha);
      }

      public void estimateVelocity(double dt, FramePose3DReadOnly previousPose, FramePose3DReadOnly currentPose)
      {
         /*
          * We compute the velocity of the inputs and then do a 1st-order extrapolation in the future and
          * update the raw input. This way, if for the next control tick we didn't get any new inputs, the IK
          * keep moving which in result should improve continuity of any motion.
          */
         KSTTools.computeSpatialVelocity(dt, previousPose, currentPose, rawInputSpatialVelocity);
         decayingInputSpatialVelocity.set(rawInputSpatialVelocity);
         inputVelocityDecayFactor.set(0.0);
      }

      public FramePose3DReadOnly getOutputPose()
      {
         return filteredExtrapolatedInputPose;
      }
   }

   private static class CenterOfMassEstimator
   {
      private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      private final YoFramePoint3D rawInputPosition;
      private final YoFramePoint3D rawExtrapolatedInputPosition;
      private final AlphaFilteredYoFramePoint filteredExtrapolatedInputPosition;

      private final YoFrameVector3D rawInputLinearVelocity;
      private final YoFrameVector3D decayingInputLinearVelocity;
      private final YoDouble inputVelocityDecayFactor;

      public CenterOfMassEstimator(DoubleProvider inputsAlphaProvider, YoRegistry registry)
      {
         rawInputPosition = new YoFramePoint3D("CoMInput_FOF_RawPosition", worldFrame, registry);
         rawExtrapolatedInputPosition = new YoFramePoint3D("CoMInput_FOF_RawExtrapolatedPosition", worldFrame, registry);
         filteredExtrapolatedInputPosition = new AlphaFilteredYoFramePoint("CoMInput_FOF_FilteredExtrapolatedPosition",
                                                                           "",
                                                                           registry,
                                                                           inputsAlphaProvider,
                                                                           rawExtrapolatedInputPosition);
         rawInputLinearVelocity = new YoFrameVector3D("CoMInput_FOF_RawLinearVelocity", worldFrame, registry);
         decayingInputLinearVelocity = new YoFrameVector3D("CoMInput_FOF_DecayingLinearVelocity", worldFrame, registry);
         inputVelocityDecayFactor = new YoDouble("CoMInput_FOF_InputVelocityDecayFactor", registry);
      }

      public void reset()
      {
         rawInputPosition.setToNaN();
         rawExtrapolatedInputPosition.setToNaN();
         filteredExtrapolatedInputPosition.setToNaN();
         filteredExtrapolatedInputPosition.reset();

         resetVelocity();
      }

      private void resetVelocity()
      {
         rawInputLinearVelocity.setToZero();
         decayingInputLinearVelocity.setToZero();
         inputVelocityDecayFactor.set(0.0);
      }

      public void updateInput(FramePoint3DReadOnly position)
      {
         rawInputPosition.set(position);
         rawExtrapolatedInputPosition.set(position);
         filteredExtrapolatedInputPosition.update();
      }

      public void extrapolateInput(double integrationDT)
      {
         KSTTools.integrateLinearVelocity(integrationDT, rawInputPosition, rawInputLinearVelocity, rawExtrapolatedInputPosition);
         filteredExtrapolatedInputPosition.update();
      }

      public void decayInputVelocity(double decayIncrement)
      {
         double alpha = Math.min(1.0, inputVelocityDecayFactor.getDoubleValue() + decayIncrement);
         inputVelocityDecayFactor.set(alpha);
         decayingInputLinearVelocity.interpolate(rawInputLinearVelocity, EuclidCoreTools.zeroVector3D, alpha);
      }

      public void estimateVelocity(double dt, FramePoint3DReadOnly previousPosition, FramePoint3DReadOnly currentPosition)
      {
         KSTTools.computeLinearVelocity(dt, previousPosition, currentPosition, rawInputLinearVelocity);
         decayingInputLinearVelocity.set(rawInputLinearVelocity);
         inputVelocityDecayFactor.set(0.0);
      }

      public FramePoint3DReadOnly getOutputPosition()
      {
         return rawInputPosition;
      }
   }
}
