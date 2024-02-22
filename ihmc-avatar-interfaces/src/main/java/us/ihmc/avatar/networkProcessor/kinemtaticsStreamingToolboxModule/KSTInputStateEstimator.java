package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

public class KSTInputStateEstimator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final Map<RigidBodyBasics, KSTSingleInputPoseEstimator> inputPoseEstimators = new HashMap<>();
   private final KSTSingleInputPoseEstimator[] inputPoseEstimatorsArray;
   private final YoDouble inputVelocityDecayDuration = new YoDouble("inputVelocityDecayDuration", registry);
   private final YoDouble inputsFilterBreakFrequency = new YoDouble("inputsFilterBreakFrequency", registry);
   private final double updateDT;

   public KSTInputStateEstimator(Collection<? extends RigidBodyBasics> endEffectors, double updateDT, YoRegistry parentRegistry)
   {
      this.updateDT = updateDT;
      DoubleProvider inputsAlphaProvider = () -> AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(inputsFilterBreakFrequency.getValue(),
                                                                                                                 updateDT);

      for (RigidBodyBasics endEffector : endEffectors)
      {
         inputPoseEstimators.put(endEffector, new KSTSingleInputPoseEstimator(endEffector, inputsAlphaProvider, registry));
      }
      inputPoseEstimatorsArray = new KSTSingleInputPoseEstimator[endEffectors.size()];

      int index = 0;

      for (RigidBodyBasics rigidBody : endEffectors)
      {
         inputPoseEstimatorsArray[index++] = inputPoseEstimators.get(rigidBody);
      }

      parentRegistry.addChild(registry);
   }

   public void setInputFilterBreakFrequency(double breakFrequency)
   {
      inputsFilterBreakFrequency.set(breakFrequency);
   }

   public void setInputVelocityDecayDuration(double decayDuration)
   {
      inputVelocityDecayDuration.set(decayDuration);
   }

   public void reset()
   {
      for (KSTSingleInputPoseEstimator inputPoseEstimator : inputPoseEstimatorsArray)
      {
         inputPoseEstimator.reset();
      }
   }

   /**
    * Update the state estimator with the latest input command.
    * <p>
    * If the input command is new, the state estimator will update, otherwise it will extrapolate the input.
    * </p>
    *
    * @param isNewInput           whether the input command is new or not.
    * @param inputCommandToFilter the latest input command. Modified when {@code isNewInput} is {@code false}.
    */
   public void update(boolean isNewInput,
                      KinematicsStreamingToolboxInputCommand inputCommandToFilter,
                      KinematicsStreamingToolboxInputCommand currentRawInputCommand,
                      KinematicsStreamingToolboxInputCommand previousRawInputCommand)
   {
      updateVelocity(isNewInput, currentRawInputCommand, previousRawInputCommand);
      updatePose(isNewInput, inputCommandToFilter);
   }

   private void updatePose(boolean isNewInput, KinematicsStreamingToolboxInputCommand inputCommandToFilter)
   {
      if (isNewInput)
      {
         for (int i = 0; i < inputCommandToFilter.getNumberOfInputs(); i++)
         { // This is just for viz purpose
            KinematicsToolboxRigidBodyCommand input = inputCommandToFilter.getInput(i);

            KSTSingleInputPoseEstimator inputPoseEstimator = inputPoseEstimators.get(input.getEndEffector());
            if (inputPoseEstimator == null)
               continue;

            // TODO Simplify the following
            FramePose3D desiredPose = input.getDesiredPose();
            inputPoseEstimator.updateInput(desiredPose);
            desiredPose.getPosition().set(inputPoseEstimator.getFilteredExtrapolatedInputPosition());
            desiredPose.getOrientation().set(inputPoseEstimator.getFilteredExtrapolatedInputOrientation());
         }
      }
      else
      {
         for (int i = 0; i < inputCommandToFilter.getNumberOfInputs(); i++)
         {
            KinematicsToolboxRigidBodyCommand input = inputCommandToFilter.getInput(i);
            KSTSingleInputPoseEstimator inputPoseEstimator = inputPoseEstimators.get(input.getEndEffector());

            if (inputPoseEstimator == null)
               continue;

            FramePose3D desiredPose = input.getDesiredPose();
            inputPoseEstimator.extrapolateInput(updateDT);
            desiredPose.getPosition().set(inputPoseEstimator.getFilteredExtrapolatedInputPosition());
            desiredPose.getOrientation().set(inputPoseEstimator.getFilteredExtrapolatedInputOrientation());
         }
      }
   }

   private void updateVelocity(boolean isNewInput,
                               KinematicsStreamingToolboxInputCommand currentRawInputCommand,
                               KinematicsStreamingToolboxInputCommand previousRawInputCommand)
   {
      // First update velocities
      if (previousRawInputCommand == null)
      { // Can't compute velocity, resetting
         for (KSTSingleInputPoseEstimator kstSingleInputPoseEstimator : inputPoseEstimatorsArray)
         {
            kstSingleInputPoseEstimator.resetVelocity();
         }
      }
      else if (isNewInput)
      { // New input, let's update the velocities
         for (int i = 0; i < currentRawInputCommand.getNumberOfInputs(); i++)
         {
            KinematicsToolboxRigidBodyCommand currentInput = currentRawInputCommand.getInput(i);

            KSTSingleInputPoseEstimator inputPoseEstimator = inputPoseEstimators.get(currentInput.getEndEffector());
            if (inputPoseEstimator == null)
               continue;

            KinematicsToolboxRigidBodyCommand previousInput = previousRawInputCommand.getInputFor(currentInput.getEndEffector());
            if (previousInput == null)
            { // Can't compute velocity, resetting
               inputPoseEstimator.resetVelocity();
               continue;
            }

            inputPoseEstimator.estimateVelocity(updateDT, previousInput.getDesiredPose(), currentInput.getDesiredPose());
         }

         // Let's loop through the previous inputs to see if there are any that are not in the current input
         for (int i = 0; i < previousRawInputCommand.getNumberOfInputs(); i++)
         {
            KinematicsToolboxRigidBodyCommand previousInput = previousRawInputCommand.getInput(i);
            if (currentRawInputCommand.getInputFor(previousInput.getEndEffector()) == null)
            {
               KSTSingleInputPoseEstimator inputPoseEstimator = inputPoseEstimators.get(previousInput.getEndEffector());
               if (inputPoseEstimator != null)
                  inputPoseEstimator.resetVelocity();
            }
         }
      }
      else
      { // No new input, let's decay the velocities
         for (KSTSingleInputPoseEstimator kstSingleInputPoseEstimator : inputPoseEstimatorsArray)
         {
            kstSingleInputPoseEstimator.decayInputVelocity(updateDT / inputVelocityDecayDuration.getDoubleValue());
         }
      }
   }
}
