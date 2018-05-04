package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParametersReadOnly;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointAccelerationIntegrationCalculator;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

/**
 * {@link SpatialFeedbackControlCommand} is a command meant to be submitted to the
 * {@link WholeBodyControllerCore} via the {@link ControllerCoreCommand}.
 * <p>
 * The {@link JointAccelerationIntegrationCalculator} uses these commands to properly setup joint
 * acceleration integration whether it is for such or such joint and what gains should be used.
 * </p>
 * <p>
 * When enabled for a joint, the {@code JointAccelerationIntegrationCalculator} will perform a
 * double step integration off of the desired joint acceleration to first compute the desired joint
 * velocity and finally the desired joint position.
 * </p>
 * 
 * @author Sylvain Bertrand
 *
 */
public class JointAccelerationIntegrationCommand implements InverseDynamicsCommand<JointAccelerationIntegrationCommand>
{
   private final int initialCapacity = 15;
   private final List<String> jointNamesToComputeDesiredPositionFor = new ArrayList<>(initialCapacity);
   private final List<OneDoFJoint> jointsToComputeDesiredPositionFor = new ArrayList<>(initialCapacity);
   private final RecyclingArrayList<JointAccelerationIntegrationParameters> jointParameters = new RecyclingArrayList<>(initialCapacity,
                                                                                                                       JointAccelerationIntegrationParameters.class);

   /**
    * Creates an empty command. It needs to be configured before being submitted to the controller
    * core.
    */
   public JointAccelerationIntegrationCommand()
   {
      jointParameters.clear();
   }

   /**
    * Clear the internal memory of this command such that it is empty after calling this method.
    * <p>
    * This operation does cause garbage generation now nor later.
    * </p>
    */
   public void clear()
   {
      jointNamesToComputeDesiredPositionFor.clear();
      jointsToComputeDesiredPositionFor.clear();
      jointParameters.clear();
   }

   /**
    * Register an additional joint the {@link JointAccelerationIntegrationCalculator} should compute
    * the desired position of.
    * <p>
    * Optional: Specific integration parameters can be provided via
    * {@link #setJointAlphas(int, double, double)} and {@link #setJointMaxima(int, double, double)}.
    * If not provided, the calculator will use a default set.
    * </p>
    * 
    * @param joint the joint for which the desired acceleration is to be integrated to desired
    *           velocity and desired acceleration.
    */
   public void addJointToComputeDesiredPositionFor(OneDoFJoint joint)
   {
      jointNamesToComputeDesiredPositionFor.add(joint.getName());
      jointsToComputeDesiredPositionFor.add(joint);
      jointParameters.add().reset();
   }

   /**
    * Sets the acceleration integration parameters for the {@code jointIndex}<sup>th</sup> of this
    * command to {@code jointParameters}.
    * 
    * @param jointIndex the index of the joint to provide parameters for.
    * @param jointParameters the set of parameters to use for the joint. Not modified.
    */
   public void setJointParameters(int jointIndex, JointAccelerationIntegrationParametersReadOnly jointParameters)
   {
      this.jointParameters.get(jointIndex).set(jointParameters);
   }

   /**
    * Provides to the {@link JointAccelerationIntegrationCalculator} specific parameter values for
    * the {@code jointIndex}<sup>th</sup> of this command.
    * <p>
    * For the usage of these parameters see<br>
    * {@link JointAccelerationIntegrationParametersReadOnly#getPositionBreakFrequency()}<br>
    * {@link JointAccelerationIntegrationParametersReadOnly#getVelocityBreakFrequency()}
    * </p>
    *
    * @param jointIndex the index of the joint to provide parameters for.
    * @param positionBreakFrequency the break frequency used to compute the desired position.
    * @param velocityBreakFrequency the break frequency used to compute the desired velocity.
    */
   public void setBreakFrequencies(int jointIndex, double positionBreakFrequency, double velocityBreakFrequency)
   {
      jointParameters.get(jointIndex).setBreakFrequencies(positionBreakFrequency, velocityBreakFrequency);
   }

   /**
    * Provides to the {@link JointAccelerationIntegrationCalculator} specific parameter values for
    * the {@code jointIndex}<sup>th</sup> of this command.
    * <p>
    * These two parameters are safety parameters that are relevant to the tuning process for a
    * joint. The default values used by the calculator should be adequate in most situation.
    * </p>
    * <p>
    * The maximum velocity parameter is used to saturate the value of the desired velocity computed.
    * If not specified otherwise, {@code maxVelocity} =
    * {@link JointAccelerationIntegrationCalculator#DEFAULT_MAX_VELOCITY}. It can be increased once
    * the acceleration integration is proven to be working properly on a specific robot to allow the
    * joint to reach higher velocities.
    * </p>
    * <p>
    * The maximum position error parameter is used to limit the gap between the desired position
    * computed and the actual joint position. This is a critical parameter and should be only
    * changed once heavy testing has been performed on the robot knowing that the effects of this
    * parameter may show up only under specific circumstances. If not specified otherwise
    * {@code maxPositionError} =
    * {@link JointAccelerationIntegrationCalculator#DEFAULT_MAX_POSITION_ERROR}.
    * </p>
    * 
    * @param jointIndex the index of the joint to provide parameters for.
    * @param maxPositionError limits the gap between the desired joint position and the actual joint
    *           position.
    * @param maxVelocity limits the maximum value of the desired joint velocity.
    */
   public void setJointMaxima(int jointIndex, double maxPositionError, double maxVelocity)
   {
      jointParameters.get(jointIndex).setMaxima(maxPositionError, maxVelocity);
   }

   /**
    * Performs a full-depth copy of the data contained in the other command.
    */
   @Override
   public void set(JointAccelerationIntegrationCommand other)
   {
      clear();
      for (int i = 0; i < other.getNumberOfJointsToComputeDesiredPositionFor(); i++)
      {
         jointNamesToComputeDesiredPositionFor.add(other.jointNamesToComputeDesiredPositionFor.get(i));
         jointsToComputeDesiredPositionFor.add(other.jointsToComputeDesiredPositionFor.get(i));
      }
   }

   public void retrieveJointsFromName(Map<String, ? extends OneDoFJoint> nameToJointMap)
   {
      for (int i = 0; i < getNumberOfJointsToComputeDesiredPositionFor(); i++)
      {
         jointsToComputeDesiredPositionFor.set(i, nameToJointMap.get(jointNamesToComputeDesiredPositionFor.get(i)));
      }
   }

   public OneDoFJoint getJointToComputeDesiredPositionFor(int jointIndex)
   {
      return jointsToComputeDesiredPositionFor.get(jointIndex);
   }

   public JointAccelerationIntegrationParametersReadOnly getJointParameters(int jointIndex)
   {
      return jointParameters.get(jointIndex);
   }

   public int getNumberOfJointsToComputeDesiredPositionFor()
   {
      return jointsToComputeDesiredPositionFor.size();
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.JOINT_ACCELERATION_INTEGRATION;
   }
}
