package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointVelocityIntegrationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointVelocityIntegrationParametersReadOnly;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointVelocityIntegrationCalculator;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * {@link SpatialFeedbackControlCommand} is a command meant to be submitted to the
 * {@link WholeBodyControllerCore} via the {@link ControllerCoreCommand}.
 * <p>
 * The {@link JointVelocityIntegrationCalculator} uses these commands to properly setup joint
 * acceleration integration whether it is for such or such joint and what gains should be used.
 * </p>
 * <p>
 * When enabled for a joint, the {@code JointAccelerationIntegrationCalculator} will perform a
 * single step integration off of the desired joint velocity to compute the desired joint
 * position and then a single step differentiation off of the desired joint velocity to
 * compute the desired joint acceleration.
 * </p>
 * 
 * @author Robert Griffin
 *
 */
public class JointVelocityIntegrationCommand implements InverseKinematicsCommand<JointVelocityIntegrationCommand>
{
   private final int initialCapacity = 15;
   private final List<String> jointNamesToComputeDesiredPositionFor = new ArrayList<>(initialCapacity);
   private final List<OneDoFJoint> jointsToComputeDesiredPositionFor = new ArrayList<>(initialCapacity);
   private final RecyclingArrayList<JointVelocityIntegrationParameters> jointParameters = new RecyclingArrayList<>(initialCapacity,
                                                                                                                   JointVelocityIntegrationParameters.class);

   /**
    * Creates an empty command. It needs to be configured before being submitted to the controller
    * core.
    */
   public JointVelocityIntegrationCommand()
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
    * Register an additional joint the {@link JointVelocityIntegrationCalculator} should compute
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
    * Sets the velocity integration parameters for the {@code jointIndex}<sup>th</sup> of this
    * command to {@code jointParameters}.
    * 
    * @param jointIndex the index of the joint to provide parameters for.
    * @param jointParameters the set of parameters to use for the joint. Not modified.
    */
   public void setJointParameters(int jointIndex, JointVelocityIntegrationParametersReadOnly jointParameters)
   {
      this.jointParameters.get(jointIndex).set(jointParameters);
   }

   /**
    * Provides to the {@link JointVelocityIntegrationCalculator} specific parameter values for
    * the {@code jointIndex}<sup>th</sup> of this command.
    * <p>
    * For the usage of these parameters see<br>
    * {@link JointVelocityIntegrationParametersReadOnly#getPositionBreakFrequency()}<br>
    * {@link JointVelocityIntegrationParametersReadOnly#getAccelerationBreakFrequency()}
    * </p>
    *
    * @param jointIndex the index of the joint to provide parameters for.
    * @param positionBreakFrequency the break frequency used to compute the desired position.
    * @param accelerationBreakFrequency the break frequency used to compute the desired acceleration.
    */
   public void setBreakFrequencies(int jointIndex, double positionBreakFrequency, double accelerationBreakFrequency)
   {
      jointParameters.get(jointIndex).setBreakFrequencies(positionBreakFrequency, accelerationBreakFrequency);
   }

   /**
    * Provides to the {@link JointVelocityIntegrationCalculator} specific parameter values for
    * the {@code jointIndex}<sup>th</sup> of this command.
    * <p>
    * These two parameters are safety parameters that are relevant to the tuning process for a
    * joint. The default values used by the calculator should be adequate in most situation.
    * </p>
    * <p>
    * The maximum acceleration parameter is used to saturate the value of the desired acceleration computed.
    * If not specified otherwise, {@code maxVelocity} =
    * {@link JointVelocityIntegrationCalculator#DEFAULT_MAX_ACCELERATION}. It can be increased once
    * the velocity differentiation is proven to be working properly on a specific robot to allow the
    * joint to reach higher accelerations.
    * </p>
    * <p>
    * The maximum position error parameter is used to limit the gap between the desired position
    * computed and the actual joint position. This is a critical parameter and should be only
    * changed once heavy testing has been performed on the robot knowing that the effects of this
    * parameter may show up only under specific circumstances. If not specified otherwise
    * {@code maxPositionError} =
    * {@link JointVelocityIntegrationCalculator#DEFAULT_MAX_POSITION_ERROR}.
    * </p>
    * 
    * @param jointIndex the index of the joint to provide parameters for.
    * @param maxPositionError limits the gap between the desired joint position and the actual joint
    *           position.
    * @param maxAcceleration limits the maximum value of the desired joint acceleration.
    */
   public void setJointMaxima(int jointIndex, double maxPositionError, double maxAcceleration)
   {
      jointParameters.get(jointIndex).setMaxima(maxPositionError, maxAcceleration);
   }

   /**
    * Performs a full-depth copy of the data contained in the other command.
    */
   @Override
   public void set(JointVelocityIntegrationCommand other)
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

   public JointVelocityIntegrationParametersReadOnly getJointParameters(int jointIndex)
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
      return ControllerCoreCommandType.JOINT_VELOCITY_INTEGRATION;
   }
}
