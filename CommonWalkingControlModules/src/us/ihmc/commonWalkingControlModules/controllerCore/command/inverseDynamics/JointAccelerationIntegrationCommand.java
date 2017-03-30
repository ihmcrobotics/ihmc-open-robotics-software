package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointAccelerationIntegrationCalculator;
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
   private final TDoubleArrayList jointAlphaPosition = new TDoubleArrayList(initialCapacity);
   private final TDoubleArrayList jointAlphaVelocity = new TDoubleArrayList(initialCapacity);
   private final TDoubleArrayList jointMaxPositionError = new TDoubleArrayList(initialCapacity);
   private final TDoubleArrayList jointMaxVelocity = new TDoubleArrayList(initialCapacity);

   /**
    * Creates an empty command. It needs to be configured before being submitted to the controller
    * core.
    */
   public JointAccelerationIntegrationCommand()
   {
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
      jointAlphaPosition.reset();
      jointAlphaVelocity.reset();
      jointMaxPositionError.reset();
      jointMaxVelocity.reset();
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
      jointAlphaPosition.add(Double.NaN);
      jointAlphaVelocity.add(Double.NaN);
      jointMaxPositionError.add(Double.NaN);
      jointMaxVelocity.add(Double.NaN);
   }

   /**
    * Provides to the {@link JointAccelerationIntegrationCalculator} specific parameter values for
    * the {@code jointIndex}<sup>th</sup> of this command.
    * <p>
    * The two alpha parameters are used as leak ratios for each integration: <br>
    * Desired velocity:<br>
    * qDot<sub>des</sub><sup>t</sup> = &alpha;<sub>V</sub> qDot<sub>des</sub><sup>t - &Delta;t</sup>
    * + &Delta;t qDDot<sub>des</sub><sup>t</sup> <br>
    * Desired position:<br>
    * q<sub>des</sub><sup>t</sup> = (1 - &alpha;<sub>P</sub>) q<sub>cur</sub><sup>t - &Delta;t</sup>
    * + &alpha;<sub>P</sub> (q<sub>des</sub><sup>t - &Delta;t</sup> + &Delta;t
    * qDot<sub>des</sub><sup>t</sup>)
    * </p>
    * <p>
    * Both leak ratios have to be &in; [0, 1].
    * </p>
    * <p>
    * Decreasing the leak ratio &alpha;<sub>V</sub> used to compute the desired velocity appears to
    * be equivalent to inserting damping to the joint. A low value will cause a loss of precision on
    * the resulting q<sub>des</sub> such it does impair the tracking that high-level controller is
    * performing. If not specified otherwise, &alpha;<sub>V</sub> =
    * {@link JointAccelerationIntegrationCalculator#DEFAULT_ALPHA_VELOCITY}.
    * </p>
    * <p>
    * A high value for the leak ratio &alpha;<sub>P</sup> used to compute the desired position will
    * cause the joint to never settle by having stick-slip behavior around the "true" desired
    * position the high-level controller is trying to achieve. It can simply be downtuned until this
    * undesirable effect disappear. If not specified otherwise, &alpha;<sub>P</sup> =
    * {@link JointAccelerationIntegrationCalculator#DEFAULT_ALPHA_VELOCITY}.
    * </p>
    * 
    * @param jointIndex the index of the joint to provide parameters for.
    * @param alphaPosition the leak ratio &alpha;<sub>P</sup> used to compute the desired position.
    * @param alphaVelocity the leak ratio &alpha;<sub>V</sup> used to compute the desired velocity.
    */
   public void setJointAlphas(int jointIndex, double alphaPosition, double alphaVelocity)
   {
      jointAlphaPosition.set(jointIndex, alphaPosition);
      jointAlphaVelocity.set(jointIndex, alphaVelocity);
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
    * @param maxPositionError limits the gap between the desired joint position and the actual joint position.
    * @param maxVelocity limits the maximum value of the desired joint velocity.
    */
   public void setJointMaxima(int jointIndex, double maxPositionError, double maxVelocity)
   {
      jointMaxPositionError.set(jointIndex, maxPositionError);
      jointMaxVelocity.set(jointIndex, maxVelocity);
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

   public double getJointAlphaPosition(int jointIndex)
   {
      return jointAlphaPosition.get(jointIndex);
   }

   public double getJointAlphaVelocity(int jointIndex)
   {
      return jointAlphaVelocity.get(jointIndex);
   }

   public double getJointMaxPositionError(int jointIndex)
   {
      return jointMaxPositionError.get(jointIndex);
   }

   public double getJointMaxVelocity(int jointIndex)
   {
      return jointMaxVelocity.get(jointIndex);
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
