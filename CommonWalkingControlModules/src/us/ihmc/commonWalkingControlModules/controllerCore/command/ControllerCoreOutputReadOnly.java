package us.ihmc.commonWalkingControlModules.controllerCore.command;

import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.screwTheory.RigidBody;

/**
 * {@link ControllerCoreOutputReadOnly} is a read-only access to the controller core output.
 * <p>
 * The most important information is obtained via
 * {@link #getLowLevelOneDoFJointDesiredDataHolder()}. It returns the entire desired state of the
 * robot to be controlled. It also holds onto the desired actuation mode, for instance whether an
 * actuator should be position or torque controlled.
 * </p>
 * <p>
 * The methods besides {@link #getLowLevelOneDoFJointDesiredDataHolder()} provide additional
 * information on the controller core output.
 * </p>
 */
public interface ControllerCoreOutputReadOnly
{
   /**
    * Typically used to obtain the desired center of pressures for the feet.
    */
   public abstract void getDesiredCenterOfPressure(FramePoint2d copToPack, RigidBody rigidBody);

   /**
    * Retrieves the linear momentum rate obtained after the optimization problem has been solved.
    * <p>
    * This is only available for the inverse dynamics as desired joint accelerations are required to
    * compute the linear momentum rate.
    * </p>
    * <p>
    * It is useful to compare against the desired momentum rate that provided to the controller core
    * as input.
    * </p>
    * 
    * @param linearMomentumRateToPack the linear momentum rate after optimization.
    */
   public abstract void getLinearMomentumRate(FrameVector linearMomentumRateToPack);

   /**
    * Retrieves the desired state for the root joint.
    * <p>
    * It not directly useful as to control the actual robot, but combined with
    * {@link #getLowLevelOneDoFJointDesiredDataHolder()} it completes information to entire robot
    * desired state.
    * </p>
    * 
    * @return the root joint desired state data.
    */
   public abstract RootJointDesiredConfigurationDataReadOnly getRootJointDesiredConfigurationData();

   /**
    * Retrieves the desired state for the robot joints.
    * <p>
    * It contains all the information needed to the low-level controller of each joint.
    * </p>
    * 
    * @return the desired state of the robot joints.
    */
   public abstract LowLevelOneDoFJointDesiredDataHolderReadOnly getLowLevelOneDoFJointDesiredDataHolder();
}