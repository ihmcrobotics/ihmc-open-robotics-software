package us.ihmc.commonWalkingControlModules.controllerCore.command;

import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.interfaces.WrenchBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;

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
   void getDesiredCenterOfPressure(FramePoint2DBasics copToPack, RigidBodyBasics rigidBody);

   /**
    * Typically used to obtain the desired external wrench for the feet.
    */
   boolean getDesiredExternalWrench(WrenchBasics desiredExternalWrenchToPack, RigidBodyBasics rigidBody);

   /**
    * Retrieves the linear momentum obtained after the optimization problem has been solved.
    * <p>
    * This is only available for the inverse kinematics.
    * </p>
    * <p>
    * It is useful to compare against the desired momentum that provided to the controller core as input.
    * </p>
    *
    * @return the linear momentum after optimization.
    */
   FrameVector3DReadOnly getLinearMomentum();

   /**
    * Retrieves the angular momentum obtained after the optimization problem has been solved.
    * <p>
    * This is only available for the inverse kinematics.
    * </p>
    * <p>
    * It is useful to compare against the desired momentum that provided to the controller core as input.
    * </p>
    *
    * @return the angular momentum after optimization.
    */
   FrameVector3DReadOnly getAngularMomentum();

   /**
    * Retrieves the linear momentum rate obtained after the optimization problem has been solved.
    * <p>
    * This is only available for the inverse dynamics as desired joint accelerations are required to
    * compute the linear momentum rate.
    * </p>
    * <p>
    * It is useful to compare against the desired momentum rate that provided to the controller core as
    * input.
    * </p>
    *
    * @return the linear momentum rate after optimization.
    */
   FrameVector3DReadOnly getLinearMomentumRate();

   default void getLinearMomentumRate(FrameVector3DBasics linearMomentumRateToPack)
   {
      linearMomentumRateToPack.setIncludingFrame(getLinearMomentumRate());
   }

   /**
    * Retrieves the angular momentum rate obtained after the optimization problem has been solved.
    * <p>
    * This is only available for the inverse dynamics as desired joint accelerations are required to
    * compute the angular momentum rate.
    * </p>
    * <p>
    * It is useful to compare against the desired momentum rate that provided to the controller core as
    * input.
    * </p>
    *
    * @return the angular momentum rate after optimization.
    */
   FrameVector3DReadOnly getAngularMomentumRate();

   default void getAngularMomentumRate(FrameVector3DBasics angularMomentumRateToPack)
   {
      angularMomentumRateToPack.setIncludingFrame(getAngularMomentumRate());
   }

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
   RootJointDesiredConfigurationDataReadOnly getRootJointDesiredConfigurationData();

   /**
    * Retrieves the desired state for the robot joints.
    * <p>
    * It contains all the information needed to the low-level controller of each joint.
    * </p>
    *
    * @return the desired state of the robot joints.
    */
   JointDesiredOutputListReadOnly getLowLevelOneDoFJointDesiredDataHolder();
}