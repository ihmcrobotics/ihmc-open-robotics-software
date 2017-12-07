package us.ihmc.commonWalkingControlModules.configurations;

import java.util.List;

import org.apache.commons.lang3.tuple.ImmutableTriple;

import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParametersReadOnly;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointAccelerationIntegrationCalculator;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;

public interface HighLevelControllerParameters
{
   WholeBodySetpointParameters getStandPrepParameters();

   JointDesiredControlMode getJointDesiredControlMode(String joint, HighLevelControllerName state);
   double getDesiredJointStiffness(String joint, HighLevelControllerName state);
   double getDesiredJointDamping(String joint, HighLevelControllerName state);

   HighLevelControllerName getDefaultInitialControllerState();
   HighLevelControllerName getFallbackControllerState();

   boolean automaticallyTransitionToWalkingWhenReady();

   double getTimeToMoveInStandPrep();
   double getMinimumTimeInStandReady();
   double getTimeInStandTransition();
   double getCalibrationDuration();

   /**
    * Parameter that allows to scale desired joint velocities in the joint level control. This
    * parameter is passed down as part of the {@link JointDesiredOutputReadOnly} and if or how the
    * parameter is used is implementation specific to each robot.
    */
   public default double getJointVelocityScaling(String joint, HighLevelControllerName state)
   {
      return 1.0;
   }

   /**
    * Specifies if the controller should compute desired positions and velocities for all the robot
    * joints from the desired acceleration. This will enable acceleration integration for all joints
    * that have their integration settings defined in {@link #getJointAccelerationIntegrationParametersNoLoad()}.
    * If this is set to false acceleration integration can still be enabled for select upper body joints
    * using the setting in {@link #getOrCreatePositionControlledJoints()}.
    * <p>
    * It is {@code false} by default and this method should be overridden to return otherwise.
    * </p>
    *
    * @return {@code true} if the desired acceleration should be integrated into desired velocity
    *         and position for all the joints.
    */
   public default boolean enableJointAccelerationIntegrationForAllJoints()
   {
      return false;
   }

   /**
    * Returns a list with triples of joint acceleration integration parameters and the names of the joints
    * that the parameter will be used for. The triple also contains the name of the joint set for the specific
    * parameters. The name will be used to create tunable parameters in the controller. E.g. the left and
    * right arm joints could be grouped this way so only a single parameter for tuning is created that affects
    * both sides.
    * <p>
    * If a joint is not contained in this map the controller will not create tunable parameters and use
    * default acceleration integration settings defined in {@link JointAccelerationIntegrationCalculator}.
    * </p>
    * <p>
    * As long as a joint is not part of a loaded joint chain these acceleration integration settings will be
    * used for that joint. E.g. all joints of a leg in support will be considered loaded. If a joint is loaded
    * the parameters defined here can be overwritten (this is optional) by defining integration parameters
    * in {@link #getJointAccelerationIntegrationParametersLoaded()}.
    * </p>
    * @return list containing acceleration integration parameters and the corresponding joints
    */
   public default List<ImmutableTriple<String, JointAccelerationIntegrationParametersReadOnly, List<String>>> getJointAccelerationIntegrationParametersNoLoad()
   {
      return null;
   }

   /**
    * This method is similar to {@link #getJointAccelerationIntegrationParametersNoLoad()}. The controller will
    * apply the acceleration integration parameters defined here only to joints that are part of a loaded chain.
    * This can be used if, for example, a different set of acceleration integration parameters should be used for
    * joints that are part of a leg that is swinging versus a leg that supporting weight.
    *
    * @return list containing acceleration integration parameters to be used if a joint is loaded
    */
   public default List<ImmutableTriple<String, JointAccelerationIntegrationParametersReadOnly, List<String>>> getJointAccelerationIntegrationParametersLoaded()
   {
      return null;
   }
}