package us.ihmc.commonWalkingControlModules.configurations;

import java.util.List;

import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParametersReadOnly;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointAccelerationIntegrationCalculator;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehaviorReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;

public interface HighLevelControllerParameters
{
   WholeBodySetpointParameters getStandPrepParameters();

   HighLevelControllerName getDefaultInitialControllerState();
   HighLevelControllerName getFallbackControllerState();

   boolean automaticallyTransitionToWalkingWhenReady();

   double getTimeToMoveInStandPrep();
   double getMinimumTimeInStandReady();
   double getTimeInStandTransition();
   default double getTimeInStandTransition(HighLevelControllerName controllerName)
   {
      return getTimeInStandTransition();
   }
   double getCalibrationDuration();

   /**
    * Returns a list of joint behaviors for groups of joints. In each {@link GroupParameter} a set
    * of joints can be specified that will share a common tunable parameter. In this way joints can
    * be grouped together and tuned using a single parameter (e.g. left elbow and right elbow).
    * <p>
    * The joint behavior defines parameters for the control law used by a joint level controller to
    * track the desired values of the whole body controller. It contains information such as stiffness
    * and {@link JointDesiredControlMode}. The implementation of this is usually robot specific.
    * </p>
    * <p>
    * As long as a joint is not part of a loaded joint chain these behavior settings will be
    * used for that joint. E.g. all joints of a leg in support will be considered loaded. If a joint is loaded
    * the parameters defined here can be overwritten (this is optional) by defining integration parameters
    * in {@link #getDesiredJointBehaviorsLoaded()}.
    * </p>
    * @return list containing joint behavior parameters and the corresponding joint groups
    */
   public default List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviors(HighLevelControllerName state)
   {
      return null;
   }

   /**
    * This method is similar to {@link #getDesiredJointBehaviors()}. The controller will
    * apply the joint settings defined here only to joints that are part of a loaded chain.
    * This can be used if, for example, a different set of parameters should be used for
    * joints that are part of a leg that is swinging versus a leg that supporting weight.
    *
    * @return list containing joint behavior parameters to be used if a joint is loaded
    */
   public default List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviorsUnderLoad(HighLevelControllerName state)
   {
      return null;
   }

   /**
    * Returns a list of acceleration integration parameters for groups of joints. In each
    * {@link GroupParameter} a set of joints can be specified that will share a common tunable
    * parameter. In this way joints can be grouped together and tuned using a single parameter (e.g.
    * left elbow and right elbow).
    * <p>
    * If a joint is not contained in this list the controller will not create tunable parameters and use
    * default acceleration integration settings defined in {@link JointAccelerationIntegrationCalculator}.
    * </p>
    * <p>
    * As long as a joint is not part of a loaded joint chain these acceleration integration settings will be
    * used for that joint. E.g. all joints of a leg in support will be considered loaded. If a joint is loaded
    * the parameters defined here can be overwritten (this is optional) by defining integration parameters
    * in {@link #getJointAccelerationIntegrationParametersUnderLoad()}.
    * </p>
    * @return list containing acceleration integration parameters and the corresponding joint groups
    */
   public default List<GroupParameter<JointAccelerationIntegrationParametersReadOnly>> getJointAccelerationIntegrationParameters(HighLevelControllerName state)
   {
      return null;
   }

   /**
    * This method is similar to {@link #getJointAccelerationIntegrationParameters()}. The controller will
    * apply the acceleration integration parameters defined here only to joints that are part of a loaded chain.
    * This can be used if, for example, a different set of acceleration integration parameters should be used for
    * joints that are part of a leg that is swinging versus a leg that supporting weight.
    *
    * @return list containing acceleration integration parameters to be used if a joint is loaded
    */
   public default List<GroupParameter<JointAccelerationIntegrationParametersReadOnly>> getJointAccelerationIntegrationParametersUnderLoad(HighLevelControllerName state)
   {
      return null;
   }

   /**
    * Provides the option to deactivate the integration of the desired accelerations inside the whole body
    * controller. This can be used if the acceleration should be integrated in the robot specific joint
    * controllers (e.g. because they have a higher loop rate).
    *
    * @return whether the acceleration integration should be deactivated in the whole body controller
    */
   public default boolean deactivateAccelerationIntegrationInTheWBC()
   {
      return false;
   }

   /**
    * If a torque offset calibration controller is used, this defines the max torque offset that can be applied from each execution
    * See {@link JointTorqueOffsetEstimatorController} for implementation details
    * @return the max torque offset that can be applied in Nm. 
    */
   public default double getCalibrationMaxTorqueOffset()
   {
      return 5.0;
   }
}