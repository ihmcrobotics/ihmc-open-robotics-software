package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyInverseDynamicsSolver;
import us.ihmc.commonWalkingControlModules.inverseKinematics.InverseKinematicsOptimizationControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.virtualModelControl.VirtualModelControlOptimizationControlModule;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.FrictionConeRotationCalculator;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ZeroConeRotationCalculator;
import us.ihmc.convexOptimization.quadraticProgram.NativeActiveSetQPSolverWithInactiveVariablesInterface;
import us.ihmc.convexOptimization.quadraticProgram.SimpleEfficientActiveSetQPSolver;
import us.ihmc.convexOptimization.quadraticProgram.SimpleEfficientActiveSetQPSolverWithInactiveVariables;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.mecano.algorithms.InverseDynamicsCalculator;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.robotSide.RobotSide;

public interface ControllerCoreOptimizationSettings
{
   /**
    * Gets the weight specifying how much high joint velocity values should be penalized in the
    * optimization problem.
    * <p>
    * This parameter is used in {@link InverseKinematicsOptimizationControlModule} which itself is used
    * when running the {@link WholeBodyControllerCore} in the
    * {@link WholeBodyControllerCoreMode#INVERSE_KINEMATICS} mode.
    * </p>
    * <p>
    * A non-zero positive value should be used to ensure the Hessian matrix in the optimization is
    * invertible. It is should preferably be above {@code 1.0e-8}. A high value will cause the system
    * to become too 'lazy'.
    * </p>
    *
    * @return the weight to use for joint acceleration regularization.
    */
   default double getJointVelocityWeight()
   {
      return 1.0e-8;
   }

   /**
    * Gets the weight specifying how much high joint acceleration values should be penalized in the
    * optimization problem.
    * <p>
    * This parameter is used in:
    * <ul>
    * <li>{@link InverseDynamicsOptimizationControlModule} which itself is used when running the
    * {@link WholeBodyControllerCore} in the {@link WholeBodyControllerCoreMode#INVERSE_DYNAMICS} mode.
    * <li>{@link InverseKinematicsOptimizationControlModule} which itself is used when running the
    * {@link WholeBodyControllerCore} in the {@link WholeBodyControllerCoreMode#INVERSE_KINEMATICS}
    * mode.
    * </ul>
    * </p>
    * <p>
    * When used for the inverse dynamics mode, a non-zero positive value should be used to ensure the
    * Hessian matrix in the optimization is invertible. It is should preferably be above
    * {@code 1.0e-8}. A high value will cause the system to become too 'lazy'. A value of {@code 0.005}
    * is used for Atlas' simulations.
    * </p>
    *
    * @return the weight to use for joint acceleration regularization.
    */
   double getJointAccelerationWeight();

   /**
    * Gets the maximum value for the absolute joint accelerations in the optimization problem.
    * <p>
    * This parameter is used in {@link InverseDynamicsOptimizationControlModule} which itself is used
    * when running the {@link WholeBodyControllerCore} in the
    * {@link WholeBodyControllerCoreMode#INVERSE_DYNAMICS} mode.
    * </p>
    *
    * @return the maximum joint acceleration, the returned value has to be in [0,
    *         {@link Double#POSITIVE_INFINITY}].
    */
   default double getMaximumJointAcceleration()
   {
      return 200.0;
   }

   /**
    * Gets the weight specifying how much high joint jerk values should be penalized in the
    * optimization problem.
    * <p>
    * This parameter is used in {@link InverseDynamicsOptimizationControlModule} which itself is used
    * when running the {@link WholeBodyControllerCore} in the
    * {@link WholeBodyControllerCoreMode#INVERSE_DYNAMICS} mode.
    * </p>
    * <p>
    * A positive value should be used but does not necessarily need to be non-zero. This weight helps
    * to improve smoothness of the resulting motions. A high value will cause the system to become too
    * 'floppy'. A value of {@code 0.1} is used for Atlas' simulations.
    * </p>
    *
    * @return the weight to use for joint jerk regularization.
    */
   double getJointJerkWeight();

   /**
    * Gets the weight specifying how much high joint torque values should be penalized in the
    * optimization problem.
    * <p>
    * This parameter is used in {@link InverseDynamicsOptimizationControlModule} which itself is used
    * when running the {@link WholeBodyControllerCore} in the
    * {@link WholeBodyControllerCoreMode#INVERSE_DYNAMICS} mode.
    * </p>
    * <p>
    * This parameter is only used when the joint torques are part of the optimization's objective.
    * </p>
    *
    * @return the weight to use for joint torques regularization.
    */
   default double getJointTorqueWeight()
   {
      return 0.005;
   }

   /**
    * Whether an objective should be added to minimize the resulting joint torques.
    * <p>
    * This parameter is used when running the {@link WholeBodyControllerCore} in the
    * {@link WholeBodyControllerCoreMode#INVERSE_DYNAMICS} mode.
    * </p>
    * 
    * @return {@code true} to enable joint torque minimization, {@code false} otherwise. Default value
    *         is {@code false}.
    */
   default boolean areJointTorquesMinimized()
   {
      return false;
   }

   /**
    * Whether the joint torques should be constrained and, if so, where they are constrained. Joints
    * are constrained to their effort limits in the URDF at this time.
    * <ul>
    * <li>{@code CONSTRAINTS_IN_QP} will constrain torque inside the QP, which means that acceleration
    * constraints based on the torque limits will be added to the QP. This ensures that the torque
    * coming out of the inverse dynamics will respect the torque limits.
    * <li>{@code CONSTRAINTS_IN_CONTROLLER} will clamp torque in the controller (after the QP).
    * Choosing this option means that the QP might ask for torque above a joint's torque limit, but it
    * will be clamped afterward in the controller.
    * <li>{@code CONSTRAINTS_IN_QP_AND_CONTROLLER} will constrain torque inside the QP and the
    * controller. This option ensures both the QP and the controller respect a joint's torque limit.
    * <li>{@code NO_CONSTRAINTS} will not add constraints anywhere.
    * </ul>
    * Default value: {@code NO_CONSTRAINTS}.
    */
   public enum JointTorqueLimitEnforcementMethod
   {

      NO_CONSTRAINTS, CONSTRAINTS_IN_QP, CONSTRAINTS_IN_CONTROLLER, CONSTRAINTS_IN_QP_AND_CONTROLLER;

   }

   /**
    * Whether the joint torques should be constrained and, if so, where they are constrained. Joints
    * are constrained to their effort limits in the URDF at this time. See
    * {@link JointTorqueLimitEnforcementMethod}. Default value: {@code NO_CONSTRAINTS}.
    */
   default JointTorqueLimitEnforcementMethod getJointTorqueLimitEnforcementMethod()
   {
      return JointTorqueLimitEnforcementMethod.NO_CONSTRAINTS;
   }

   /**
    * Whether the joint power should be constrained and, if so, where they are constrained. The list of
    * joints to be constrained and their power limits should be provided by overriding
    * getJointPowerLimits(), typically inside [YourRobotHere]MomentumOptimizationSettings.
    * <ul>
    * <li>{@code CONSTRAINTS_IN_QP} will constrain power inside the QP. The power limits are converted
    * to torque limits (since velocity is known) and fed as a JointTorqueCommand, and are finally
    * converted into constraints on joint accelerations. This tries to ensure that the power coming out
    * of the inverse dynamics will respect the power limits, but there may be slight violation of this
    * limit, since power is only indirectly constrained via the constrained torque.
    * <li>{@code CONSTRAINTS_IN_CONTROLLER} will clamp power in the controller (after the QP). Choosing
    * this option means that the QP might ask for power above a joint's power limit, but it will be
    * clamped (technically torque is clamped) afterward in the controller. There may be slight
    * violation of this limit, since power is only indirectly clamped via the clamped torque.
    * <li>{@code CONSTRAINTS_IN_QP_AND_CONTROLLER} will constrain power inside the QP and the
    * controller. This option tries to ensure both the QP and the controller respect a joint's power
    * limit.
    * <li>{@code NO_CONSTRAINTS} will not add constraints anywhere.
    * </ul>
    * Default value: {@code NO_CONSTRAINTS}.
    */
   public enum JointPowerLimitEnforcementMethod
   {

      NO_CONSTRAINTS, CONSTRAINTS_IN_QP, CONSTRAINTS_IN_CONTROLLER, CONSTRAINTS_IN_QP_AND_CONTROLLER;

   }

   /**
    * Whether the joint powers should be constrained and, if so, where they are constrained. The list
    * of joints to be constrained and their power limits should be provided by overriding
    * getJointPowerLimits(), typically inside [YourRobotHere]MomentumOptimizationSettings. Default
    * value: {@code NO_CONSTRAINTS}.
    */
   default JointPowerLimitEnforcementMethod getJointPowerLimitEnforcementMethod()
   {
      return JointPowerLimitEnforcementMethod.NO_CONSTRAINTS;
   }

   /**
    * Gets the list of all joints with a constraint on power. This list is typically defined inside
    * [YourRobotHere]MomentumOptimizationSettings. Each element of the map has a string, which is the
    * name of a joint (i.e. jointMap.getArmJointName(RobotSide.LEFT, ArmJointName.SHOULDER_PITCH)), and
    * a double which is that joint's power limit in Watts. Power limits can be enforced in the QP, in
    * the controller, or in both the QP and the controller {@link JointPowerLimitEnforcementMethod}.
    * Power limits must be defined for both the LEFT and RIGHT sides if both are desired. For the
    * default JointPowerLimitEnforcementMethod.NO_CONSTRAINTS, this list is not used.
    */
   default TObjectDoubleHashMap<String> getJointPowerLimits()
   {
      return null;
   }

   /**
    * Gets the weight specifying how much high contact force values should be penalized in the
    * optimization problem.
    * <p>
    * This parameter is used in {@link InverseDynamicsOptimizationControlModule} and
    * {@link VirtualModelControlOptimizationControlModule} which are use when running the
    * {@link WholeBodyControllerCore} in {@link WholeBodyControllerCoreMode#INVERSE_DYNAMICS} or
    * {@link WholeBodyControllerCoreMode#VIRTUAL_MODEL} respectively.
    * </p>
    * <p>
    * A non-zero positive value should be used to ensure the Hessian matrix in the optimization is
    * invertible. It is should preferably be above {@code 1.0e-8}. A high value will cause the system
    * to become too 'floppy'. A value of {@code 1.0e-5} is used for Atlas' simulations. Be careful as
    * the weight is for forces and thus is affected by the total weight of the robot. A higher value
    * should be picked for a lighter robot.
    * </p>
    * <p>
    * The notation 'rho' is used to describe the contact force magnitude to exert at each basis vector
    * of each contact point. As the setup is somewhat hectic to explain, you can refer to the following
    * paper at page 9: <br>
    * <a href=
    * "https://www.researchgate.net/publication/280839675_Design_of_a_Momentum-Based_Control_Framework_and_Application_to_the_Humanoid_Robot_Atlas">
    * Design of a Momentum-Based Control Framework and Application to the Humanoid Robot Atlas.</a>
    * </p>
    *
    * @return the weight to use for contact force regularization.
    */
   double getRhoWeight();

   /**
    * Gets the minimum force value to apply at each basis vector of each contact point.
    * <p>
    * This parameter is used in {@link InverseDynamicsOptimizationControlModule} and
    * {@link VirtualModelControlOptimizationControlModule} which are use when running the
    * {@link WholeBodyControllerCore} in {@link WholeBodyControllerCoreMode#INVERSE_DYNAMICS} or
    * {@link WholeBodyControllerCoreMode#VIRTUAL_MODEL} respectively.
    * </p>
    * <p>
    * A positive value will ensure that the optimization is setup to satisfy a unilateral contact for
    * each contacting body. A non-zero and positive value ensures that any rigid-body that is assumed
    * to be in contact will exert a minimum amount of force onto the environment. This tends to reduce
    * foot slipping.
    * </p>
    * <p>
    * The notation 'rho' is used to describe the contact force magnitude to exert at each basis vector
    * of each contact point. As the setup is somewhat hectic to explain, you can refer to the following
    * paper at page 9: <br>
    * <a href=
    * "https://www.researchgate.net/publication/280839675_Design_of_a_Momentum-Based_Control_Framework_and_Application_to_the_Humanoid_Robot_Atlas">
    * Design of a Momentum-Based Control Framework and Application to the Humanoid Robot Atlas.</a>
    * </p>
    *
    * @return the minimum value for each component of rho.
    */
   double getRhoMin();

   /**
    * Gets the default weight specifying how much high variations of contact forces should be penalized
    * in the optimization problem.
    * <p>
    * This parameter is used in {@link InverseDynamicsOptimizationControlModule} and
    * {@link VirtualModelControlOptimizationControlModule} which are use when running the
    * {@link WholeBodyControllerCore} in {@link WholeBodyControllerCoreMode#INVERSE_DYNAMICS} or
    * {@link WholeBodyControllerCoreMode#VIRTUAL_MODEL} respectively.
    * </p>
    * <p>
    * A positive value should be used but does not necessarily need to be non-zero. This weight helps
    * to improve smoothness of the resulting forces. A high value will cause the system to become too
    * 'floppy' and unresponsive. A value of {@code 0.002} is used for Atlas' simulations. Be careful as
    * the weight is for forces and thus is affected by the total weight of the robot. A higher value
    * should be picked for a lighter robot.
    * </p>
    *
    * @return the weight to use for the regularization of the rate of change of contact forces.
    */
   double getRhoRateDefaultWeight();

   /**
    * When required, the controller core can switch to temporarily use a higher weight to regulate
    * variations in contact forces. This is the weight used in such situation.
    *
    * @return the high weight to use for the regularization of the rate of change of contact forces.
    */
   default double getRhoRateHighWeight()
   {
      return getRhoRateDefaultWeight();
   }

   /**
    * Gets the weight specifying how much deviation of the desired center of pressure (CoP) off of the
    * contact support centroid should be penalized.
    * <p>
    * In other words, a high positive value will result in the controller core trying to keep the CoP
    * in the middle of each foot for instance. This value does not need to be non-zero. A value of
    * about {@code 100.0} is used for Atlas' simulations.
    * </p>
    * <p>
    * This parameter is used in {@link InverseDynamicsOptimizationControlModule} and
    * {@link VirtualModelControlOptimizationControlModule} which are use when running the
    * {@link WholeBodyControllerCore} in {@link WholeBodyControllerCoreMode#INVERSE_DYNAMICS} or
    * {@link WholeBodyControllerCoreMode#VIRTUAL_MODEL} respectively.
    * </p>
    *
    * @return the regularization weight to use on the center of pressure location.
    */
   Vector2D getCoPWeight();

   /**
    * Gets the default weight specifying how much variations of the desired center of pressure should
    * be penalized.
    * <p>
    * This parameter is used in {@link InverseDynamicsOptimizationControlModule} and
    * {@link VirtualModelControlOptimizationControlModule} which are use when running the
    * {@link WholeBodyControllerCore} in {@link WholeBodyControllerCoreMode#INVERSE_DYNAMICS} or
    * {@link WholeBodyControllerCoreMode#VIRTUAL_MODEL} respectively.
    * </p>
    * <p>
    * The value should be positive but not necessarily non-zero. It helps smoothing the motion of the
    * center pressure of each contacting body.
    * </p>
    *
    * @return the regularization weight to use for center of pressure variations.
    */
   Vector2D getCoPRateDefaultWeight();

   /**
    * When required, the controller core can switch to temporarily use a higher weight to regulate
    * variations in center of pressure. This is the weight used in such situation.
    *
    * @return the high regularization weight to use for center of pressure variations.
    */
   default Vector2D getCoPRateHighWeight()
   {
      return getCoPRateDefaultWeight();
   }

   /**
    * Gets the number of basis vectors to use per contact point.
    * <p>
    * This parameter is used in {@link InverseDynamicsOptimizationControlModule} and
    * {@link VirtualModelControlOptimizationControlModule} which are use when running the
    * {@link WholeBodyControllerCore} in {@link WholeBodyControllerCoreMode#INVERSE_DYNAMICS} or
    * {@link WholeBodyControllerCoreMode#VIRTUAL_MODEL} respectively.
    * </p>
    * <p>
    * The value should very likely be {@code 4} as a lower value would be too conservative and higher
    * values would be too computationally expensive.
    * </p>
    * <p>
    * Please refer to the following paper page 9 to find the description of this variable:<br>
    * <a href=
    * "https://www.researchgate.net/publication/280839675_Design_of_a_Momentum-Based_Control_Framework_and_Application_to_the_Humanoid_Robot_Atlas">
    * Design of a Momentum-Based Control Framework and Application to the Humanoid Robot Atlas.</a>
    * </p>
    *
    * @return the number of basis vectors to use per contact point.
    */
   int getNumberOfBasisVectorsPerContactPoint();

   /**
    * Gets the maximum number of contact points that each contactable body can have.
    * <p>
    * This parameter is used in {@link InverseDynamicsOptimizationControlModule} and
    * {@link VirtualModelControlOptimizationControlModule} which are use when running the
    * {@link WholeBodyControllerCore} in {@link WholeBodyControllerCoreMode#INVERSE_DYNAMICS} or
    * {@link WholeBodyControllerCoreMode#VIRTUAL_MODEL} respectively.
    * </p>
    * <p>
    * It is usually set to four contact points.
    * </p>
    *
    * @return the maximum number of contact points to use per contactable body.
    */
   int getNumberOfContactPointsPerContactableBody();

   /**
    * Gets the number of contactable bodies for the entire robot, i.e. the number of bodies that can be
    * used to bear the robot weight.
    * <p>
    * This parameter is used in {@link InverseDynamicsOptimizationControlModule} and
    * {@link VirtualModelControlOptimizationControlModule} which are use when running the
    * {@link WholeBodyControllerCore} in {@link WholeBodyControllerCoreMode#INVERSE_DYNAMICS} or
    * {@link WholeBodyControllerCoreMode#VIRTUAL_MODEL} respectively.
    * </p>
    *
    * @return the number of contactable bodies.
    */
   int getNumberOfContactableBodies();

   /**
    * This should simply be equal to:<br>
    * {@code rhoSize = numberOfBasisVectorsPerContactPoint * numberOfContactPointsPerContactableBody * numberOfContactableBodies}
    * <p>
    * The notation 'rho' is used to describe the contact force magnitude to exert at each basis vector
    * of each contact point. As the setup is somewhat hectic to explain, you can refer to the following
    * paper at page 9: <br>
    * <a href=
    * "https://www.researchgate.net/publication/280839675_Design_of_a_Momentum-Based_Control_Framework_and_Application_to_the_Humanoid_Robot_Atlas">
    * Design of a Momentum-Based Control Framework and Application to the Humanoid Robot Atlas.</a>
    * </p>
    *
    * @return the size of the vector rho.
    */
   default int getRhoSize()
   {
      return getNumberOfContactableBodies() * getNumberOfContactPointsPerContactableBody() * getNumberOfBasisVectorsPerContactPoint();
   }

   /**
    * Returns whether or not the optimization should consider the rho variable when its associated
    * contact point is not in contact. This reduces the optimization size for the whole body
    * controller, potentially leading to higher speeds.
    */
   default boolean getDeactivateRhoWhenNotInContact()
   {
      return true;
   }

   /**
    * Gets a new instance of {@code ActiveSetQPSolver} to be used in the controller core.
    * <p>
    * A QP solver is needed for any of the three modes of the controller core.
    * </p>
    *
    * @return a new instance of the QP solver to be used. By default it is the
    *         {@link SimpleEfficientActiveSetQPSolver}.
    */
   default NativeActiveSetQPSolverWithInactiveVariablesInterface getActiveSetQPSolver()
   {
      return new SimpleEfficientActiveSetQPSolverWithInactiveVariables();
   }

   /**
    * Sets whether or not to use a warm start in the active set solver where the previous active set is
    * retained between control ticks.
    */
   default boolean useWarmStartInSolver()
   {
      return false;
   }

   /**
    * Sets the maximum number of iterations allowed in the solver before throwing a no convergence
    * exception.
    */
   default int getMaxNumberOfSolverIterations()
   {
      return 10;
   }

   /**
    * Informs the active optimization module whether the joint velocity limits should be considered
    * when calculating the desired joint accelerations or joint velocities.
    * <p>
    * If {@code true}:
    * <ul>
    * <li>if running in {@link WholeBodyControllerCoreMode#INVERSE_DYNAMICS} mode, the calculated
    * desired joint accelerations do not require the joints to overshoot their limit at any time.
    * <li>if running in {@link WholeBodyControllerCoreMode#INVERSE_KINEMATICS} mode, the calculated
    * desired joint velocities remain between the joint velocity lower and upper bounds at all time.
    * </ul>
    * </p>
    *
    * @return {@code true} if the joint velocity limits are to considered, {@code false} otherwise.
    */
   default boolean areJointVelocityLimitsConsidered()
   {
      return true;
   }

   /**
    * Adds the ability to define a robot specific friction cone rotation. This can be useful when using
    * friction cone representations with a low number of basis vectors and for feet with a lot of
    * contact points. By default this will return a zero offset provider.
    */
   default public FrictionConeRotationCalculator getFrictionConeRotation()
   {
      return new ZeroConeRotationCalculator();
   }

   /**
    * Determines which inverse dynamics calculator is used in {@link WholeBodyInverseDynamicsSolver}.
    * <p>
    * If true, {@link DynamicsMatrixCalculator} is used.
    * <p>
    * If false, {@link InverseDynamicsCalculator} is used.
    */
   default boolean useDynamicMatrixCalculatorForInverseDynamics()
   {
      return false;
   }

   /**
    * If true, {@link WholeBodyInverseDynamicsSolver} will update the {@link DynamicsMatrixCalculator}.
    */
   default boolean updateDynamicMatrixCalculator()
   {
      return useDynamicMatrixCalculatorForInverseDynamics() || areJointTorquesMinimized();
   }
}