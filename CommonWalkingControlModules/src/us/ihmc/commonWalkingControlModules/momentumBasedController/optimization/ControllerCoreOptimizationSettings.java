package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.virtualModelControl.VirtualModelControlOptimizationControlModule;
import us.ihmc.euclid.tuple2D.Vector2D;

public interface ControllerCoreOptimizationSettings
{
   /**
    * Gets the weight specifying how much high joint acceleration values should be penalized in the
    * optimization problem.
    * <p>
    * This parameter is used in {@link InverseDynamicsOptimizationControlModule} which itself is
    * used when running the {@link WholeBodyControllerCore} in the
    * {@link WholeBodyControllerCoreMode#INVERSE_DYNAMICS} mode.
    * </p>
    * <p>
    * A non-zero positive value should be used to ensure the Hessian matrix in the optimization is
    * invertible. It is should preferably be above {@code 1.0e-8}. A high value will cause the
    * system to become too 'lazy'. A value of {@code 0.005} is used for Atlas' simulations.
    * </p>
    * 
    * @return the weight to use for joint acceleration regularization.
    */
   double getJointAccelerationWeight();

   /**
    * Gets the weight specifying how much high joint jerk values should be penalized in the
    * optimization problem.
    * <p>
    * This parameter is used in {@link InverseDynamicsOptimizationControlModule} which itself is
    * used when running the {@link WholeBodyControllerCore} in the
    * {@link WholeBodyControllerCoreMode#INVERSE_DYNAMICS} mode.
    * </p>
    * <p>
    * A positive value should be used but does not necessarily need to be non-zero. This weight
    * helps to improve smoothness of the resulting motions. A high value will cause the system to
    * become too 'floppy'. A value of {@code 0.1} is used for Atlas' simulations.
    * </p>
    * 
    * @return the weight to use for joint jerk regularization.
    */
   double getJointJerkWeight();

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
    * invertible. It is should preferably be above {@code 1.0e-8}. A high value will cause the
    * system to become too 'floppy'. A value of {@code 1.0e-5} is used for Atlas' simulations. Be
    * careful as the weight is for forces and thus is affected by the total weight of the robot. A
    * higher value should be picked for a lighter robot.
    * </p>
    * <p>
    * The notation 'rho' is used to describe the contact force magnitude to exert at each basis
    * vector of each contact point. As the setup is somewhat hectic to explain, you can refer to the
    * following paper at page 9: <br>
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
    * A positive value will ensure that the optimization is setup to satisfy a unilateral contact
    * for each contacting body. A non-zero and positive value ensures that any rigid-body that is
    * assumed to be in contact will exert a minimum amount of force onto the environment. This tends
    * to reduce foot slipping.
    * </p>
    * <p>
    * The notation 'rho' is used to describe the contact force magnitude to exert at each basis
    * vector of each contact point. As the setup is somewhat hectic to explain, you can refer to the
    * following paper at page 9: <br>
    * <a href=
    * "https://www.researchgate.net/publication/280839675_Design_of_a_Momentum-Based_Control_Framework_and_Application_to_the_Humanoid_Robot_Atlas">
    * Design of a Momentum-Based Control Framework and Application to the Humanoid Robot Atlas.</a>
    * </p>
    * 
    * @return the minimum value for each component of rho.
    */
   double getRhoMin();

   /**
    * Gets the default weight specifying how much high variations of contact forces should be
    * penalized in the optimization problem.
    * <p>
    * This parameter is used in {@link InverseDynamicsOptimizationControlModule} and
    * {@link VirtualModelControlOptimizationControlModule} which are use when running the
    * {@link WholeBodyControllerCore} in {@link WholeBodyControllerCoreMode#INVERSE_DYNAMICS} or
    * {@link WholeBodyControllerCoreMode#VIRTUAL_MODEL} respectively.
    * </p>
    * <p>
    * A positive value should be used but does not necessarily need to be non-zero. This weight
    * helps to improve smoothness of the resulting forces. A high value will cause the system to
    * become too 'floppy' and unresponsive. A value of {@code 0.002} is used for Atlas' simulations.
    * Be careful as the weight is for forces and thus is affected by the total weight of the robot.
    * A higher value should be picked for a lighter robot.
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
   double getRhoRateHighWeight();

   /**
    * Gets the weight specifying how much deviation of the desired center of pressure (CoP) off of
    * the contact support centroid should be penalized.
    * <p>
    * In other words, a high positive value will result in the controller core trying to keep the
    * CoP in the middle of each foot for instance. This value does not need to be non-zero. A value
    * of about {@code 100.0} is used for Atlas' simulations.
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
    * Gets the default weight specifying how much variations of the desired center of pressure
    * should be penalized.
    * <p>
    * This parameter is used in {@link InverseDynamicsOptimizationControlModule} and
    * {@link VirtualModelControlOptimizationControlModule} which are use when running the
    * {@link WholeBodyControllerCore} in {@link WholeBodyControllerCoreMode#INVERSE_DYNAMICS} or
    * {@link WholeBodyControllerCoreMode#VIRTUAL_MODEL} respectively.
    * </p>
    * <p>
    * The value should be positive but not necessarily non-zero. It helps smoothing the motion of
    * the center pressure of each contacting body.
    * </p>
    * 
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
   Vector2D getCoPRateHighWeight();

   /**
    * Gets the number of basis vectors to use per contact point.
    * <p>
    * This parameter is used in {@link InverseDynamicsOptimizationControlModule} and
    * {@link VirtualModelControlOptimizationControlModule} which are use when running the
    * {@link WholeBodyControllerCore} in {@link WholeBodyControllerCoreMode#INVERSE_DYNAMICS} or
    * {@link WholeBodyControllerCoreMode#VIRTUAL_MODEL} respectively.
    * </p>
    * <p>
    * The value should very likely be {@code 4} as a lower value would be too conservative and
    * higher values would be too computationally expensive.
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
    * Gets the number of contactable bodies for the entire robot, i.e. the number of bodies that can
    * be used to bear the robot weight.
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
    * The notation 'rho' is used to describe the contact force magnitude to exert at each basis
    * vector of each contact point. As the setup is somewhat hectic to explain, you can refer to the
    * following paper at page 9: <br>
    * <a href=
    * "https://www.researchgate.net/publication/280839675_Design_of_a_Momentum-Based_Control_Framework_and_Application_to_the_Humanoid_Robot_Atlas">
    * Design of a Momentum-Based Control Framework and Application to the Humanoid Robot Atlas.</a>
    * </p>
    * 
    * @return the size of the vector rho.
    */
   int getRhoSize();

}