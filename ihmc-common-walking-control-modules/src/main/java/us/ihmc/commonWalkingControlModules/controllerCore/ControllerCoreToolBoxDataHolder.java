package us.ihmc.commonWalkingControlModules.controllerCore;

import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.PlaneContactWrenchProcessor;
import us.ihmc.commonWalkingControlModules.momentumBasedController.WholeBodyControllerBoundCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ContactWrenchMatrixCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointTorqueMinimizationWeightCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MotionQPInputCalculator;
import us.ihmc.commonWalkingControlModules.visualizer.WrenchVisualizer;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchMatrixCalculator;
import us.ihmc.mecano.algorithms.MultiBodyGravityGradientCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.screwTheory.GravityCoriolisExternalWrenchMatrixCalculator;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;

import java.util.List;

/**
 * This dataHolder is for the sharing toolbox between the controllerThread and wholebodyControllerThread
 * Shared controllerCoreToolboxDataHolder will be used to construct the controllerCore in the wholebodycontrollercorethread
 * each state, each controller has its own controllertoolbox. This is to share the different controllercoretoolbox for each state and controllers.
 * In the wholebodyControllercorethread, the same statemachine will be setup for the same structure.
 */
public class ControllerCoreToolBoxDataHolder
{
   private final boolean hasControlDT = false;
   private double controlDT;
   private final boolean hasGravityZ = false;
   private double gravityZ;
   private final boolean hasTotalMassProvider = false;
   private DoubleProvider totalMassProvider;
   private final boolean hasRootJoint = false;
//   private final FloatingJointBasics rootJoint;
//
//   private final boolean hasMultiBodySystemInput = false;
//   private final MultiBodySystemBasics multiBodySystemInput;
//   private final boolean hasKinematicLoopFunctions = false;
//   private final List<KinematicLoopFunction> kinematicLoopFunctions = new ArrayList<>();
//   private final boolean hasCenterOfMassFrame = false;
//   private final ReferenceFrame centerOfMassFrame;
//   private final boolean hasOptimizationSettings = false;
//   private final ControllerCoreOptimizationSettings optimizationSettings;
//   private final boolean hasFeedbackControllerSettings = false;
//   private final FeedbackControllerSettings feedbackControllerSettings = FeedbackControllerSettings.getDefault();
//   private final boolean hasYoGraphicsListRegistry = false;
//   private final YoGraphicsListRegistry yoGraphicsListRegistry;
//
//   private final boolean hasJointIndexHandler = false;
//   private final JointIndexHandler jointIndexHandler;
//   private final boolean hasInactiveOneDoFJoints = false;
//   private final List<OneDoFJointBasics> inactiveOneDoFJoints = new ArrayList<>();
//   private final boolean hasCenteroidalMomentumCalculator = false;
//   private CentroidalMomentumCalculator centroidalMomentumCalculator;
//   private CentroidalMomentumRateCalculator centroidalMomentumRateCalculator;
//   // TODO The mass-matrix calculator (when created) should be used for computing the momentum stuff. Probably need some interface and API improvements.
//   private CompositeRigidBodyMassMatrixCalculator massMatrixCalculator;
//   private final InverseDynamicsCalculator inverseDynamicsCalculator;
//   /**
//    * Used with the inverse dynamics module to compute the achieved accelerations from the QP solution.
//    */
//   private final RigidBodyAccelerationProvider rigidBodyAccelerationProvider;
//   /**
//    * Used with the inverse kinematics module to compute the achieved velocities from the QP solution.
//    */
//   private final RigidBodyTwistCalculator rigidBodyTwistCalculator;
   private JointTorqueMinimizationWeightCalculator jointTorqueMinimizationWeightCalculator;
   /**
    * Calculator used to formulate minimization of the joint torques due to gravity compensation.
    */
   private MultiBodyGravityGradientCalculator gravityGradientCalculator;
   /**
    * Calculator used to formulate the torque minimization objective. Allows to evaluate the joint
    * efforts due to: gravity, Coriolis, and centrifugal accelerations and external wrenches that are
    * not part of the optimization.
    */
   private GravityCoriolisExternalWrenchMatrixCalculator gravityCoriolisExternalWrenchMatrixCalculator;
   /**
    * Calculator used to formulate the torque minimization objective. Allows to evaluates the Jacobian
    * from joint efforts to &rho;s.
    */
   private ContactWrenchMatrixCalculator contactWrenchMatrixCalculator;

   private RigidBodyBasics vmcMainBody;

   private List<? extends ContactablePlaneBody> contactablePlaneBodies;

   private JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters;

   private PlaneContactWrenchProcessor planeContactWrenchProcessor;
   private WrenchVisualizer wrenchVisualizer;

   private YoFrameVector3D yoDesiredMomentumRateLinear;
   private YoFrameVector3D yoAchievedMomentumRateLinear;
   private YoFrameVector3D yoDesiredMomentumRateAngular;
   private YoFrameVector3D yoAchievedMomentumRateAngular;

   private YoFrameVector3D yoDesiredMomentumLinear;
   private YoFrameVector3D yoDesiredMomentumAngular;
   private YoFrameVector3D yoAchievedMomentumLinear;
   private YoFrameVector3D yoAchievedMomentumAngular;

   private YoFrameVector3D yoResidualRootJointForce;
   private YoFrameVector3D yoResidualRootJointTorque;

   private MotionQPInputCalculator motionQPInputCalculator;
   private WholeBodyControllerBoundCalculator qpBoundCalculator;
   private WrenchMatrixCalculator wrenchMatrixCalculator;

   private final boolean enableInverseDynamicsModule = false;
   private final boolean enableInverseKinematicsModule = false;
   private final boolean enableVirtualModelControlModule = false;

   public ControllerCoreToolBoxDataHolder()
   {
      clear();
   }

   public void clear()
   {
   }

   public void set(ControllerCoreToolBoxDataHolder other)
   {
      this.controlDT = other.getControlDT();
   }

   public double getControlDT()
   {
      return controlDT;
   }

   public double getGravityZ()
   {
      return gravityZ;
   }
//
//   public FloatingJointBasics getRootJoint()
//   {
//      return rootJoint;
//   }
//
//   public ControllerCoreOptimizationSettings getOptimizationSettings()
//   {
//      return optimizationSettings;
//   }
//
//   public MultiBodySystemBasics getMultiBodySystemInput()
//   {
//      return multiBodySystemInput;
//   }
//
//   public ReferenceFrame getCenterOfMassFrame()
//   {
//      return centerOfMassFrame;
//   }
//
//   public JointIndexHandler getJointIndexHandler()
//   {
//      return jointIndexHandler;
//   }
//
//   public InverseDynamicsCalculator getInverseDynamicsCalculator()
//   {
//      return inverseDynamicsCalculator;
//   }
//
//   public RigidBodyAccelerationProvider getRigidBodyAccelerationProvider()
//   {
//      return rigidBodyAccelerationProvider;
//   }
//
//   public RigidBodyTwistCalculator getRigidBodyTwistCalculator()
//   {
//      return rigidBodyTwistCalculator;
//   }

   public DoubleProvider getTotalMassProvider()
   {
      return totalMassProvider;
   }
}
