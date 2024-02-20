package us.ihmc.commonWalkingControlModules.controllerCore;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.stream.Stream;

import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.DesiredExternalWrenchHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsOptimizationSettingsCommand;
import us.ihmc.commonWalkingControlModules.inverseKinematics.JointPrivilegedConfigurationHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.PlaneContactWrenchProcessor;
import us.ihmc.commonWalkingControlModules.momentumBasedController.WholeBodyControllerBoundCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ContactWrenchMatrixCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointTorqueMinimizationWeightCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MotionQPInputCalculator;
import us.ihmc.commonWalkingControlModules.visualizer.WrenchVisualizer;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchMatrixCalculator;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.mecano.algorithms.CentroidalMomentumCalculator;
import us.ihmc.mecano.algorithms.CentroidalMomentumRateCalculator;
import us.ihmc.mecano.algorithms.CompositeRigidBodyMassMatrixCalculator;
import us.ihmc.mecano.algorithms.InverseDynamicsCalculator;
import us.ihmc.mecano.algorithms.MultiBodyGravityGradientCalculator;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyAccelerationProvider;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.KinematicLoopFunction;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.screwTheory.GravityCoriolisExternalWrenchMatrixCalculator;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class WholeBodyControlCoreToolbox implements SCS2YoGraphicHolder
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final double controlDT;
   private final double gravityZ;
   private final FloatingJointBasics rootJoint;
   private RigidBodyBasics elevator;
   private final List<KinematicLoopFunction> kinematicLoopFunctions = new ArrayList<>();
   private final ReferenceFrame centerOfMassFrame;
   private final ControllerCoreOptimizationSettings optimizationSettings;
   private FeedbackControllerSettings feedbackControllerSettings = FeedbackControllerSettings.getDefault();
   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private final JointIndexHandler jointIndexHandler;
   private final List<OneDoFJointBasics> inactiveOneDoFJoints = new ArrayList<>();
   private CentroidalMomentumCalculator centroidalMomentumCalculator;
   private CentroidalMomentumRateCalculator centroidalMomentumRateCalculator;
   // TODO The mass-matrix calculator (when created) should be used for computing the momentum stuff. Probably need some interface and API improvements.
   private CompositeRigidBodyMassMatrixCalculator massMatrixCalculator;
   private final InverseDynamicsCalculator inverseDynamicsCalculator;
   private final RigidBodyAccelerationProvider rigidBodyAccelerationProvider;
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

   private boolean enableInverseDynamicsModule = false;
   private boolean enableInverseKinematicsModule = false;
   private boolean enableVirtualModelControlModule = false;

   /**
    * Creates a new toolbox with the required parameters for running any of the controller core
    * modules.
    * <p>
    * These parameters are necessary but not sufficient. To ensure that this toolbox is setup for one
    * or more control core module(s), the following methods have to be called depending on the usage:
    * <ul>
    * <li>{@link #setupForInverseDynamicsSolver(List)} to complete the parameters necessary and
    * sufficient to run the inverse dynamics module.
    * <li>{@link #setupForInverseKinematicsSolver()} to complete the parameters necessary and
    * sufficient to run the inverse kinematics module. Not that at the moment this method is empty, it
    * should be called wherever the inverse kinematics module is called in case in the future new
    * parameters are added.
    * <li>{@link #setupForVirtualModelControlSolver(RigidBodyBasics)} to complete the parameters
    * necessary and sufficient to run the virtual model control module.
    * </ul>
    * Calling these methods will also notice the {@link WholeBodyControllerCore} at construction time
    * which module is to be created.
    * </p>
    *
    * @param controlDT                          duration of one control tick.
    * @param gravityZ                           magnitude of the gravity assumed to be along the
    *                                           z-axis. The parameter is assumed to be positive.
    * @param rootJoint                          the floating joint of the robot.
    * @param controlledJoints                   the set of all the joints to be controlled.
    * @param centerOfMassFrame                  the reference frame centered at the robot's center of
    *                                           mass and oriented as world frame. This reference is
    *                                           assumed to be updated from outside the controller core.
    * @param controllerCoreOptimizationSettings set of parameters used to initialize the optimization
    *                                           problems.
    * @param yoGraphicsListRegistry             the registry in which the {@link YoGraphic}s and
    *                                           {@link Artifact}s of the controller core are
    *                                           registered.
    * @param parentRegistry                     registry to which this toolbox will attach its own
    *                                           registry.
    */
   public WholeBodyControlCoreToolbox(double controlDT,
                                      double gravityZ,
                                      FloatingJointBasics rootJoint,
                                      JointBasics[] controlledJoints,
                                      ReferenceFrame centerOfMassFrame,
                                      ControllerCoreOptimizationSettings controllerCoreOptimizationSettings,
                                      YoGraphicsListRegistry yoGraphicsListRegistry,
                                      YoRegistry parentRegistry)
   {
      this.controlDT = controlDT;
      this.gravityZ = gravityZ;
      this.rootJoint = rootJoint;
      this.optimizationSettings = controllerCoreOptimizationSettings;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;

      if (rootJoint != null && Stream.of(controlledJoints).noneMatch(joint -> joint == rootJoint))
      {
         JointBasics[] controlledJointsWithRoot = new JointBasics[controlledJoints.length + 1];
         System.arraycopy(controlledJoints, 0, controlledJointsWithRoot, 1, controlledJoints.length);
         controlledJointsWithRoot[0] = rootJoint;
         controlledJoints = controlledJointsWithRoot;
         
      }

      elevator = rootJoint.getPredecessor();

      if (centerOfMassFrame == null)
         centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", elevator.getBodyFixedFrame().getRootFrame(), elevator);
      this.centerOfMassFrame = centerOfMassFrame;

      jointIndexHandler = new JointIndexHandler(controlledJoints);
      inverseDynamicsCalculator = new InverseDynamicsCalculator(elevator);
      inverseDynamicsCalculator.setGravitionalAcceleration(-gravityZ); // Watch out for the sign here, it changed with the switch to Mecano.
      rigidBodyAccelerationProvider = inverseDynamicsCalculator.getAccelerationProvider();

      parentRegistry.addChild(registry);
   }

   /**
    * Registers an new function for handling a kinematic loop in the multi-body system.
    * 
    * @param function the constraint function for one kinematic loop.
    */
   public void addKinematicLoopFunction(KinematicLoopFunction function)
   {
      kinematicLoopFunctions.add(function);
   }

   /**
    * Registers a joint as inactive, i.e. it cannot be controlled but should still be considered.
    * <p>
    * The list of inactive joints can be modified at runtime via
    * {@link InverseDynamicsOptimizationSettingsCommand}.
    * </p>
    * 
    * @param inactiveJoint the joint to be registered as inactive.
    */
   public void addInactiveJoint(OneDoFJointBasics inactiveJoint)
   {
      if (!inactiveOneDoFJoints.contains(inactiveJoint))
         inactiveOneDoFJoints.add(inactiveJoint);
   }

   /**
    * Sets the parameters necessary to initialize and enable the joint privileged configuration.
    * <p>
    * This feature is useful to complete the sets of command given to the controller core such that if
    * a set of joints remain uncontrolled they will smoothly go to a privileged configuration/position.
    * This feature slightly affects the desired commands from the high-level control but is highly
    * beneficial to keep the system in a safe state even if most of it is uncontrolled and is also
    * useful to escape singularity configurations.
    * </p>
    *
    * @param jointPrivilegedConfigurationParameters the set of parameters necessary to initialize the
    *                                               {@link JointPrivilegedConfigurationHandler}.
    */
   public void setJointPrivilegedConfigurationParameters(JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters)
   {
      this.jointPrivilegedConfigurationParameters = jointPrivilegedConfigurationParameters;
   }

   /**
    * Provides the settings to use for configuring the {@code WholeBodyFeedbackController}.
    * <p>
    * This has to be provided before creating the controller core.
    * </p>
    * 
    * @param feedbackControllerSettings the settings to use.
    */
   public void setFeedbackControllerSettings(FeedbackControllerSettings feedbackControllerSettings)
   {
      if (feedbackControllerSettings != null)
         this.feedbackControllerSettings = feedbackControllerSettings;
   }

   /**
    * Adds the missing information necessary to enable the inverse dynamics module and notices the
    * {@link WholeBodyControllerCore} at construction time that the inverse dynamics module has to be
    * created.
    * <p>
    * WARNING: This method has be to called BEFORE creating the {@link WholeBodyControllerCore}.
    * </p>
    *
    * @param contactablePlaneBodies the list of rigid-body which can be used to bear the robot weight.
    */
   public void setupForInverseDynamicsSolver(List<? extends ContactablePlaneBody> contactablePlaneBodies)
   {
      enableInverseDynamicsModule = true;
      // TODO add tools specific to the inverse dynamics module here.
      this.contactablePlaneBodies = contactablePlaneBodies;
      if (centroidalMomentumCalculator != null)
         centroidalMomentumCalculator = null;
      centroidalMomentumRateCalculator = new CentroidalMomentumRateCalculator(elevator, centerOfMassFrame);
   }

   /**
    * Notices the {@link WholeBodyControllerCore} at construction time that the inverse kinematics
    * module has to be created.
    * <p>
    * WARNING: This method has be to called BEFORE creating the {@link WholeBodyControllerCore}.
    * </p>
    */
   public void setupForInverseKinematicsSolver()
   {
      setupForInverseKinematicsSolver(null);
   }

   /**
    * Notices the {@link WholeBodyControllerCore} at construction time that the inverse kinematics
    * module has to be created.
    * <p>
    * WARNING: This method has be to called BEFORE creating the {@link WholeBodyControllerCore}.
    * </p>
    */
   public void setupForInverseKinematicsSolver(JointTorqueMinimizationWeightCalculator calculator)
   {
      enableInverseKinematicsModule = true;
      // TODO add tools specific to the inverse kinematics module here.
      if (centroidalMomentumRateCalculator == null)
         centroidalMomentumCalculator = new CentroidalMomentumCalculator(elevator, centerOfMassFrame);
      if (calculator != null)
      {
         jointTorqueMinimizationWeightCalculator = calculator;
         if (calculator.getRegistry() != null)
            registry.addChild(calculator.getRegistry());
      }
   }

   /**
    * Adds the missing information necessary to enable the virtual model control module and notices the
    * {@link WholeBodyControllerCore} at construction time that the virtual model control module has to
    * be created.
    * <p>
    * WARNING: This method has be to called BEFORE creating the {@link WholeBodyControllerCore}.
    * </p>
    *
    * @param vmcMainBody            the main rigid-body of the robot.
    * @param controlledBodies       the set of rigid-bodies that are to be controllable.
    * @param contactablePlaneBodies the list of rigid-body which can be used to bear the robot weight.
    */
   public void setupForVirtualModelControlSolver(RigidBodyBasics vmcMainBody, List<? extends ContactablePlaneBody> contactablePlaneBodies)
   {
      enableVirtualModelControlModule = true;
      // TODO add tools specific to the virtual model control module here.
      this.vmcMainBody = vmcMainBody;
      this.contactablePlaneBodies = contactablePlaneBodies;
      if (centroidalMomentumCalculator != null)
         centroidalMomentumCalculator = null;
      centroidalMomentumRateCalculator = new CentroidalMomentumRateCalculator(elevator, centerOfMassFrame);
   }

   /**
    * Informs whereas the inverse dynamics module is setup for the controller core using this toolbox.
    *
    * @return {@code true} if the inverse dynamics module is setup, {@code false} otherwise.
    */
   public boolean isEnableInverseDynamicsModule()
   {
      return enableInverseDynamicsModule;
   }

   /**
    * Informs whereas the inverse kinematics module is setup for the controller core using this
    * toolbox.
    *
    * @return {@code true} if the inverse kinematics module is setup, {@code false} otherwise.
    */
   public boolean isEnableInverseKinematicsModule()
   {
      return enableInverseKinematicsModule;
   }

   /**
    * Informs whereas the virtual model control module is setup for the controller core using this
    * toolbox.
    *
    * @return {@code true} if the virtual model control module is setup, {@code false} otherwise.
    */
   public boolean isEnableVirtualModelControlModule()
   {
      return enableVirtualModelControlModule;
   }

   public List<KinematicLoopFunction> getKinematicLoopFunctions()
   {
      return kinematicLoopFunctions;
   }

   public MotionQPInputCalculator getMotionQPInputCalculator()
   {
      if (motionQPInputCalculator == null)
      {
         if (centroidalMomentumRateCalculator != null)
         {
            motionQPInputCalculator = new MotionQPInputCalculator(centerOfMassFrame,
                                                                  centroidalMomentumRateCalculator,
                                                                  getGravityGradientCalculator(),
                                                                  jointIndexHandler,
                                                                  jointPrivilegedConfigurationParameters,
                                                                  registry);
         }
         else
         {
            Objects.requireNonNull(centroidalMomentumCalculator);

            motionQPInputCalculator = new MotionQPInputCalculator(centerOfMassFrame,
                                                                  centroidalMomentumCalculator,
                                                                  getGravityGradientCalculator(),
                                                                  jointIndexHandler,
                                                                  jointPrivilegedConfigurationParameters,
                                                                  registry);
         }
      }
      return motionQPInputCalculator;
   }

   public WholeBodyControllerBoundCalculator getQPBoundCalculator()
   {
      if (qpBoundCalculator == null)
      {
         boolean areJointVelocityLimitsConsidered = optimizationSettings.areJointVelocityLimitsConsidered();
         qpBoundCalculator = new WholeBodyControllerBoundCalculator(jointIndexHandler, controlDT, areJointVelocityLimitsConsidered, registry);
      }
      return qpBoundCalculator;
   }

   public WrenchMatrixCalculator getWrenchMatrixCalculator()
   {
      if (wrenchMatrixCalculator == null)
         wrenchMatrixCalculator = new WrenchMatrixCalculator(this, registry);
      return wrenchMatrixCalculator;
   }

   public ControllerCoreOptimizationSettings getOptimizationSettings()
   {
      return optimizationSettings;
   }

   public FeedbackControllerSettings getFeedbackControllerSettings()
   {
      return feedbackControllerSettings;
   }

   public JointPrivilegedConfigurationParameters getJointPrivilegedConfigurationParameters()
   {
      return jointPrivilegedConfigurationParameters;
   }

   public RigidBodyAccelerationProvider getRigidBodyAccelerationProvider()
   {
      return rigidBodyAccelerationProvider;
   }

   public InverseDynamicsCalculator getInverseDynamicsCalculator()
   {
      return inverseDynamicsCalculator;
   }

   public JointTorqueMinimizationWeightCalculator getJointTorqueMinimizationWeightCalculator()
   {
      return jointTorqueMinimizationWeightCalculator;
   }

   public GravityCoriolisExternalWrenchMatrixCalculator getGravityCoriolisExternalWrenchMatrixCalculator()
   {
      if (gravityCoriolisExternalWrenchMatrixCalculator == null)
      {
         gravityCoriolisExternalWrenchMatrixCalculator = new GravityCoriolisExternalWrenchMatrixCalculator(elevator);
         gravityCoriolisExternalWrenchMatrixCalculator.setGravitionalAcceleration(-Math.abs(gravityZ));
      }
      return gravityCoriolisExternalWrenchMatrixCalculator;
   }

   public MultiBodyGravityGradientCalculator getGravityGradientCalculator()
   {
      if (gravityGradientCalculator == null)
      {
         gravityGradientCalculator = new MultiBodyGravityGradientCalculator(elevator);
         gravityGradientCalculator.setGravitionalAcceleration(-Math.abs(gravityZ));
      }
      return gravityGradientCalculator;
   }

   public ContactWrenchMatrixCalculator getContactWrenchMatrixCalculator()
   {
      if (contactWrenchMatrixCalculator == null)
         contactWrenchMatrixCalculator = new ContactWrenchMatrixCalculator(this);
      return contactWrenchMatrixCalculator;
   }

   /**
    * <b>Important note</b>: the {@code CentroidalMomentumRateCalculator} is updated every control tick
    * in {@link MotionQPInputCalculator#initialize()}.
    * <p>
    * Gets the {@code CentroidalMomentumRateCalculator} which allows to calculate:
    * <ul>
    * <li>the robot momentum, often denoted 'h', and center of mass velocity.
    * <li>the centroidal momentum matrix, often denoted 'A', which is the N-by-6 Jacobian matrix
    * mapping joint velocities to momentum. N is equal to the number of degrees of freedom for the
    * robot.
    * <li>the convective term in the equation of the rate of change of momentum 'hDot':<br>
    * hDot = A * vDot + ADot * v<br>
    * where v and vDot are the vectors of joint velocities and accelerations respectively.<br>
    * The convective term is: ADot * v.
    * </ul>
    * </p>
    *
    * @return the centroidalMomentumHandler.
    */
   public CentroidalMomentumRateCalculator getCentroidalMomentumRateCalculator()
   {
      return centroidalMomentumRateCalculator;
   }

   public CompositeRigidBodyMassMatrixCalculator getMassMatrixCalculator()
   {
      if (massMatrixCalculator == null)
         massMatrixCalculator = new CompositeRigidBodyMassMatrixCalculator(elevator);
      return massMatrixCalculator;
   }

   public FloatingJointBasics getRootJoint()
   {
      return rootJoint;
   }

   public RigidBodyBasics getRootBody()
   {
      return elevator;
   }

   public RigidBodyBasics getVirtualModelControlMainBody()
   {
      return vmcMainBody;
   }

   public double getControlDT()
   {
      return controlDT;
   }

   public ReferenceFrame getCenterOfMassFrame()
   {
      return centerOfMassFrame;
   }

   public double getGravityZ()
   {
      return gravityZ;
   }

   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return yoGraphicsListRegistry;
   }

   public List<? extends ContactablePlaneBody> getContactablePlaneBodies()
   {
      return contactablePlaneBodies;
   }

   public PlaneContactWrenchProcessor getPlaneContactWrenchProcessor()
   {
      if (planeContactWrenchProcessor == null)
         planeContactWrenchProcessor = new PlaneContactWrenchProcessor(contactablePlaneBodies, yoGraphicsListRegistry, registry);
      return planeContactWrenchProcessor;
   }

   public CenterOfPressureDataHolder getDesiredCenterOfPressureDataHolder()
   {
      return getPlaneContactWrenchProcessor().getDesiredCenterOfPressureDataHolder();
   }

   public DesiredExternalWrenchHolder getDesiredExternalWrenchHolder()
   {
      return getPlaneContactWrenchProcessor().getDesiredExternalWrenchHolder();
   }

   public WrenchVisualizer getWrenchVisualizer()
   {
      if (yoGraphicsListRegistry == null)
         return null;
      if (wrenchVisualizer == null)
      {
         wrenchVisualizer = new WrenchVisualizer("DesiredExternal", 1.0, yoGraphicsListRegistry, registry);
         wrenchVisualizer.registerContactablePlaneBodies(contactablePlaneBodies);
      }
      return wrenchVisualizer;
   }

   public YoFrameVector3D getYoDesiredMomentumRateLinear()
   {
      if (yoDesiredMomentumRateLinear == null)
         yoDesiredMomentumRateLinear = new YoFrameVector3D("desiredMomentumRateLinear", worldFrame, registry);
      return yoDesiredMomentumRateLinear;
   }

   public YoFrameVector3D getYoAchievedMomentumRateLinear()
   {
      if (yoAchievedMomentumRateLinear == null)
         yoAchievedMomentumRateLinear = new YoFrameVector3D("achievedMomentumRateLinear", worldFrame, registry);
      return yoAchievedMomentumRateLinear;
   }

   public YoFrameVector3D getYoDesiredMomentumRateAngular()
   {
      if (yoDesiredMomentumRateAngular == null)
         yoDesiredMomentumRateAngular = new YoFrameVector3D("desiredMomentumRateAngular", worldFrame, registry);
      return yoDesiredMomentumRateAngular;
   }

   public YoFrameVector3D getYoAchievedMomentumRateAngular()
   {
      if (yoAchievedMomentumRateAngular == null)
         yoAchievedMomentumRateAngular = new YoFrameVector3D("achievedMomentumRateAngular", worldFrame, registry);
      return yoAchievedMomentumRateAngular;
   }

   public YoFrameVector3D getYoDesiredMomentumLinear()
   {
      if (yoDesiredMomentumLinear == null)
         yoDesiredMomentumLinear = new YoFrameVector3D("desiredMomentumLinear", worldFrame, registry);
      return yoDesiredMomentumLinear;
   }

   public YoFrameVector3D getYoDesiredMomentumAngular()
   {
      if (yoDesiredMomentumAngular == null)
         yoDesiredMomentumAngular = new YoFrameVector3D("desiredMomentumAngular", worldFrame, registry);
      return yoDesiredMomentumAngular;
   }

   public YoFrameVector3D getYoAchievedMomentumLinear()
   {
      if (yoAchievedMomentumLinear == null)
         yoAchievedMomentumLinear = new YoFrameVector3D("achievedMomentumLinear", worldFrame, registry);
      return yoAchievedMomentumLinear;
   }

   public YoFrameVector3D getYoAchievedMomentumAngular()
   {
      if (yoAchievedMomentumAngular == null)
         yoAchievedMomentumAngular = new YoFrameVector3D("achievedMomentumAngular", worldFrame, registry);
      return yoAchievedMomentumAngular;
   }

   public YoFrameVector3D getYoResidualRootJointForce()
   {
      if (yoResidualRootJointForce == null)
         yoResidualRootJointForce = new YoFrameVector3D("residualRootJointForce", worldFrame, registry);
      return yoResidualRootJointForce;
   }

   public YoFrameVector3D getYoResidualRootJointTorque()
   {
      if (yoResidualRootJointTorque == null)
         yoResidualRootJointTorque = new YoFrameVector3D("residualRootJointTorque", worldFrame, registry);
      return yoResidualRootJointTorque;
   }

   public JointIndexHandler getJointIndexHandler()
   {
      return jointIndexHandler;
   }

   public int getNumberOfBasisVectorsPerContactPoint()
   {
      return optimizationSettings.getNumberOfBasisVectorsPerContactPoint();
   }

   public int getNumberOfContactPointsPerContactableBody()
   {
      return optimizationSettings.getNumberOfContactPointsPerContactableBody();
   }

   public int getNumberOfContactableBodies()
   {
      return optimizationSettings.getNumberOfContactableBodies();
   }

   public int getRhoSize()
   {
      return optimizationSettings.getRhoSize();
   }

   public boolean getDeactiveRhoWhenNotInContact()
   {
      return optimizationSettings.getDeactivateRhoWhenNotInContact();
   }

   public List<OneDoFJointBasics> getInactiveOneDoFJoints()
   {
      return inactiveOneDoFJoints;
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      if (planeContactWrenchProcessor != null)
         group.addChild(planeContactWrenchProcessor.getSCS2YoGraphics());
      if (wrenchVisualizer != null)
         group.addChild(wrenchVisualizer.getSCS2YoGraphics());
      if (wrenchMatrixCalculator != null)
         group.addChild(wrenchMatrixCalculator.getSCS2YoGraphics());
      return group;
   }
}
