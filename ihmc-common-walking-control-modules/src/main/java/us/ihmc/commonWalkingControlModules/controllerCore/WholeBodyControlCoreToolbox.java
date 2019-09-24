package us.ihmc.commonWalkingControlModules.controllerCore;

import static us.ihmc.commonWalkingControlModules.visualizer.WrenchVisualizer.createWrenchVisualizerWithContactableBodies;

import java.util.List;
import java.util.Objects;

import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.inverseKinematics.JointPrivilegedConfigurationHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.PlaneContactWrenchProcessor;
import us.ihmc.commonWalkingControlModules.momentumBasedController.WholeBodyControllerBoundCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
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
import us.ihmc.mecano.algorithms.InverseDynamicsCalculator;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyAccelerationProvider;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class WholeBodyControlCoreToolbox
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final double controlDT;
   private final double gravityZ;
   private final FloatingJointBasics rootJoint;
   private final MultiBodySystemBasics multiBodySystemInput;
   private final ReferenceFrame centerOfMassFrame;
   private final ControllerCoreOptimizationSettings optimizationSettings;
   private FeedbackControllerSettings feedbackControllerSettings = FeedbackControllerSettings.getDefault();
   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private final JointIndexHandler jointIndexHandler;
   private final double totalRobotMass;
   private CentroidalMomentumCalculator centroidalMomentumCalculator;
   private CentroidalMomentumRateCalculator centroidalMomentumRateCalculator;
   private final InverseDynamicsCalculator inverseDynamicsCalculator;
   private final RigidBodyAccelerationProvider rigidBodyAccelerationProvider;

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
   public WholeBodyControlCoreToolbox(double controlDT, double gravityZ, FloatingJointBasics rootJoint, JointBasics[] controlledJoints,
                                      ReferenceFrame centerOfMassFrame, ControllerCoreOptimizationSettings controllerCoreOptimizationSettings,
                                      YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.controlDT = controlDT;
      this.gravityZ = gravityZ;
      this.rootJoint = rootJoint;
      this.centerOfMassFrame = centerOfMassFrame;
      this.optimizationSettings = controllerCoreOptimizationSettings;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;

      multiBodySystemInput = MultiBodySystemBasics.toMultiBodySystemBasics(controlledJoints);
      jointIndexHandler = new JointIndexHandler(controlledJoints);
      totalRobotMass = TotalMassCalculator.computeSubTreeMass(multiBodySystemInput.getRootBody());
      inverseDynamicsCalculator = new InverseDynamicsCalculator(multiBodySystemInput);
      inverseDynamicsCalculator.setGravitionalAcceleration(-gravityZ); // Watch out for the sign here, it changed with the switch to Mecano.
      rigidBodyAccelerationProvider = inverseDynamicsCalculator.getAccelerationProvider();

      parentRegistry.addChild(registry);
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
      centroidalMomentumRateCalculator = new CentroidalMomentumRateCalculator(multiBodySystemInput, centerOfMassFrame);
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
      enableInverseKinematicsModule = true;
      // TODO add tools specific to the inverse kinematics module here.
      if (centroidalMomentumRateCalculator == null)
         centroidalMomentumCalculator = new CentroidalMomentumCalculator(multiBodySystemInput, centerOfMassFrame);
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
      centroidalMomentumRateCalculator = new CentroidalMomentumRateCalculator(multiBodySystemInput, centerOfMassFrame);
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

   public MotionQPInputCalculator getMotionQPInputCalculator()
   {
      if (motionQPInputCalculator == null)
      {
         if (centroidalMomentumRateCalculator != null)
         {
            motionQPInputCalculator = new MotionQPInputCalculator(centerOfMassFrame,
                                                                  centroidalMomentumRateCalculator,
                                                                  jointIndexHandler,
                                                                  jointPrivilegedConfigurationParameters,
                                                                  registry);
         }
         else
         {
            Objects.requireNonNull(centroidalMomentumCalculator);

            motionQPInputCalculator = new MotionQPInputCalculator(centerOfMassFrame,
                                                                  centroidalMomentumCalculator,
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

   public FloatingJointBasics getRootJoint()
   {
      return rootJoint;
   }

   public RigidBodyBasics getRootBody()
   {
      return multiBodySystemInput.getRootBody();
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

   public double getTotalRobotMass()
   {
      return totalRobotMass;
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

   public WrenchVisualizer getWrenchVisualizer()
   {
      if (yoGraphicsListRegistry == null)
         return null;
      if (wrenchVisualizer == null)
         wrenchVisualizer = createWrenchVisualizerWithContactableBodies("DesiredExternalWrench", contactablePlaneBodies, 1.0, yoGraphicsListRegistry, registry);
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
}
