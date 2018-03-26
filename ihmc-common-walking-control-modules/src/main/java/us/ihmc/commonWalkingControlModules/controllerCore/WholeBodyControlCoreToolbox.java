package us.ihmc.commonWalkingControlModules.controllerCore;

import static us.ihmc.commonWalkingControlModules.visualizer.WrenchVisualizer.createWrenchVisualizerWithContactableBodies;

import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.inverseKinematics.JointPrivilegedConfigurationHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.PlaneContactWrenchProcessor;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.CentroidalMomentumHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.InverseDynamicsQPBoundCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MotionQPInputCalculator;
import us.ihmc.commonWalkingControlModules.visualizer.WrenchVisualizer;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchMatrixCalculator;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculator;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class WholeBodyControlCoreToolbox
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final String namePrefix;
   private final double controlDT;
   private final double gravityZ;
   private final FloatingInverseDynamicsJoint rootJoint;
   private final RigidBody rootBody;
   private final ReferenceFrame centerOfMassFrame;
   private final ControllerCoreOptimizationSettings optimizationSettings;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private final JointIndexHandler jointIndexHandler;
   private final double totalRobotMass;
   private final CentroidalMomentumHandler centroidalMomentumHandler;
   private final InverseDynamicsCalculator inverseDynamicsCalculator;
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;

   private RigidBody vmcMainBody;
   private RigidBody[] controlledBodies;

   private List<? extends ContactablePlaneBody> contactablePlaneBodies;

   private JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters;

   private PlaneContactWrenchProcessor planeContactWrenchProcessor;
   private WrenchVisualizer wrenchVisualizer;

   private YoFrameVector yoDesiredMomentumRateLinear;
   private YoFrameVector yoAchievedMomentumRateLinear;
   private YoFrameVector yoDesiredMomentumRateAngular;
   private YoFrameVector yoAchievedMomentumRateAngular;

   private YoFrameVector yoResidualRootJointForce;
   private YoFrameVector yoResidualRootJointTorque;

   private MotionQPInputCalculator motionQPInputCalculator;
   private InverseDynamicsQPBoundCalculator qpBoundCalculator;
   private WrenchMatrixCalculator wrenchMatrixCalculator;

   private boolean enableInverseDynamicsModule = false;
   private boolean enableInverseKinematicsModule = false;
   private boolean enableVirtualModelControlModule = false;

   /**
    * Creates a new toolbox with the required parameters for running any of the controller core
    * modules.
    * <p>
    * These parameters are necessary but not sufficient. To ensure that this toolbox is setup for
    * one or more control core module(s), the following methods have to be called depending on the
    * usage:
    * <ul>
    * <li>{@link #setupForInverseDynamicsSolver(List)} to complete the parameters necessary and
    * sufficient to run the inverse dynamics module.
    * <li>{@link #setupForInverseKinematicsSolver()} to complete the parameters necessary and
    * sufficient to run the inverse kinematics module. Not that at the moment this method is empty,
    * it should be called wherever the inverse kinematics module is called in case in the future new
    * parameters are added.
    * <li>{@link #setupForVirtualModelControlSolver(RigidBody, RigidBody[])} to complete the
    * parameters necessary and sufficient to run the virtual model control module.
    * </ul>
    * Calling these methods will also notice the {@link WholeBodyControllerCore} at construction
    * time which module is to be created.
    * </p>
    * 
    * @param controlDT duration of one control tick.
    * @param gravityZ magnitude of the gravity assumed to be along the z-axis. The parameter is
    *           assumed to be positive.
    * @param rootJoint the floating joint of the robot.
    * @param controlledJoints the set of all the joints to be controlled.
    * @param centerOfMassFrame the reference frame centered at the robot's center of mass and
    *           oriented as world frame. This reference is assumed to be updated from outside the
    *           controller core.
    * @param controllerCoreOptimizationSettings set of parameters used to initialize the
    *           optimization problems.
    * @param yoGraphicsListRegistry the registry in which the {@link YoGraphic}s and
    *           {@link Artifact}s of the controller core are registered.
    * @param parentRegistry registry to which this toolbox will attach its own registry.
    */
   public WholeBodyControlCoreToolbox(String namePrefix, double controlDT, double gravityZ, FloatingInverseDynamicsJoint rootJoint, InverseDynamicsJoint[] controlledJoints,
                                      ReferenceFrame centerOfMassFrame, ControllerCoreOptimizationSettings controllerCoreOptimizationSettings,
                                      YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.namePrefix = namePrefix + "ControllerCoreToolbox";
      this.controlDT = controlDT;
      this.gravityZ = gravityZ;
      this.rootJoint = rootJoint;
      this.centerOfMassFrame = centerOfMassFrame;
      this.optimizationSettings = controllerCoreOptimizationSettings;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;

      rootBody = ScrewTools.getRootBody(controlledJoints[0].getPredecessor());
      jointIndexHandler = new JointIndexHandler(controlledJoints);
      //for(int i = 0; i < controlledJoints.length; i++)
      //   PrintTools.debug("ControlledJoint " + i + ": " +controlledJoints[i].getName().toString());
      totalRobotMass = TotalMassCalculator.computeSubTreeMass(rootBody);
      centroidalMomentumHandler = new CentroidalMomentumHandler(rootBody, centerOfMassFrame);
      inverseDynamicsCalculator = new InverseDynamicsCalculator(rootBody, gravityZ);
      spatialAccelerationCalculator = inverseDynamicsCalculator.getSpatialAccelerationCalculator();

      parentRegistry.addChild(registry);
   }

   /**
    * Sets the parameters necessary to initialize and enable the joint privileged configuration.
    * <p>
    * This feature is useful to complete the sets of command given to the controller core such that
    * if a set of joints remain uncontrolled they will smoothly go to a privileged
    * configuration/position. This feature slightly affects the desired commands from the high-level
    * control but is highly beneficial to keep the system in a safe state even if most of it is
    * uncontrolled and is also useful to escape singularity configurations.
    * </p>
    * 
    * @param jointPrivilegedConfigurationParameters the set of parameters necessary to initialize
    *           the {@link JointPrivilegedConfigurationHandler}.
    */
   public void setJointPrivilegedConfigurationParameters(JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters)
   {
      this.jointPrivilegedConfigurationParameters = jointPrivilegedConfigurationParameters;
   }

   /**
    * Adds the missing information necessary to enable the inverse dynamics module and notices the
    * {@link WholeBodyControllerCore} at construction time that the inverse dynamics module has to
    * be created.
    * <p>
    * WARNING: This method has be to called BEFORE creating the {@link WholeBodyControllerCore}.
    * </p>
    * 
    * @param contactablePlaneBodies the list of rigid-body which can be used to bear the robot
    *           weight.
    */
   public void setupForInverseDynamicsSolver(List<? extends ContactablePlaneBody> contactablePlaneBodies)
   {
      enableInverseDynamicsModule = true;
      // TODO add tools specific to the inverse dynamics module here.
      this.contactablePlaneBodies = contactablePlaneBodies;
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
   }

   /**
    * Adds the missing information necessary to enable the virtual model control module and notices
    * the {@link WholeBodyControllerCore} at construction time that the virtual model control module
    * has to be created.
    * <p>
    * WARNING: This method has be to called BEFORE creating the {@link WholeBodyControllerCore}.
    * </p>
    * 
    * @param vmcMainBody the main rigid-body of the robot.
    * @param controlledBodies the set of rigid-bodies that are to be controllable.
    * @param contactablePlaneBodies the list of rigid-body which can be used to bear the robot
    *           weight.
    */
   public void setupForVirtualModelControlSolver(RigidBody vmcMainBody, RigidBody[] controlledBodies,
                                                 List<? extends ContactablePlaneBody> contactablePlaneBodies)
   {
      enableVirtualModelControlModule = true;
      // TODO add tools specific to the virtual model control module here.
      this.vmcMainBody = vmcMainBody;
      this.controlledBodies = controlledBodies;
      this.contactablePlaneBodies = contactablePlaneBodies;
   }

   /**
    * Informs whereas the inverse dynamics module is setup for the controller core using this
    * toolbox.
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
         motionQPInputCalculator = new MotionQPInputCalculator(centerOfMassFrame, centroidalMomentumHandler, jointIndexHandler,
                                                               jointPrivilegedConfigurationParameters, registry);
      }
      return motionQPInputCalculator;
   }

   public InverseDynamicsQPBoundCalculator getQPBoundCalculator()
   {
      if (qpBoundCalculator == null)
      {
         boolean areJointVelocityLimitsConsidered = optimizationSettings.areJointVelocityLimitsConsidered();
         qpBoundCalculator = new InverseDynamicsQPBoundCalculator(jointIndexHandler, controlDT, areJointVelocityLimitsConsidered, registry);
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

   public JointPrivilegedConfigurationParameters getJointPrivilegedConfigurationParameters()
   {
      return jointPrivilegedConfigurationParameters;
   }

   public SpatialAccelerationCalculator getSpatialAccelerationCalculator()
   {
      return spatialAccelerationCalculator;
   }

   public InverseDynamicsCalculator getInverseDynamicsCalculator()
   {
      return inverseDynamicsCalculator;
   }

   /**
    * <b>Important note</b>: the {@code CentroidalMomentumHandler} is updated every control tick in
    * {@link MotionQPInputCalculator#initialize()}.
    * <p>
    * Gets the {@code CentroidalMomentumHandler} which allows to calculate:
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
   public CentroidalMomentumHandler getCentroidalMomentumHandler()
   {
      return centroidalMomentumHandler;
   }

   public FloatingInverseDynamicsJoint getRootJoint()
   {
      return rootJoint;
   }

   public RigidBody getRootBody()
   {
      return rootBody;
   }

   public RigidBody getVirtualModelControlMainBody()
   {
      return vmcMainBody;
   }

   public RigidBody[] getControlledBodies()
   {
      return controlledBodies;
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
         planeContactWrenchProcessor = new PlaneContactWrenchProcessor(namePrefix, contactablePlaneBodies, yoGraphicsListRegistry, registry);
      return planeContactWrenchProcessor;
   }

   public CenterOfPressureDataHolder getDesiredCenterOfPressureDataHolder()
   {
      return getPlaneContactWrenchProcessor().getDesiredCenterOfPressureDataHolder();
   }

   public WrenchVisualizer getWrenchVisualizer()
   {
      if (wrenchVisualizer == null)
         wrenchVisualizer = createWrenchVisualizerWithContactableBodies(namePrefix + "DesiredExternalWrench", contactablePlaneBodies, 1.0, yoGraphicsListRegistry, registry);
      return wrenchVisualizer;
   }

   public YoFrameVector getYoDesiredMomentumRateLinear()
   {
      if (yoDesiredMomentumRateLinear == null)
         yoDesiredMomentumRateLinear = new YoFrameVector("desiredMomentumRateLinear", worldFrame, registry);
      return yoDesiredMomentumRateLinear;
   }

   public YoFrameVector getYoAchievedMomentumRateLinear()
   {
      if (yoAchievedMomentumRateLinear == null)
         yoAchievedMomentumRateLinear = new YoFrameVector("achievedMomentumRateLinear", worldFrame, registry);
      return yoAchievedMomentumRateLinear;
   }

   public YoFrameVector getYoDesiredMomentumRateAngular()
   {
      if (yoDesiredMomentumRateAngular == null)
         yoDesiredMomentumRateAngular = new YoFrameVector("desiredMomentumRateAngular", worldFrame, registry);
      return yoDesiredMomentumRateAngular;
   }

   public YoFrameVector getYoAchievedMomentumRateAngular()
   {
      if (yoAchievedMomentumRateAngular == null)
         yoAchievedMomentumRateAngular = new YoFrameVector("achievedMomentumRateAngular", worldFrame, registry);
      return yoAchievedMomentumRateAngular;
   }

   public YoFrameVector getYoResidualRootJointForce()
   {
      if (yoResidualRootJointForce == null)
         yoResidualRootJointForce = new YoFrameVector("residualRootJointForce", worldFrame, registry);
      return yoResidualRootJointForce;
   }

   public YoFrameVector getYoResidualRootJointTorque()
   {
      if (yoResidualRootJointTorque == null)
         yoResidualRootJointTorque = new YoFrameVector("residualRootJointTorque", worldFrame, registry);
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
