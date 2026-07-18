package us.ihmc.commonWalkingControlModules.parameterEstimation;

import controller_msgs.InertialAdaptationCommandMessage;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.communication.HumanoidROS2Topic;
import us.ihmc.jros2.AsyncROS2Node;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.commonWalkingControlModules.configurations.InertialEstimationParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.InverseDynamicsCalculator;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.parameterEstimation.inertial.RigidBodyInertialParameters;
import us.ihmc.parameterEstimation.inertial.RigidBodyInertialParametersTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelWrapper;
import us.ihmc.robotics.MatrixMissingTools;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.yoVariables.filters.AlphaFilterTools;
import us.ihmc.yoVariables.filters.FilteredFiniteDifferenceYoVariable;
import us.ihmc.yoVariables.math.YoMatrix;
import us.ihmc.yoVariables.filters.AlphaFilteredYoMatrix;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.*;

/**
 * This class is used to manage the estimation of the inertial parameters of a robot. It is designed to be used in a control loop, where the inertial parameters
 * are being estimated in parallel.
 *
 * @author James Foster
 */
public class InertialParameterManager implements SCS2YoGraphicHolder
{
   /** Physical inconsistency recurs every tick once it starts, so cap the logging to keep it out of the journal. */
   private static final int MAX_PHYSICAL_CONSISTENCY_WARNINGS = 10;

   private int physicalConsistencyWarningCount = 0;

   private final YoBoolean enableFilter;

   private final MultipleHumanoidModelHandler<RobotModelTask> modelHandler;
   private final HumanoidModelCovarianceHelper covarianceHelper;
   private final InertialBaselineCalculator baselineCalculator;

   // --- Per-limb operator-intent adaptation gate -----------------------------------------------------------------
   // Both forearms are always estimated (always physically consistent), but each limb's process noise Q and its
   // nominal (Tikhonov) prior are scaled at runtime according to which limb the operator is actually adapting
   // (grasping). This resolves the cross-limb identifiability failure where a one-sided load "leeches" mass off the
   // untouched arm: whole-body torque estimation sees the mass DIFFERENCE between the two forearms well (each arm's
   // own joint rows) but the SUM only weakly (distrusted floating-base rows), so a one-sided load is mis-split as
   // load-up / other-arm-down. Pinning the idle limb to nominal removes the free variable it was being dumped into.
   // Behind an enable flag; default OFF keeps every limb ACTIVE (scales 1.0) == legacy behavior.
   public enum AdaptationState { ACTIVE, IDLE_NOMINAL, IDLE_HOLD }
   private final YoBoolean intentGatedAdaptation;
   /** Liveness of the grasp signal (Phase 2 heartbeat). True by default; when false, a limb that WAS grasping holds. */
   private final YoBoolean adaptationSignalAlive;
   /** Per-side grasp input: true while the operator is grasping with that hand (set from SCS2 now, VR trigger later). */
   private final SideDependentList<YoBoolean> armGrasping = new SideDependentList<>();
   private final YoEnum<AdaptationState>[] bodyAdaptationState;   // computed per estimated body
   private final RobotSide[] bodyAdaptationSide;                  // estimated body -> side (null if not an arm)
   private final boolean[] bodyWasGrasping;                       // latch for IDLE_HOLD on signal loss
   private final YoDouble adaptationQScaleActive;
   private final YoDouble adaptationQScaleIdle;
   private final YoDouble adaptationPriorScaleActive;
   private final YoDouble adaptationPriorScaleIdleNominal;
   private final YoDouble adaptationPriorScaleIdleHold;

   /**
    * Phase 2 transport: the VR teleop interface streams per-hand grasp intent (joystick trigger held) over ROS2 to
    * drive the gate. Opened only when the estimation parameters request it
    * ({@link InertialEstimationParameters#subscribeToGraspIntent()}); the VR side publishes on the identical topic.
    * A dedicated {@link InertialAdaptationCommandMessage} (no hand-command semantics). Fixed topic = single robot
    * per network. If the stream goes stale (link lost), the gate treats a still-grasping limb as IDLE_HOLD.
    */
   public static final ROS2Topic<InertialAdaptationCommandMessage> INERTIAL_ADAPTATION_TOPIC =
         new HumanoidROS2Topic<>().withPrefix("ihmc").withModule("inertial_adaptation").withType(InertialAdaptationCommandMessage.class);
   private final ROS2Node graspROS2Node;
   private final ROS2Subscription<InertialAdaptationCommandMessage> graspSubscriber;
   private final InertialAdaptationCommandMessage latestGraspMessage;
   private final double dtSeconds;
   private double secondsSinceGraspMessage;
   private static final double GRASP_SIGNAL_TIMEOUT_SECONDS = 0.5;  // stream stale beyond this -> signal lost

   private final ArrayList<YoInertiaEllipsoid> yoInertiaEllipsoids;
   private final YoGraphicDefinition ellipsoidGraphicGroup;

   private final InverseDynamicsCalculator inverseDynamicsCalculator;
   private final JointTorqueRegressorCalculator regressorCalculator;

   private final SideDependentList<? extends FootSwitchInterface> footSwitches;
   private final SideDependentList<DMatrixRMaj> contactWrenches;
   private final JointIndexHandler jointIndexHandler;
   private final SideDependentList<JointBasics[]> legJoints;
   private final SideDependentList<GeometricJacobian> compactContactJacobians;
   private final SideDependentList<DMatrixRMaj> fullContactJacobians;

   // Two-nub force-closure grasp rejection: a per-side nub contact (arm joint path -> nub-end frame), whose
   // INTERNAL (squeeze) wrench is subtracted from the measurement so it is not attributed to forearm inertia.
   private final boolean graspRejectionEnabled;
   private final GraspInternalWrenchCalculator graspCalculator;
   private final SideDependentList<JointBasics[]> armJoints = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> nubFrames = new SideDependentList<>();
   private final SideDependentList<GeometricJacobian> compactNubJacobians = new SideDependentList<>();
   private final SideDependentList<DMatrixRMaj> fullNubJacobians = new SideDependentList<>();
   private final SideDependentList<DMatrixRMaj> handContactWrenches = new SideDependentList<>();
   private final FramePoint3D leftNubPosition = new FramePoint3D();
   private final FramePoint3D rightNubPosition = new FramePoint3D();
   private final double knownGraspSqueeze;
   private final YoDouble graspSqueezeApplied;
   private final YoDouble graspNubDistance;

   private final DMatrixRMaj wholeSystemTorques;
   /** SIM-ONLY constant measurement bias added to {@link #wholeSystemTorques} each tick, or null. See constructor. */
   private final DMatrixRMaj measurementBiasInjection;

   private final DMatrixRMaj jointVelocitiesContainer;
   private final FilteredFiniteDifferenceYoVariable[] jointAccelerations;

   private final Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption>[] basisSets;
   private final RigidBodyBasics[] regressorModelBodiesToProcess;
   private final DMatrixRMaj[] regressorBlocks;
   private final DMatrixRMaj regressor;

   private OnlineInertialEstimator filter;
   private final YoMatrix estimate;
   private final AlphaFilteredYoMatrix filteredEstimate;
   /**
    * Set by {@link #resetEstimator()} to clear the estimate filter's memory. {@link AlphaFilteredYoMatrix} has no
    * reset, but it asks us for its alpha every tick, so handing it alpha = 0 for one tick makes its own update
    * ({@code filtered = alpha * previous + (1 - alpha) * current}) overwrite the remembered (pre-reset, possibly
    * diverged) output with the current value. Without this the filter would drag the diverged estimate back into the
    * robot models on the tick after the reset. Cleared again as soon as that tick's filter call has happened.
    */
   private boolean clearEstimateFilterMemory = false;

   private final YoMatrix residual;

   private final InertialBiasWindowFilter biasWindowFilter;
   private final YoBoolean excludeBias;
   private final YoBoolean eraseBias;

   /**
    * Operator request to TARE THE PROCESS: re-seed the parameters to nominal, then -- with the parameters held
    * there -- average the measurement residual over one window and fold it into the bias. See {@link #tareProcess()}.
    */
   private final YoBoolean tareProcess;
   /**
    * True while a {@link #tareProcess()} run is in progress; self-clears when the bias window fills. This is also
    * what gates the collection of residuals into the bias window: a tare is the ONLY way to move the bias, because
    * the bias is only meaningful when it is measured at a known (nominal, held) parameter value.
    */
   private final YoBoolean tareProcessInProgress;
   /**
    * Holds the estimator's parameters where they are: the residual is still computed, but the state and covariance
    * are not updated. Set by {@link #tareProcess()} for the duration of the tare; also exposed on its own so the
    * residual can be watched at nominal without the filter chasing it.
    */
   private final YoBoolean holdParametersAtNominal;

   private final DoubleProvider accelerationCalculationAlpha;  // useful to have a master setting, we want to filter all DoFs equally

   private final YoDouble normalizedInnovation;
   private final YoDouble normalizedInnovationThreshold;

   private final YoBoolean passThroughEstimatesToController;
   private final YoDouble passThroughGain;
   private final YoBoolean resetEstimator;

   private final YoBoolean areParametersPhysicallyConsistent;

   /**
    * Whether the estimator has ever written its estimate into the controller's robot model, i.e. whether pass-through
    * has been on at any point. Used by {@link #resetEstimator()} to know whether the controller model needs restoring
    * to nominal -- if the estimator never touched it, it must be left well alone.
    */
   private boolean hasWrittenToControllerRobotModel = false;

   private enum RobotModelTask
   {
      // the robot model used in the controller that filtered inertial parameters are sent to
      CONTROLLER,
      // the model used to estimate inertial parameters and for visualization
      ESTIMATE,
      // a model where the parameters are being estimated are zeroed, so that an inverse dynamics call results in torques from only known parameters
      INVERSE_DYNAMICS,
      // a model to use for regressor calculations, which requires all bodies to be iteratively zeroed inertially
      REGRESSOR
   }

   @SuppressWarnings("unchecked")  // generic YoEnum[] for the per-body adaptation state
   public InertialParameterManager(HighLevelHumanoidControllerToolbox toolbox, InertialEstimationParameters inertialEstimationParameters, DoubleProvider dt, YoRegistry parentRegistry)
   {
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      parentRegistry.addChild(registry);
      basisSets = inertialEstimationParameters.getBasisSets();

      enableFilter = new YoBoolean("enableFilter", registry);
      // Default from the parameters (false unless a robot opts in, e.g. Alex hardware runs the estimator open-loop
      // from startup). Sim entries additionally poke this true post-construction, which is a no-op when already true.
      enableFilter.set(inertialEstimationParameters.getEnableFilterOnStartup());

      modelHandler = new MultipleHumanoidModelHandler<>(RobotModelTask.class);
      FullHumanoidRobotModel controllerRobotModel = toolbox.getFullRobotModel();
      modelHandler.putRobotModel(RobotModelTask.CONTROLLER, controllerRobotModel);

      for (RobotModelTask task : new RobotModelTask[] {RobotModelTask.ESTIMATE, RobotModelTask.INVERSE_DYNAMICS, RobotModelTask.REGRESSOR})
      {
         RigidBodyBasics clonedElevator = MultiBodySystemFactories.cloneMultiBodySystem(controllerRobotModel.getElevator(),
                                                                                       controllerRobotModel.getModelStationaryFrame(),
                                                                                       "_" + task.name());
         FullHumanoidRobotModel clonedRobotModel = new FullHumanoidRobotModelWrapper(clonedElevator, false);
         modelHandler.putRobotModel(task, clonedRobotModel);
      }

      // Attachment-point prior: override the ESTIMATE-model reference CoM of selected bodies (e.g. shift a
      // loaded forearm's CoM to a known cuff/hand attachment point) BEFORE the baseline and filter capture the
      // nominal, so the estimator deposits estimated payload mass there rather than at the nominal link CoM.
      for (RigidBodyBasics estimateBody : modelHandler.getBodyArray(RobotModelTask.ESTIMATE))
      {
         if (estimateBody == null || estimateBody.getInertia() == null)
            continue;
         us.ihmc.euclid.tuple3D.Vector3D referenceCoMOverride = inertialEstimationParameters.getReferenceCenterOfMassOffsetOverride(estimateBody.getName());
         if (referenceCoMOverride != null)
         {
            // Move the reference CoM to the attachment point AND rebuild a physically-consistent (PD) moment of
            // inertia there -- moving the CoM alone would make the pseudo-inertia non-positive-definite and break
            // the log-Cholesky reference. Mass is unchanged.
            double referenceMass = estimateBody.getInertia().getMass();
            estimateBody.getInertia().getCenterOfMassOffset().set(referenceCoMOverride);
            us.ihmc.euclid.matrix.Matrix3D referenceMomentOfInertia = new us.ihmc.euclid.matrix.Matrix3D();
            InertialEstimationParameters.packAttachmentReferenceMomentOfInertia(referenceMass, referenceCoMOverride, referenceMomentOfInertia);
            estimateBody.getInertia().getMomentOfInertia().set(referenceMomentOfInertia);
            LogTools.info("InertialParameterManager: attachment-point reference for '" + estimateBody.getName() + "' -> CoM " + referenceCoMOverride);
         }
      }

      covarianceHelper = new HumanoidModelCovarianceHelper(controllerRobotModel, inertialEstimationParameters, registry);
      baselineCalculator = new InertialBaselineCalculator(modelHandler.getRobotModel(RobotModelTask.ESTIMATE), inertialEstimationParameters, 1.0, registry);

      yoInertiaEllipsoids = InertiaVisualizationTools.createYoInertiaEllipsoids(controllerRobotModel.getRootBody(), registry);
      ellipsoidGraphicGroup = InertiaVisualizationTools.getInertiaEllipsoidGroup(yoInertiaEllipsoids);

      inverseDynamicsCalculator = new InverseDynamicsCalculator(modelHandler.getRobotModel(RobotModelTask.INVERSE_DYNAMICS).getElevator());
      inverseDynamicsCalculator.setGravitationalAcceleration(-toolbox.getGravityZ());
      zeroInverseDynamicsParameters(basisSets);
      regressorCalculator = new JointTorqueRegressorCalculator(modelHandler.getRobotModel(RobotModelTask.REGRESSOR).getElevator());
      regressorCalculator.setGravitationalAcceleration(-toolbox.getGravityZ());

      int nDoFs = MultiBodySystemTools.computeDegreesOfFreedom(controllerRobotModel.getRootJoint().subtreeArray());
      int nParameters = inertialEstimationParameters.getNumberOfParameters();
      int nNonEmptyBasisSets = inertialEstimationParameters.getNumberOfNonEmptyBasisSets();

      // To be as efficient as possible, we construct a compact regressor that only contains the bodies being estimated
      regressorBlocks = new DMatrixRMaj[nNonEmptyBasisSets];
      for (int i = 0; i < nNonEmptyBasisSets; i++)
         regressorBlocks[i] = new DMatrixRMaj(nDoFs, RigidBodyInertialParameters.PARAMETERS_PER_RIGID_BODY);
      regressor = new DMatrixRMaj(nDoFs, nParameters);
      regressorModelBodiesToProcess = new RigidBodyBasics[nNonEmptyBasisSets];
      int regressorIndex = 0;
      for (int i = 0; i < basisSets.length; ++i)
      {
         if (basisSets[i].isEmpty())
            continue;

         regressorModelBodiesToProcess[regressorIndex] = modelHandler.getBodyArray(RobotModelTask.REGRESSOR)[i];
         regressorIndex++;
      }

      this.footSwitches = toolbox.getFootSwitches();
      contactWrenches = new SideDependentList<>();
      jointIndexHandler = new JointIndexHandler(controllerRobotModel.getRootJoint().subtreeArray());
      legJoints = new SideDependentList<>();
      compactContactJacobians = new SideDependentList<>();
      fullContactJacobians = new SideDependentList<>();
      // NOTE: for the leg joints and compact jacobians, we use the controller robot model because it has the full model information, including all joint names
      for (RobotSide side : RobotSide.values)
      {
         contactWrenches.put(side, new DMatrixRMaj(Wrench.SIZE, 1));
         legJoints.put(side, MultiBodySystemTools.createJointPath(controllerRobotModel.getElevator(), controllerRobotModel.getFoot(side)));
         compactContactJacobians.put(side, new GeometricJacobian(legJoints.get(side), footSwitches.get(side).getMeasurementFrame()));
         fullContactJacobians.put(side, new DMatrixRMaj(Wrench.SIZE, nDoFs));
      }

      // Two-nub force-closure grasp rejection: build a nub contact (arm joint path elevator->hand, Jacobian
      // expressed at a fixed nub-end frame off the hand link) per side. Requires a non-null nub offset for BOTH
      // sides (force closure needs both nubs). The nub frame coincides with where the sim wrencher applies the
      // grasp and where the nub-tracking monitor reports, so the estimator's contact frame matches the load point.
      knownGraspSqueeze = inertialEstimationParameters.getKnownGraspSqueeze();
      graspSqueezeApplied = new YoDouble("graspSqueezeApplied", registry);
      graspNubDistance = new YoDouble("graspNubDistance", registry);
      Vector3D leftNubOffset = inertialEstimationParameters.isGraspRejectionEnabled() ? inertialEstimationParameters.getNubContactOffsetInHandFrame(RobotSide.LEFT) : null;
      Vector3D rightNubOffset = inertialEstimationParameters.isGraspRejectionEnabled() ? inertialEstimationParameters.getNubContactOffsetInHandFrame(RobotSide.RIGHT) : null;
      graspRejectionEnabled = leftNubOffset != null && rightNubOffset != null;
      if (graspRejectionEnabled)
      {
         SideDependentList<Vector3D> nubOffsets = new SideDependentList<>(leftNubOffset, rightNubOffset);
         for (RobotSide side : RobotSide.values)
         {
            RigidBodyBasics hand = controllerRobotModel.getHand(side);
            RigidBodyTransform handToNub = new RigidBodyTransform();
            handToNub.getTranslation().set(nubOffsets.get(side));
            ReferenceFrame nubFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("nubContactFrame" + side.name(),
                                                                                                       hand.getParentJoint().getFrameAfterJoint(),
                                                                                                       handToNub);
            nubFrames.put(side, nubFrame);
            armJoints.put(side, MultiBodySystemTools.createJointPath(controllerRobotModel.getElevator(), hand));
            compactNubJacobians.put(side, new GeometricJacobian(armJoints.get(side), nubFrame));
            fullNubJacobians.put(side, new DMatrixRMaj(Wrench.SIZE, nDoFs));
            handContactWrenches.put(side, new DMatrixRMaj(Wrench.SIZE, 1));
         }
         graspCalculator = new GraspInternalWrenchCalculator(nubFrames, ReferenceFrame.getWorldFrame());
         LogTools.info("InertialParameterManager: two-nub grasp rejection ON (method=" + inertialEstimationParameters.getGraspRejectionMethod()
                       + ", knownSqueeze=" + knownGraspSqueeze + " N, hands "
                       + controllerRobotModel.getHand(RobotSide.LEFT).getName() + "/" + controllerRobotModel.getHand(RobotSide.RIGHT).getName() + ").");
      }
      else
      {
         graspCalculator = null;
      }

      wholeSystemTorques = new DMatrixRMaj(nDoFs, 1);

      // SIM-ONLY: optional constant bias added to the torque measurement each tick, to emulate the persistent
      // hardware model/sensor bias the estimator cannot null (see getMeasurementBiasInjection). Null -> no bias.
      double[] biasInjection = inertialEstimationParameters.getMeasurementBiasInjection();
      if (biasInjection != null)
      {
         if (biasInjection.length != nDoFs)
            throw new RuntimeException("getMeasurementBiasInjection() length " + biasInjection.length + " != nDoFs " + nDoFs);
         measurementBiasInjection = new DMatrixRMaj(nDoFs, 1, true, biasInjection);
         LogTools.warn("InertialParameterManager: SIM measurement-bias injection ACTIVE (||bias|| = "
                       + String.format("%.2f", org.ejml.dense.row.NormOps_DDRM.normF(measurementBiasInjection)) + " Nm).");
      }
      else
      {
         measurementBiasInjection = null;
      }

      String[] measurementNames = inertialEstimationParameters.getMeasurementNames();
      residual = new YoMatrix("residual_", nDoFs, 1, measurementNames, registry);

      accelerationCalculationAlpha = () -> AlphaFilterTools.computeAlphaGivenBreakFrequencyProperly(inertialEstimationParameters.getBreakFrequencyForAccelerationCalculation(), dt.getValue());
      jointVelocitiesContainer = new DMatrixRMaj(nDoFs, 1);
      jointAccelerations = new FilteredFiniteDifferenceYoVariable[nDoFs];
      for (int i = 0; i < measurementNames.length; i++)
         jointAccelerations[i] = new FilteredFiniteDifferenceYoVariable("jointAcceleration_" + measurementNames[i], "", accelerationCalculationAlpha, dt, registry);

      String[] estimateNames = inertialEstimationParameters.getEstimateNames();
      estimate = new YoMatrix("", nParameters, 1, estimateNames, null, registry);
      DoubleProvider defaultEstimateFilteringAlpha = () -> clearEstimateFilterMemory ? 0.0
            : AlphaFilterTools.computeAlphaGivenBreakFrequencyProperly(inertialEstimationParameters.getBreakFrequencyForEstimateFiltering(), dt.getValue());
      filteredEstimate = new AlphaFilteredYoMatrix("filtered_", defaultEstimateFilteringAlpha, nParameters, 1, estimateNames, null, registry);

      int windowSizeInTicks = (int) (inertialEstimationParameters.getBiasCompensationWindowSizeInSeconds() / dt.getValue());
      biasWindowFilter = new InertialBiasWindowFilter(nDoFs, windowSizeInTicks, measurementNames, registry);
      excludeBias = new YoBoolean("excludeBias", registry);
      excludeBias.set(false);
      eraseBias = new YoBoolean("eraseBias", registry);
      eraseBias.set(false);

      tareProcess = new YoBoolean("tareProcess", registry);
      tareProcess.set(false);
      tareProcessInProgress = new YoBoolean("tareProcessInProgress", registry);
      tareProcessInProgress.set(false);
      holdParametersAtNominal = new YoBoolean("holdParametersAtNominal", registry);
      holdParametersAtNominal.set(false);

      normalizedInnovation = new YoDouble("normalizedInnovation", registry);
      normalizedInnovation.set(0.0);
      normalizedInnovationThreshold = new YoDouble("normalizedInnovationThreshold", registry);
      normalizedInnovationThreshold.set(inertialEstimationParameters.getNormalizedInnovationThreshold());

      // Construct the type of filter used based on enum value
      switch (inertialEstimationParameters.getTypeOfEstimatorToUse())
      {
         case KF -> filter = new InertialKalmanFilter(modelHandler.getRobotModel(RobotModelTask.ESTIMATE), inertialEstimationParameters, dt, registry);
         case PHYSICALLY_CONSISTENT_EKF -> filter = new InertialPhysicallyConsistentKalmanFilter(modelHandler.getRobotModel(RobotModelTask.ESTIMATE),
                                                                                                 inertialEstimationParameters, dt, registry);
      }

      // Per-limb operator-intent adaptation gate (see field block). covarianceHelper and filter now both exist and
      // share the estimated-body order, so a single block index b maps to both the Q scale and the prior scale.
      intentGatedAdaptation = new YoBoolean("intentGatedAdaptation", registry);
      intentGatedAdaptation.set(false);
      adaptationSignalAlive = new YoBoolean("adaptationSignalAlive", registry);
      adaptationSignalAlive.set(true);
      for (RobotSide side : RobotSide.values)
      {
         YoBoolean grasping = new YoBoolean(side.getCamelCaseNameForStartOfExpression() + "ArmGrasping", registry);
         grasping.set(false);
         armGrasping.put(side, grasping);
      }
      adaptationQScaleActive = new YoDouble("adaptationQScaleActive", registry);
      adaptationQScaleActive.set(1.0);
      adaptationQScaleIdle = new YoDouble("adaptationQScaleIdle", registry);
      adaptationQScaleIdle.set(1.0e-3);
      adaptationPriorScaleActive = new YoDouble("adaptationPriorScaleActive", registry);
      adaptationPriorScaleActive.set(1.0);
      adaptationPriorScaleIdleNominal = new YoDouble("adaptationPriorScaleIdleNominal", registry);
      adaptationPriorScaleIdleNominal.set(1.0e-6);   // tight -> pins the idle limb to nominal
      adaptationPriorScaleIdleHold = new YoDouble("adaptationPriorScaleIdleHold", registry);
      adaptationPriorScaleIdleHold.set(1.0e6);       // no pull -> freeze in place (with low Q)
      int nEstimatedBodies = covarianceHelper.getNumberOfEstimatedBodies();
      bodyAdaptationState = new YoEnum[nEstimatedBodies];
      bodyAdaptationSide = new RobotSide[nEstimatedBodies];
      bodyWasGrasping = new boolean[nEstimatedBodies];
      for (int b = 0; b < nEstimatedBodies; b++)
      {
         String bodyName = covarianceHelper.getEstimatedBodyName(b);
         bodyAdaptationState[b] = new YoEnum<>(bodyName + "_adaptationState", registry, AdaptationState.class, false);
         bodyAdaptationState[b].set(AdaptationState.ACTIVE);
         bodyAdaptationSide[b] = bodyName.contains("LEFT") ? RobotSide.LEFT : (bodyName.contains("RIGHT") ? RobotSide.RIGHT : null);
      }

      // Phase 2 transport: subscribe to the VR grasp-intent stream when requested (default off -> no ROS2 in
      // sim/tests). When enabled, the gate is switched on too, so a grasping hand drives its forearm's adaptation
      // and idle hands pin to nominal out of the box.
      dtSeconds = dt.getValue();
      secondsSinceGraspMessage = Double.POSITIVE_INFINITY;
      if (inertialEstimationParameters.subscribeToGraspIntent())
      {
         graspROS2Node = new AsyncROS2Node("inertial_adaptation_grasp");
         latestGraspMessage = new InertialAdaptationCommandMessage();
         graspSubscriber = graspROS2Node.createSubscription(INERTIAL_ADAPTATION_TOPIC, message -> { });
         intentGatedAdaptation.set(true);
         LogTools.info("InertialParameterManager: grasp-intent gate subscribed on " + INERTIAL_ADAPTATION_TOPIC.getName());
      }
      else
      {
         graspROS2Node = null;
         latestGraspMessage = null;
         graspSubscriber = null;
      }

      passThroughEstimatesToController = new YoBoolean("passThroughEstimatesToController", registry);
      passThroughEstimatesToController.set(false);

      // Trust gain in [0, 1] scaling the estimator delta blended into the controller model when pass-through is
      // on: 0 = nominal (no estimate applied even with pass-through on), 1 = full estimate. Ramp up at runtime.
      passThroughGain = new YoDouble("inertialEstimatorPassthroughGain", registry);
      passThroughGain.set(0.0);

      // Operator escape hatch if the estimator diverges: throws away everything the filter has learned and starts
      // again from the nominal model. Polled and self-cleared in update(). See resetEstimator().
      resetEstimator = new YoBoolean("resetEstimator", registry);
      resetEstimator.set(false);

      areParametersPhysicallyConsistent = new YoBoolean("areParametersPhysicallyConsistent", registry);
      areParametersPhysicallyConsistent.set(false);
   }

   public void update()
   {
      if (eraseBias.getValue())
      {
         biasWindowFilter.reset();
         eraseBias.set(false);
      }

      // Handle a requested process tare. Only the ARMING happens here; the residuals are collected at the bottom of
      // update(), after the filter has actually produced this tick's residual.
      if (tareProcess.getValue())
      {
         startTareProcess();
         tareProcess.set(false);
      }

      // Handle a requested reset of the estimator. Outside the enableFilter check, so that a diverged estimator can
      // also be cleaned up with the filter switched off. We skip the filter for this tick and pick it up again on the
      // next one, so that the nominal re-seed is actually what the estimate holds for a tick -- running the update
      // immediately would overwrite it before anyone (a log, a plot, the controller) could see it.
      if (resetEstimator.getValue())
      {
         resetEstimator();
         resetEstimator.set(false);
         return;
      }

      // Main loop of inertial estimator
      if (enableFilter.getValue())
      {
         // Hold the parameters while a process tare is running: the filter still computes its residual, but does not
         // chase it. Without this the filter would absorb the very mismatch we are trying to measure.
         filter.setParametersHeld(holdParametersAtNominal.getValue());

         // Per-limb intent gate: push this tick's Q/prior scales before the covariances are read below.
         updateAdaptationGate();

         updateFilterCovariances();

         updateContactJacobians();
         updateContactWrenches();
         updateWholeSystemTorques();
         if (measurementBiasInjection != null)  // SIM-ONLY: emulate persistent hardware measurement bias
            CommonOps_DDRM.addEquals(wholeSystemTorques, measurementBiasInjection);

         updateRegressorAndInverseDynamicsModels();
         inverseDynamicsCalculator.compute();
         regressorCalculator.compute(regressorModelBodiesToProcess);

         for (int i = 0; i < regressorModelBodiesToProcess.length; i++)
            regressorBlocks[i].set(regressorCalculator.getJointTorqueRegressorMatrixBlock(regressorModelBodiesToProcess[i]));
         packRegressorFromBlocks(regressorBlocks, basisSets, regressor);

         filter.setTorqueFromNominal(inverseDynamicsCalculator.getJointTauMatrix());
         filter.setRegressor(regressor);
         filter.setContactJacobians(fullContactJacobians);
         filter.setContactWrenches(contactWrenches);

         if (graspRejectionEnabled)
         {
            updateNubContacts();
            filter.setHandContactJacobians(fullNubJacobians);
            filter.setHandContactWrenches(handContactWrenches);
         }

         if (excludeBias.getValue())
            filter.setTorqueFromBias(biasWindowFilter.getZero());
         else
            filter.setTorqueFromBias(biasWindowFilter.getBias());

         filter.setNormalizedInnovationThreshold(normalizedInnovationThreshold.getValue());

         estimate.set(filter.calculateEstimate(wholeSystemTorques));
         filteredEstimate.setAndSolve(estimate);
         // The filter has now consumed the one zeroed alpha, so its memory holds the post-reset estimate rather than a
         // pre-reset (possibly diverged) one. Back to the tuned alpha from here on.
         clearEstimateFilterMemory = false;
         residual.set(filter.getMeasurementResidual());

         normalizedInnovation.set(filter.getNormalizedInnovation());

         // Collect residuals for the bias HERE, not at the top of update(): this tick's residual has just been
         // produced, and -- during a process tare -- it was produced at a known, held parameter value, which is the
         // whole point. Reading it at the top would sample the PREVIOUS tick's residual instead.
         if (tareProcessInProgress.getValue() && biasWindowFilter.update(residual))
            finishTareProcess();  // window full: the bias has just been accumulated

         // Pack smoothed estimate back into estimate robot bodies
         RegressorTools.packRigidBodies(basisSets, filteredEstimate, modelHandler.getBodyArray(RobotModelTask.ESTIMATE));

         // Check physical consistency
         checkPhysicalConsistency();

         // Pass through estimates to controller
         if (passThroughEstimatesToController.getValue())
            updateControllerRobotModel();

         updateVisuals();
      }
   }

   /**
    * Throws away everything the estimator has learned and starts again from the nominal model, as a recovery action
    * if the filter diverges. The filter's state is re-seeded to the nominal inertial parameters and its covariance to
    * the value it was constructed with, the estimate and residual outputs and their smoothing filters are cleared,
    * and both the estimate and controller robot models are put back to their nominal inertias -- so a diverged
    * estimate is removed from the controller immediately rather than slewing back down through the rate limiters.
    * <p>
    * Deliberately NOT reset, because they are operator intent rather than filter state, and each has its own control:
    * the measurement bias ({@code eraseBias}), the process and measurement covariance tuning, whether the filter is
    * enabled ({@code enableFilter}), and the pass-through gain.
    * </p>
    */
   private void resetEstimator()
   {
      filter.reset();

      estimate.set(filter.getState());
      // Show the nominal re-seed immediately, and have the filter throw away its remembered output next tick.
      filteredEstimate.set(estimate);
      clearEstimateFilterMemory = true;
      residual.zero();
      normalizedInnovation.set(0.0);

      baselineCalculator.resetParameterDeltas();
      baselineCalculator.restoreNominalParameters(modelHandler.getBodyArray(RobotModelTask.ESTIMATE));

      // Only put the controller's model back if the estimator is the one that changed it -- with pass-through off it
      // is not ours to touch.
      if (hasWrittenToControllerRobotModel)
         baselineCalculator.restoreNominalParameters(modelHandler.getBodyArray(RobotModelTask.CONTROLLER));

      // Re-arm the physical consistency warnings, the estimate is nominal and therefore consistent again.
      physicalConsistencyWarningCount = 0;
      areParametersPhysicallyConsistent.set(true);

      // Abort any in-flight process tare. Note this is safe to run during startTareProcess()'s own call to this
      // method, which re-arms these flags immediately afterwards -- but if an operator resets MID-tare, the filter
      // must not be left held forever with a half-filled bias window.
      if (tareProcessInProgress.getValue())
      {
         tareProcessInProgress.set(false);
         holdParametersAtNominal.set(false);
         filter.setParametersHeld(false);
         biasWindowFilter.rearm();
         LogTools.warn("InertialParameterManager: estimator reset during a process tare -- tare ABORTED, bias unchanged.");
      }

      updateVisuals();

      LogTools.info("InertialParameterManager: estimator reset -- parameters re-seeded to nominal, covariance re-initialized.");
   }

   /**
    * Arms a process tare: the estimator is re-seeded to nominal, its parameters are HELD there, and the measurement
    * residual is collected over one bias window.
    * <p>
    * This is the "zero the scale before you put the load on it" operation. On hardware the residual is not zero even
    * with the correct (nominal) parameters, because the rest of the model is not ideal -- friction, torque-sensor
    * offsets, and inertial error in the OTHER links all show up in it. Left alone, the estimator will happily explain
    * that mismatch by moving the one body it is allowed to move, which is how the parameters walk off into the
    * degenerate directions. Taring instead attributes it to where it belongs: an additive measurement bias.
    * </p>
    * <p>
    * Afterwards, theta = 0 is a genuine fixed point -- parameters at nominal AND residual at zero -- so anything the
    * filter subsequently moves is the added payload, which is what the estimate is supposed to mean.
    * </p>
    * <p>
    * Run this UNLOADED, with the robot in (or near) the pose it will operate in. The bias is a single constant, but
    * the mismatch it stands in for is configuration-dependent, so a tare taken in one pose is only good near that
    * pose.
    * </p>
    */
   private void startTareProcess()
   {
      if (!enableFilter.getValue())
      {
         LogTools.warn("InertialParameterManager: tareProcess requested but the filter is disabled -- no residual is "
                       + "computed with the filter off, so there is nothing to tare. Ignoring.");
         return;
      }

      // Parameters back to nominal: the bias must be measured at a KNOWN parameter value, and nominal is the one the
      // estimate is defined as a deviation from.
      resetEstimator();

      // Keep the existing bias and collect the mismatch that REMAINS on top of it; the window filter accumulates.
      // The rearm() is what makes a re-tare safe: without it the window is still full from last time, and the very
      // next update() would re-average those stale samples and add them a second time.
      biasWindowFilter.rearm();
      holdParametersAtNominal.set(true);
      tareProcessInProgress.set(true);

      LogTools.info("InertialParameterManager: process tare STARTED -- parameters re-seeded to nominal and held; "
                    + "collecting the residual over the bias window.");
   }

   /** Completes a process tare: the bias has just been accumulated, so release the parameters. */
   private void finishTareProcess()
   {
      holdParametersAtNominal.set(false);
      filter.setParametersHeld(false);
      tareProcessInProgress.set(false);

      LogTools.info("InertialParameterManager: process tare COMPLETE -- bias identified at nominal parameters, filter "
                    + "released. Deviations from here on are the added payload.");
   }

   /**
    * Operator-intent input to the per-limb adaptation gate: whether the given hand is currently grasping. Set from
    * the VR grasp trigger (Phase 2 transport) or by hand from SCS2. Only has effect while {@code intentGatedAdaptation}
    * is enabled. See {@link #updateAdaptationGate()}.
    */
   public void setArmGrasping(RobotSide side, boolean grasping)
   {
      armGrasping.get(side).set(grasping);
   }

   /**
    * Liveness of the grasp-intent signal (a heartbeat from the operator side). When set false, a limb that was
    * grasping switches to IDLE_HOLD (freezes) rather than relaxing to nominal, so a still-held load's gravity
    * compensation does not collapse if the operator link drops. See {@link #updateAdaptationGate()}.
    */
   public void setAdaptationSignalAlive(boolean alive)
   {
      adaptationSignalAlive.set(alive);
   }

   private void updateFilterCovariances()
   {
      filter.setProcessCovariance(covarianceHelper.getProcessCovariance());
      filter.setMeasurementCovariance(covarianceHelper.getMeasurementCovariance());
   }

   /**
    * Per-limb operator-intent adaptation gate. Each estimated limb is placed in one of three regimes and the
    * matching process-noise (Q) and prior scales are pushed to the covariance helper and the filter. Disabled
    * ({@code intentGatedAdaptation} false) -> every limb ACTIVE with scales 1.0, i.e. legacy behavior. See
    * {@link AdaptationState}. Runs each tick just before the covariances are read.
    * <ul>
    * <li>ACTIVE (grasping): base Q, base (loose) prior -> the limb tracks its payload.</li>
    * <li>IDLE_NOMINAL (not grasping, signal alive): low Q, tight prior -> pinned to nominal (prevents the leech
    *     and returns to nominal on release).</li>
    * <li>IDLE_HOLD (was grasping, signal lost): low Q, no prior pull -> frozen at the last estimate, so gravity
    *     compensation does not collapse under a still-held load.</li>
    * </ul>
    */
   private void updateAdaptationGate()
   {
      // Phase 2: pull the latest per-hand grasp intent from the VR stream, and track liveness. A fresh message sets
      // the per-hand grasping booleans and marks the signal alive; if no message arrives within the timeout the
      // signal is marked lost, so a still-grasping limb switches to IDLE_HOLD (freeze) rather than relaxing.
      if (graspSubscriber != null)
      {
         if (graspSubscriber.readLatest(latestGraspMessage) > 0)
         {
            armGrasping.get(RobotSide.LEFT).set(latestGraspMessage.getLeftHandGrasping());
            armGrasping.get(RobotSide.RIGHT).set(latestGraspMessage.getRightHandGrasping());
            secondsSinceGraspMessage = 0.0;
         }
         else
         {
            secondsSinceGraspMessage += dtSeconds;
         }
         adaptationSignalAlive.set(secondsSinceGraspMessage < GRASP_SIGNAL_TIMEOUT_SECONDS);
      }

      boolean gated = intentGatedAdaptation.getValue();
      boolean alive = adaptationSignalAlive.getValue();
      for (int b = 0; b < bodyAdaptationState.length; b++)
      {
         AdaptationState state;
         if (!gated)
         {
            state = AdaptationState.ACTIVE;
         }
         else if (!alive)
         {
            // Signal lost: the grasp state can no longer be trusted, so freeze if we were mid-grasp (don't let a
            // held load's estimate collapse), otherwise relaxing to nominal is safe. Checked BEFORE grasping so a
            // stale-true grasp flag cannot keep an arm ACTIVE on dead data.
            state = bodyWasGrasping[b] ? AdaptationState.IDLE_HOLD : AdaptationState.IDLE_NOMINAL;
         }
         else
         {
            RobotSide side = bodyAdaptationSide[b];
            boolean grasping = side != null && armGrasping.get(side).getValue();
            if (grasping)
            {
               state = AdaptationState.ACTIVE;
               bodyWasGrasping[b] = true;
            }
            else
            {
               state = AdaptationState.IDLE_NOMINAL;
               bodyWasGrasping[b] = false;
            }
         }
         bodyAdaptationState[b].set(state);

         double qScale;
         double priorScale;
         switch (state)
         {
            case IDLE_NOMINAL ->
            {
               qScale = adaptationQScaleIdle.getValue();
               priorScale = adaptationPriorScaleIdleNominal.getValue();
            }
            case IDLE_HOLD ->
            {
               qScale = adaptationQScaleIdle.getValue();
               priorScale = adaptationPriorScaleIdleHold.getValue();
            }
            default ->
            {
               qScale = adaptationQScaleActive.getValue();
               priorScale = adaptationPriorScaleActive.getValue();
            }
         }
         covarianceHelper.setProcessScaleForBody(b, qScale);
         filter.setPriorScaleForBody(b, priorScale);
      }
   }

   private void updateWholeSystemTorques()
   {
      modelHandler.extractJointsState(RobotModelTask.CONTROLLER, JointStateType.EFFORT, wholeSystemTorques);
   }

   private void updateContactJacobians()
   {
      for (RobotSide side : RobotSide.values)
      {
         compactContactJacobians.get(side).compute();
         jointIndexHandler.compactBlockToFullBlock(legJoints.get(side), compactContactJacobians.get(side).getJacobianMatrix(), fullContactJacobians.get(side));
      }
   }

   private void updateContactWrenches()
   {
      for (RobotSide side : RobotSide.values)
         footSwitches.get(side).getMeasuredWrench().get(contactWrenches.get(side));
   }

   /**
    * Update the nub contact Jacobians and the internal (squeeze) wrench to subtract. Stage A (KNOWN): the squeeze
    * magnitude is prescribed and the grasp line is taken from the two nub-frame origins. Stage B (JACOBIAN) is
    * added later. The packed {@code handContactWrenches} are pure point-contact forces in each nub frame.
    */
   private void updateNubContacts()
   {
      for (RobotSide side : RobotSide.values)
      {
         compactNubJacobians.get(side).compute();
         jointIndexHandler.compactBlockToFullBlock(armJoints.get(side), compactNubJacobians.get(side).getJacobianMatrix(), fullNubJacobians.get(side));
      }
      leftNubPosition.setToZero(nubFrames.get(RobotSide.LEFT));
      rightNubPosition.setToZero(nubFrames.get(RobotSide.RIGHT));
      graspCalculator.computeFromKnownSqueeze(leftNubPosition, rightNubPosition, knownGraspSqueeze, handContactWrenches);
      graspSqueezeApplied.set(graspCalculator.getSqueeze());
      graspNubDistance.set(graspCalculator.getDistance());
   }

   private void updateRegressorAndInverseDynamicsModels()
   {
      modelHandler.copyJointsState(RobotModelTask.CONTROLLER, RobotModelTask.REGRESSOR, JointStateType.CONFIGURATION);
      modelHandler.copyJointsState(RobotModelTask.CONTROLLER, RobotModelTask.REGRESSOR, JointStateType.VELOCITY);
      modelHandler.copyJointsState(RobotModelTask.CONTROLLER, RobotModelTask.INVERSE_DYNAMICS, JointStateType.CONFIGURATION);
      modelHandler.copyJointsState(RobotModelTask.CONTROLLER, RobotModelTask.INVERSE_DYNAMICS, JointStateType.VELOCITY);

      // Update joint accelerations by processing joint velocities
      modelHandler.extractJointsState(RobotModelTask.CONTROLLER, JointStateType.VELOCITY, jointVelocitiesContainer);
      for (int i = 0; i < jointAccelerations.length; ++i)
      {
         jointAccelerations[i].update(jointVelocitiesContainer.get(i));
         // Reuse the joint velocities container to store the joint accelerations
         jointVelocitiesContainer.set(i ,0, jointAccelerations[i].getValue());
      }
      modelHandler.insertJointsState(RobotModelTask.REGRESSOR, JointStateType.ACCELERATION, jointVelocitiesContainer);
      modelHandler.insertJointsState(RobotModelTask.INVERSE_DYNAMICS, JointStateType.ACCELERATION, jointVelocitiesContainer);

      modelHandler.getRobotModel(RobotModelTask.REGRESSOR).updateFrames();
      modelHandler.getRobotModel(RobotModelTask.INVERSE_DYNAMICS).updateFrames();
   }

   private void updateControllerRobotModel()
   {
      baselineCalculator.calculateRateLimitedParameterDeltas(modelHandler.getBodyArray(RobotModelTask.ESTIMATE));
      baselineCalculator.addRateLimitedParameterDeltas(modelHandler.getBodyArray(RobotModelTask.CONTROLLER), passThroughGain.getValue());
      hasWrittenToControllerRobotModel = true;
   }

   private void zeroInverseDynamicsParameters(Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption>[] basisSets)
   {
      RigidBodyBasics[] bodies = modelHandler.getBodyArray(RobotModelTask.INVERSE_DYNAMICS);
      for (int i = 0; i < basisSets.length; ++i)
      {
         if (basisSets[i].isEmpty())
            continue;

         for (JointTorqueRegressorCalculator.SpatialInertiaBasisOption option : basisSets[i])
            RigidBodyInertialParametersTools.zeroInertialParameter(bodies[i], option);
      }
   }

   private void packRegressorFromBlocks(DMatrixRMaj[] regressorBlocks, Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption>[] basisSets, DMatrixRMaj regressorToPack)
   {
      int regressorBlockIndex = 0;
      int regressorToPackColumnIndex = 0;
      for (Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption> basisSet : basisSets)
      {
         if (basisSet.isEmpty())
            continue;

         for (int i = 0; i < JointTorqueRegressorCalculator.SpatialInertiaBasisOption.values.length; ++i)
         {
            if (basisSet.contains(JointTorqueRegressorCalculator.SpatialInertiaBasisOption.values[i]))
            {
               MatrixMissingTools.setMatrixColumn(regressorToPack, regressorToPackColumnIndex, regressorBlocks[regressorBlockIndex], i);
               regressorToPackColumnIndex++;
            }
         }
         regressorBlockIndex++;
      }
   }

   /**
    * We can only check for physical consistency online, and not *full* physical consistency as the latter requires an expensive eigendecomposition to find the
    * principal moments of inertia.
    */
   private void checkPhysicalConsistency()
   {
      areParametersPhysicallyConsistent.set(true);
      for (RigidBodyBasics estimateModelBody : modelHandler.getBodyArray(RobotModelTask.ESTIMATE))
      {
         if (!RigidBodyInertialParametersTools.isPhysicallyConsistent(estimateModelBody.getInertia()))
         {
            if (physicalConsistencyWarningCount < MAX_PHYSICAL_CONSISTENCY_WARNINGS)
            {
               physicalConsistencyWarningCount++;
               LogTools.warn("Inertial parameter estimate for " + estimateModelBody.getName() + " is not physically consistent"
                             + (physicalConsistencyWarningCount == MAX_PHYSICAL_CONSISTENCY_WARNINGS ? " (suppressing further warnings)" : ""));
            }
            areParametersPhysicallyConsistent.set(false);
            break;
         }
      }
   }

   private void updateVisuals()
   {
      RigidBodyReadOnly[] controllerModelBodies = modelHandler.getBodyArray(RobotModelTask.CONTROLLER);
      SpatialInertiaReadOnly[] nominalSpatialInertias = baselineCalculator.getURDFSpatialInertias();
      for (int i = 0; i < controllerModelBodies.length; i++)
      {
         RigidBodyReadOnly controllerBody = controllerModelBodies[i];
         SpatialInertiaReadOnly nominalForBody = nominalSpatialInertias[i];

         double scale = EuclidCoreTools.clamp(controllerBody.getInertia().getMass() / nominalForBody.getMass() / 2.0, 0.0, 1.0);

         if (controllerBody.getInertia() != null)
            yoInertiaEllipsoids.get(i).update(scale);
      }
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(ellipsoidGraphicGroup);
      return group;
   }
}