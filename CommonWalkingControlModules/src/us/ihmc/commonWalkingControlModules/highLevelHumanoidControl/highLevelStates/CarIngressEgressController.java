package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.ChestOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.PelvisDesiredsHandler;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.endEffector.EndEffectorControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.endEffector.EndEffectorControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controlModules.head.HeadOrientationManager;
import us.ihmc.commonWalkingControlModules.controllers.LidarControllerInterface;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingManagers;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumControlModuleBridge.MomentumControlModuleType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredChestOrientationProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredFootPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredFootStateProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredPelvisLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredPelvisPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredThighLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.trajectories.ChangeableConfigurationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.OrientationInterpolationTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.PoseTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.SettableOrientationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.StraightLinePositionTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.WrapperForPositionAndOrientationTrajectoryGenerators;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.SpatialMotionVector;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.VariableChangedListener;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.util.GainCalculator;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.trajectory.ConstantDoubleProvider;
import com.yobotics.simulationconstructionset.util.trajectory.CubicPolynomialTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleProvider;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.ThirdOrderPolynomialTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.YoVariableDoubleProvider;

public class CarIngressEgressController extends AbstractHighLevelHumanoidControlPattern
{
   public final static HighLevelState controllerState = HighLevelState.INGRESS_EGRESS;
   private final static MomentumControlModuleType MOMENTUM_CONTROL_MODULE_TO_USE = MomentumControlModuleType.OPTIMIZATION;
   private final static double DELAY_TIME_BEFORE_TRUSTING_CONTACTS = 1.1;

   private final DesiredFootPoseProvider footPoseProvider;
   private final DesiredFootStateProvider footLoadBearingProvider;
   private DesiredThighLoadBearingProvider thighLoadBearingProvider;
   private DesiredPelvisLoadBearingProvider pelvisLoadBearingProvider;

   private final DesiredPelvisPoseProvider pelvisPoseProvider;
   private final RigidBodySpatialAccelerationControlModule pelvisController;
   private final ReferenceFrame pelvisPositionControlFrame;
   private final GeometricJacobian pelvisJacobian;
   private final TaskspaceConstraintData pelvisTaskspaceConstraintData = new TaskspaceConstraintData();
   private final ChangeableConfigurationProvider desiredPelvisConfigurationProvider, initialPelvisConfigurationProvider;
   private final StraightLinePositionTrajectoryGenerator pelvisPositionTrajectoryGenerator;
   private final OrientationInterpolationTrajectoryGenerator pelvisOrientationTrajectoryGenerator;
   private double pelvisTrajectoryStartTime = 0.0;

   private final DesiredChestOrientationProvider chestOrientationProvider;
   private final ReferenceFrame chestPositionControlFrame;
   private final SettableOrientationProvider finalDesiredChestOrientation, initialDesiredChestOrientation;
   private final OrientationInterpolationTrajectoryGenerator chestOrientationTrajectoryGenerator;
   private double chestTrajectoryStartTime = 0.0;

   private final BooleanYoVariable l_footDoHeelOff = new BooleanYoVariable("l_footDoHeelOff", registry);
   private final BooleanYoVariable r_footDoHeelOff = new BooleanYoVariable("r_footDoHeelOff", registry);
   private final SideDependentList<BooleanYoVariable> doHeelOff = new SideDependentList<BooleanYoVariable>(l_footDoHeelOff, r_footDoHeelOff);

   private final LinkedHashMap<ContactablePlaneBody, ChangeableConfigurationProvider> initialFootConfigurationProviders =
      new LinkedHashMap<ContactablePlaneBody, ChangeableConfigurationProvider>();
   private final LinkedHashMap<ContactablePlaneBody, ChangeableConfigurationProvider> desiredFootConfigurationProviders =
      new LinkedHashMap<ContactablePlaneBody, ChangeableConfigurationProvider>();
   private final LinkedHashMap<ContactablePlaneBody, StraightLinePositionTrajectoryGenerator> swingPositionTrajectoryGenerators =
      new LinkedHashMap<ContactablePlaneBody, StraightLinePositionTrajectoryGenerator>();
   private final LinkedHashMap<ContactablePlaneBody, OrientationInterpolationTrajectoryGenerator> swingOrientationTrajectoryGenerators =
      new LinkedHashMap<ContactablePlaneBody, OrientationInterpolationTrajectoryGenerator>();

   private final ConstantDoubleProvider footTrajectoryTimeProvider = new ConstantDoubleProvider(1.0);
   private final ConstantDoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(2.0);

   private final SideDependentList<ContactablePlaneBody> contactableThighs;
   private final ContactablePlaneBody contactablePelvis, contactablePelvisBack;
   
   private final DoubleYoVariable coefficientOfFrictionForBumAndThighs = new DoubleYoVariable("coefficientOfFrictionForBumAndThighs", registry);
   private final DoubleYoVariable coefficientOfFrictionForFeet = new DoubleYoVariable("coefficientOfFrictionForFeet", registry);
   
   private final RigidBody elevator;
   private final List<ContactablePlaneBody> bodiesInContact = new ArrayList<ContactablePlaneBody>();
   private final Map<ContactablePlaneBody, GeometricJacobian> contactJacobians = new LinkedHashMap<ContactablePlaneBody, GeometricJacobian>();

   private final DenseMatrix64F pelvisNullspaceMultipliers = new DenseMatrix64F(0, 1);

   private BooleanYoVariable requestedPelvisLoadBearing = new BooleanYoVariable("requestedPelvisLoadBearing", registry);
   private BooleanYoVariable requestedPelvisBackLoadBearing = new BooleanYoVariable("requestedPelvisBackLoadBearing", registry);
   private BooleanYoVariable requestedLeftFootLoadBearing = new BooleanYoVariable("requestedLeftFootLoadBearing", registry);
   private BooleanYoVariable requestedRightFootLoadBearing = new BooleanYoVariable("requestedRightFootLoadBearing", registry);
   private BooleanYoVariable requestedLeftThighLoadBearing = new BooleanYoVariable("requestedLeftThighLoadBearing", registry);
   private BooleanYoVariable requestedRightThighLoadBearing = new BooleanYoVariable("requestedRightThighLoadBearing", registry);
   private SideDependentList<BooleanYoVariable> requestedFootLoadBearing = new SideDependentList<BooleanYoVariable>(requestedLeftFootLoadBearing,
                                                                              requestedRightFootLoadBearing);
   private SideDependentList<BooleanYoVariable> requestedThighLoadBearing = new SideDependentList<BooleanYoVariable>(requestedLeftThighLoadBearing,
                                                                               requestedRightThighLoadBearing);
   private final VariousWalkingManagers variousWalkingManagers;
   
   // Parameters for smooth transitions between loading and unloading an end-effector.
   private final YoVariableDoubleProvider loadBearingTransitionTimeProvider;
   private final Map<ContactablePlaneBody, DoubleYoVariable> loadBearingTransitionStartTime = new LinkedHashMap<ContactablePlaneBody, DoubleYoVariable>();
   private final Map<ContactablePlaneBody, BooleanYoVariable> loadingTransitionHasBeenInitiated = new LinkedHashMap<ContactablePlaneBody, BooleanYoVariable>();
   private final Map<ContactablePlaneBody, BooleanYoVariable> unloadingTransitionHasBeenInitiated = new LinkedHashMap<ContactablePlaneBody, BooleanYoVariable>();
   private final Map<ContactablePlaneBody, Boolean> doLoadingTransition = new LinkedHashMap<ContactablePlaneBody, Boolean>();
   private final Map<ContactablePlaneBody, Boolean> doUnloadingTransition = new LinkedHashMap<ContactablePlaneBody, Boolean>();
   private final Map<ContactablePlaneBody, CubicPolynomialTrajectoryGenerator> smoothLoadingWRhoTrajectoryGenerators = new LinkedHashMap<ContactablePlaneBody, CubicPolynomialTrajectoryGenerator>();
   private final Map<ContactablePlaneBody, CubicPolynomialTrajectoryGenerator> smoothUnloadingWRhoTrajectoryGenerators = new LinkedHashMap<ContactablePlaneBody, CubicPolynomialTrajectoryGenerator>();


   private final PelvisDesiredsHandler pelvisDesiredsHandler; 
   private final DoubleYoVariable carIngressPelvisPositionKp = new DoubleYoVariable("carIngressPelvisPositionKp", registry);
   private final DoubleYoVariable carIngressPelvisPositionZeta = new DoubleYoVariable("carIngressPelvisPositionZeta", registry);
   private final DoubleYoVariable carIngressPelvisOrientationKp = new DoubleYoVariable("carIngressPelvisOrientationKp", registry);
   private final DoubleYoVariable carIngressPelvisOrientationZeta = new DoubleYoVariable("carIngressPelvisOrientationZeta", registry);
   
   private final DoubleYoVariable carIngressChestOrientationKp = new DoubleYoVariable("carIngressChestOrientationKp", registry);
   private final DoubleYoVariable carIngressChestOrientationZeta = new DoubleYoVariable("carIngressChestOrientationZeta", registry);
   
   private final DoubleYoVariable carIngressHeadOrientationKp = new DoubleYoVariable("carIngressHeadOrientationKp", registry);
   private final DoubleYoVariable carIngressHeadOrientationZeta = new DoubleYoVariable("carIngressHeadOrientationZeta", registry);
   

   public CarIngressEgressController(VariousWalkingProviders variousWalkingProviders, VariousWalkingManagers variousWalkingManagers,
                                     MomentumBasedController momentumBasedController, WalkingControllerParameters walkingControllerParameters,
                                     LidarControllerInterface lidarControllerInterface,
                                     DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      super(variousWalkingProviders, variousWalkingManagers, null, momentumBasedController, walkingControllerParameters,
            lidarControllerInterface, dynamicGraphicObjectsListRegistry, controllerState);

      setupManagers(variousWalkingManagers);

      coefficientOfFrictionForBumAndThighs.set(0.0);
      coefficientOfFrictionForFeet.set(0.6);

      elevator = fullRobotModel.getElevator();
      contactableThighs = momentumBasedController.getContactablePlaneThighs();
      contactablePelvis = momentumBasedController.getContactablePlanePelvis();
      contactablePelvisBack = momentumBasedController.getContactablePlanePelvisBack();
      
      this.variousWalkingManagers = variousWalkingManagers;
      this.pelvisDesiredsHandler = variousWalkingManagers.getPelvisDesiredsHandler();
      
      this.pelvisPoseProvider = variousWalkingProviders.getDesiredPelvisPoseProvider();
      this.footPoseProvider = variousWalkingProviders.getDesiredFootPoseProvider();
      this.footLoadBearingProvider = variousWalkingProviders.getDesiredFootStateProvider();
      this.thighLoadBearingProvider = variousWalkingProviders.getDesiredThighLoadBearingProvider();
      this.pelvisLoadBearingProvider = variousWalkingProviders.getDesiredPelvisLoadBearingProvider();
      
      // Setup the pelvis trajectory generator
      pelvisPositionControlFrame = fullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint();
      boolean visualize = true;
      this.pelvisController = new RigidBodySpatialAccelerationControlModule("pelvis", twistCalculator, fullRobotModel.getPelvis(), pelvisPositionControlFrame,
              visualize, registry);

      carIngressPelvisPositionKp.set(100.0);
      carIngressPelvisPositionZeta.set(1.0);
      carIngressPelvisOrientationKp.set(100.0);
      carIngressPelvisOrientationZeta.set(1.0);
      
      VariableChangedListener pelvisGainsChangedListener = createPelvisGainsChangedListener();
      
      
      pelvisGainsChangedListener.variableChanged(null);
      

      pelvisJacobian = new GeometricJacobian(fullRobotModel.getElevator(), fullRobotModel.getPelvis(), fullRobotModel.getPelvis().getBodyFixedFrame());

      initialPelvisConfigurationProvider = new ChangeableConfigurationProvider(new FramePose(pelvisPositionControlFrame));
      desiredPelvisConfigurationProvider = new ChangeableConfigurationProvider(new FramePose(pelvisPositionControlFrame));

      pelvisPositionTrajectoryGenerator = new StraightLinePositionTrajectoryGenerator("pelvis", worldFrame, trajectoryTimeProvider,
            initialPelvisConfigurationProvider, desiredPelvisConfigurationProvider, registry);
      pelvisOrientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator("pelvis", worldFrame, trajectoryTimeProvider,
            initialPelvisConfigurationProvider, desiredPelvisConfigurationProvider, registry);

      // Set up the chest trajectory generator
      this.chestOrientationProvider = variousWalkingProviders.getDesiredChestOrientationProvider();
      chestPositionControlFrame = fullRobotModel.getChest().getParentJoint().getFrameAfterJoint();
      initialDesiredChestOrientation = new SettableOrientationProvider("initialDesiredChest", worldFrame, registry);
      finalDesiredChestOrientation = new SettableOrientationProvider("finalDesiredChest", worldFrame, registry);
      chestOrientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator("chest", worldFrame, trajectoryTimeProvider,
            initialDesiredChestOrientation, finalDesiredChestOrientation, registry);

      setupFootControlModules();

      VariableChangedListener heelOffVariableChangedListener = new HeelOffYoVariableChangedListener();
      for (RobotSide robotSide : RobotSide.values)
      {
         doHeelOff.get(robotSide).addVariableChangedListener(heelOffVariableChangedListener);
      }

      LoadBearingVariableChangedListener loadBearingVariableChangedListener = new LoadBearingVariableChangedListener();

      requestedPelvisLoadBearing.addVariableChangedListener(loadBearingVariableChangedListener);
      requestedPelvisBackLoadBearing.addVariableChangedListener(loadBearingVariableChangedListener);
      for (RobotSide robotSide : RobotSide.values)
      {
         requestedFootLoadBearing.get(robotSide).addVariableChangedListener(loadBearingVariableChangedListener);
         requestedThighLoadBearing.get(robotSide).addVariableChangedListener(loadBearingVariableChangedListener);
      }

      // Parameters for smooth transitions between loading and unloading an end-effector.
      loadBearingTransitionTimeProvider = new YoVariableDoubleProvider("loadBearingTransitionTime", registry);
      loadBearingTransitionTimeProvider.set(0.5);
      double contactRegularizationWeightToUnloadEndEffector = 20.0 * PlaneContactState.DEFAULT_WRHO;

      setupTrajectoryGeneratorsForSmoothLoadBearingTransitions(contactRegularizationWeightToUnloadEndEffector);
      
      initialize();
   }

   private VariableChangedListener createPelvisGainsChangedListener()
   {
      VariableChangedListener ret = new VariableChangedListener()
      {
         public void variableChanged(YoVariable v)
         {
            double kPelvisPosition = carIngressPelvisPositionKp.getDoubleValue();
            double dPelvisPosition = GainCalculator.computeDerivativeGain(kPelvisPosition, carIngressPelvisPositionZeta.getDoubleValue());
            pelvisController.setPositionProportionalGains(kPelvisPosition , kPelvisPosition, kPelvisPosition);
            pelvisController.setPositionDerivativeGains(dPelvisPosition, dPelvisPosition, dPelvisPosition);

            double kPelvisOrientation = carIngressPelvisOrientationKp.getDoubleValue();
            double dPelvisOrientation = GainCalculator.computeDerivativeGain(kPelvisOrientation, carIngressPelvisOrientationZeta.getDoubleValue());
            pelvisController.setOrientationProportionalGains(kPelvisOrientation, kPelvisOrientation, kPelvisOrientation);
            pelvisController.setOrientationDerivativeGains(dPelvisOrientation, dPelvisOrientation, dPelvisOrientation);   
         }};
      
         carIngressPelvisPositionKp.addVariableChangedListener(ret);
         carIngressPelvisPositionZeta.addVariableChangedListener(ret);
         carIngressPelvisOrientationKp.addVariableChangedListener(ret);
         carIngressPelvisOrientationZeta.addVariableChangedListener(ret);
         
      return ret;
   }
   
   private VariableChangedListener createChestGainsChangedListener()
   {
      VariableChangedListener ret = new VariableChangedListener()
      {
         public void variableChanged(YoVariable v)
         {
            double chestKp = carIngressChestOrientationKp.getDoubleValue();
            double chestZeta = carIngressChestOrientationZeta.getDoubleValue();
            double chestKd = GainCalculator.computeDerivativeGain(chestKp, chestZeta);
            chestOrientationManager.setControlGains(chestKp, chestKd); 
         }};
         
         carIngressChestOrientationKp.addVariableChangedListener(ret);
         carIngressChestOrientationZeta.addVariableChangedListener(ret);
      
      return ret;
   }
   
   private VariableChangedListener createHeadGainsChangedListener()
   {
      VariableChangedListener ret = new VariableChangedListener()
      {
         public void variableChanged(YoVariable v)
         {
            double headKp = carIngressHeadOrientationKp.getDoubleValue();
            double headZeta = carIngressHeadOrientationZeta.getDoubleValue();
            double headKd = GainCalculator.computeDerivativeGain(headKp, headZeta);
            headOrientationManager.setControlGains(headKp, headKd); 
         }};
         
         carIngressHeadOrientationKp.addVariableChangedListener(ret);
         carIngressHeadOrientationZeta.addVariableChangedListener(ret);
      
      return ret;
   }

   
   private void setupTrajectoryGeneratorsForSmoothLoadBearingTransitions(double contactRegularizationWeightToUnloadEndEffector)
   {
      List<ContactablePlaneBody> contactablePlaneBodies = new ArrayList<ContactablePlaneBody>();
      contactablePlaneBodies.addAll(feet.values());
      contactablePlaneBodies.addAll(contactableThighs.values());
//      contactablePlaneBodies.addAll(handPalms.values());
      contactablePlaneBodies.add(contactablePelvis);
//      contactablePlaneBodies.add(contactablePelvisBack);
      
      DoubleProvider wRhoForPlaneBodyLoaded = new ConstantDoubleProvider(PlaneContactState.DEFAULT_WRHO);
      DoubleProvider wRhoForPlaneBodyUnloaded = new ConstantDoubleProvider(contactRegularizationWeightToUnloadEndEffector);
      
      for (ContactablePlaneBody contactablePlaneBody : contactablePlaneBodies)
      {
         RigidBody rigidBody = contactablePlaneBody.getRigidBody();
         String namePrefix = rigidBody.getName();
         
         CubicPolynomialTrajectoryGenerator smoothLoadingWRhoTrajectoryGenerator = new CubicPolynomialTrajectoryGenerator(
               namePrefix + "_UnloadingTrajectory", wRhoForPlaneBodyUnloaded, wRhoForPlaneBodyLoaded, loadBearingTransitionTimeProvider, registry);
         CubicPolynomialTrajectoryGenerator smoothUnloadingWRhoTrajectoryGenerator = new CubicPolynomialTrajectoryGenerator(
               namePrefix + "_LoadingTrajectory", wRhoForPlaneBodyLoaded, wRhoForPlaneBodyUnloaded, loadBearingTransitionTimeProvider, registry);

         smoothLoadingWRhoTrajectoryGenerator.initialize();
         smoothUnloadingWRhoTrajectoryGenerator.initialize();
         
         loadBearingTransitionStartTime.put(contactablePlaneBody, new DoubleYoVariable(namePrefix + "_LoadBearingTransitionStartTime", registry));
         loadingTransitionHasBeenInitiated.put(contactablePlaneBody, new BooleanYoVariable(namePrefix + "_LoadingTransitionHasBeenInitiated", registry));
         unloadingTransitionHasBeenInitiated.put(contactablePlaneBody, new BooleanYoVariable(namePrefix + "_UnloadingTransitionHasBeenInitiated", registry));
         smoothLoadingWRhoTrajectoryGenerators.put(contactablePlaneBody, smoothLoadingWRhoTrajectoryGenerator);
         smoothUnloadingWRhoTrajectoryGenerators.put(contactablePlaneBody, smoothUnloadingWRhoTrajectoryGenerator);

         doLoadingTransition.put(contactablePlaneBody, false);
         doUnloadingTransition.put(contactablePlaneBody, false);
      }
   }

   protected void setupFootControlModules()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody foot = feet.get(robotSide);
         GeometricJacobian jacobian = legJacobians.get(robotSide);

         String bodyName = foot.getRigidBody().getName();
         String sideString = robotSide.getCamelCaseNameForStartOfExpression();

         final ChangeableConfigurationProvider initialConfigurationProvider = new ChangeableConfigurationProvider(new FramePose(foot.getBodyFrame()));
         final ChangeableConfigurationProvider desiredConfigurationProvider = new ChangeableConfigurationProvider(new FramePose(foot.getBodyFrame()));

         StraightLinePositionTrajectoryGenerator positionTrajectoryGenerator = new StraightLinePositionTrajectoryGenerator(bodyName, worldFrame,
                                                                                  footTrajectoryTimeProvider, initialConfigurationProvider,
                                                                                  desiredConfigurationProvider, registry);

         OrientationInterpolationTrajectoryGenerator orientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator(bodyName, worldFrame,
                                                                                         footTrajectoryTimeProvider, initialConfigurationProvider,
                                                                                         desiredConfigurationProvider, registry);
         
         PoseTrajectoryGenerator poseTrajectoryGenerator = new WrapperForPositionAndOrientationTrajectoryGenerators(positionTrajectoryGenerator, orientationTrajectoryGenerator);

         initialFootConfigurationProviders.put(foot, initialConfigurationProvider);
         desiredFootConfigurationProviders.put(foot, desiredConfigurationProvider);
         swingPositionTrajectoryGenerators.put(foot, positionTrajectoryGenerator);
         swingOrientationTrajectoryGenerators.put(foot, orientationTrajectoryGenerator);

         DoubleProvider onToesInitialPitchProvider = new ConstantDoubleProvider(0.0);
         DoubleProvider onToesInitialPitchVelocityProvider = new ConstantDoubleProvider(0.0);
         DoubleProvider onToesFinalPitchProvider = new ConstantDoubleProvider(Math.PI / 4.0);

         DoubleTrajectoryGenerator onToesTrajectory = new ThirdOrderPolynomialTrajectoryGenerator(sideString + bodyName, onToesInitialPitchProvider,
                                                         onToesInitialPitchVelocityProvider, onToesFinalPitchProvider, trajectoryTimeProvider, registry);

         EndEffectorControlModule endEffectorControlModule = new EndEffectorControlModule(foot, jacobian, poseTrajectoryGenerator, null,
                                                                onToesTrajectory, momentumBasedController, registry);
         footEndEffectorControlModules.put(foot, endEffectorControlModule);
      }
   }

   private RigidBody baseForChestOrientationControl;
   private GeometricJacobian jacobianForChestOrientationControl;
   
   private RigidBody baseForHeadOrientationControl;
   private GeometricJacobian jacobianForHeadOrientationControl;

   private void setupManagers(VariousWalkingManagers variousWalkingManagers)
   {
      baseForChestOrientationControl = fullRobotModel.getPelvis();
      ChestOrientationManager chestOrientationManager = variousWalkingManagers.getChestOrientationManager();
      String[] chestOrientationControlJointNames = walkingControllerParameters.getDefaultChestOrientationControlJointNames();
      jacobianForChestOrientationControl = chestOrientationManager.createJacobian(fullRobotModel, chestOrientationControlJointNames);
      
      baseForHeadOrientationControl = fullRobotModel.getPelvis();
      HeadOrientationManager headOrientationManager = variousWalkingManagers.getHeadOrientationManager();
      String[] headOrientationControlJointNames = walkingControllerParameters.getDefaultHeadOrientationControlJointNames();
      jacobianForHeadOrientationControl = headOrientationManager.createJacobian(fullRobotModel, baseForHeadOrientationControl, headOrientationControlJointNames);
   }
   
   public void initialize()
   {
      super.initialize();

      momentumBasedController.setMomentumControlModuleToUse(MOMENTUM_CONTROL_MODULE_TO_USE);
      momentumBasedController.setDelayTimeBeforeTrustingContacts(DELAY_TIME_BEFORE_TRUSTING_CONTACTS);
      
      chestOrientationManager.setUp(baseForChestOrientationControl, jacobianForChestOrientationControl);
      carIngressChestOrientationKp.set(100.0);
      carIngressChestOrientationZeta.set(1.0);
      VariableChangedListener chestGainsChangedListener = createChestGainsChangedListener();
      chestGainsChangedListener.variableChanged(null);
      
      headOrientationManager.setUp(baseForHeadOrientationControl, jacobianForHeadOrientationControl);
      carIngressHeadOrientationKp.set(40.0);
      carIngressHeadOrientationZeta.set(1.0);
      VariableChangedListener headGainsChangedListener = createHeadGainsChangedListener();
      headGainsChangedListener.variableChanged(null);
      
      initializeContacts();

      FramePose initialDesiredPelvisPose;
      if (pelvisDesiredsHandler.areDesiredsValid())
      {
         System.out.println("desired pelvis pose valid: initializing to desired");
         initialDesiredPelvisPose = pelvisDesiredsHandler.getDesiredPelvisPose();
      }
      else
      {
         System.out.println("desired pelvis pose invalid: initializing to current");
         FramePose currentPelvisPose = new FramePose(pelvisPositionControlFrame);
         initialDesiredPelvisPose = currentPelvisPose;
      }

      initialDesiredPelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      initialPelvisConfigurationProvider.set(initialDesiredPelvisPose);
      desiredPelvisConfigurationProvider.set(initialDesiredPelvisPose);

      pelvisTrajectoryStartTime = yoTime.getDoubleValue();
      pelvisPositionTrajectoryGenerator.initialize();
      pelvisOrientationTrajectoryGenerator.initialize();


//      FrameOrientation initialDesiredChestOrientation;
//      if (chestOrientationManager.areDesiredsValid())
//      {
//         System.out.println("desired chest orientation valid: initializing to desired");
//         initialDesiredChestOrientation = chestOrientationManager.getDesiredChestOrientation();
//      }
//      else
//      {
//         System.out.println("desired chest orientation invalid: initializing to current");
//         initialDesiredChestOrientation = new FrameOrientation(chestPositionControlFrame);
//      }

      FrameOrientation initialDesiredChestOrientation = new FrameOrientation(chestPositionControlFrame);
      initialDesiredChestOrientation.changeFrame(this.initialDesiredChestOrientation.getReferenceFrame());
      this.initialDesiredChestOrientation.setOrientation(initialDesiredChestOrientation);
      finalDesiredChestOrientation.setOrientation(initialDesiredChestOrientation);

      chestTrajectoryStartTime = yoTime.getDoubleValue();
      chestOrientationTrajectoryGenerator.initialize();
      
      for (RobotSide robotSide : RobotSide.values)
         footEndEffectorControlModules.get(feet.get(robotSide)).resetCurrentState();
   }

   
   private void initializeContacts()
   {
      requestedPelvisLoadBearing.set(isContactablePlaneBodyInContact(contactablePelvis));
      requestedPelvisLoadBearing.notifyVariableChangedListeners();
      requestedPelvisBackLoadBearing.set(false); // Set to false there is no button in the GUI to change it anymore
      requestedPelvisBackLoadBearing.notifyVariableChangedListeners();
      
      for (RobotSide robotSide : RobotSide.values)
      {
         requestedFootLoadBearing.get(robotSide).set(isContactablePlaneBodyInContact(feet.get(robotSide)));
         requestedFootLoadBearing.get(robotSide).notifyVariableChangedListeners();
         requestedThighLoadBearing.get(robotSide).set(false); // Set to false there is no button in the GUI to change it anymore
         requestedThighLoadBearing.get(robotSide).notifyVariableChangedListeners();
      }
   }

   public void doMotionControl()
   {
      momentumBasedController.doPrioritaryControl();
      callUpdatables();
      updateLoadBearingStates();

//      doContactPointControl();
      doFootControl();
      doArmControl();
      doHeadControl();
      doLidarJointControl();
      doChestControl();
      doCoMControl();
      doPelvisControl();
      doJointPositionControl();

      setTorqueControlJointsToZeroDersiredAcceleration();

      momentumBasedController.doSecondaryControl();
   }

   protected void doPelvisControl()
   {
      if (pelvisPoseProvider.checkForNewPose())
      {
         double time = yoTime.getDoubleValue() - pelvisTrajectoryStartTime;
         pelvisPositionTrajectoryGenerator.compute(time);
         pelvisOrientationTrajectoryGenerator.compute(time);

         FramePoint previousDesiredPosition = new FramePoint(pelvisPositionControlFrame);
         FrameOrientation previousDesiredOrientation = new FrameOrientation(pelvisPositionControlFrame);
         pelvisPositionTrajectoryGenerator.get(previousDesiredPosition);
         pelvisOrientationTrajectoryGenerator.get(previousDesiredOrientation);
         
         FramePose previousDesiredPelvisConfiguration = new FramePose(previousDesiredPosition, previousDesiredOrientation);
         initialPelvisConfigurationProvider.set(previousDesiredPelvisConfiguration);
         
         desiredPelvisConfigurationProvider.set(pelvisPoseProvider.getDesiredPelvisPose());
         pelvisTrajectoryStartTime = yoTime.getDoubleValue();

         pelvisPositionTrajectoryGenerator.initialize();
         pelvisOrientationTrajectoryGenerator.initialize();
      }

      double time = yoTime.getDoubleValue() - pelvisTrajectoryStartTime;
      pelvisPositionTrajectoryGenerator.compute(time);
      pelvisOrientationTrajectoryGenerator.compute(time);

      FramePoint desiredPosition = new FramePoint(pelvisPositionControlFrame);
      FrameVector desiredVelocity = new FrameVector(pelvisPositionControlFrame);
      FrameVector desiredPelvisAcceleration = new FrameVector(pelvisPositionControlFrame);

      pelvisPositionTrajectoryGenerator.packLinearData(desiredPosition, desiredVelocity, desiredPelvisAcceleration);

      FrameOrientation desiredOrientation = new FrameOrientation(pelvisPositionControlFrame);
      FrameVector desiredAngularVelocity = new FrameVector(pelvisPositionControlFrame);
      FrameVector desiredAngularAcceleration = new FrameVector(pelvisPositionControlFrame);

      pelvisOrientationTrajectoryGenerator.packAngularData(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);

      pelvisController.doPositionControl(desiredPosition, desiredOrientation, desiredVelocity, desiredAngularVelocity, desiredPelvisAcceleration,
                                         desiredAngularAcceleration, fullRobotModel.getElevator());

      pelvisDesiredsHandler.setDesireds(desiredPosition, desiredVelocity, desiredPelvisAcceleration, desiredOrientation, desiredAngularVelocity,
            desiredAngularAcceleration);

      SpatialAccelerationVector pelvisSpatialAcceleration = new SpatialAccelerationVector();
      pelvisController.packAcceleration(pelvisSpatialAcceleration);

      pelvisSpatialAcceleration.changeBodyFrameNoRelativeAcceleration(fullRobotModel.getPelvis().getBodyFixedFrame());
      pelvisSpatialAcceleration.changeFrameNoRelativeMotion(fullRobotModel.getPelvis().getBodyFixedFrame());

      if (!requestedPelvisLoadBearing.getBooleanValue())
      {
         pelvisTaskspaceConstraintData.set(pelvisSpatialAcceleration);
      }
      else
      {
         DenseMatrix64F selectionMatrix = new DenseMatrix64F(5, SpatialMotionVector.SIZE);
         for (int i = 0; i < selectionMatrix.getNumRows(); i++)
         {
            selectionMatrix.set(i, i, 1.0);
         }
         pelvisTaskspaceConstraintData.set(pelvisSpatialAcceleration, pelvisNullspaceMultipliers, selectionMatrix);
      }
      
      momentumBasedController.setDesiredSpatialAcceleration(pelvisJacobian, pelvisTaskspaceConstraintData);

      desiredOrientation.changeFrame(this.desiredPelvisOrientation.getReferenceFrame());
      desiredAngularVelocity.changeFrame(this.desiredPelvisAngularVelocity.getReferenceFrame());
      desiredAngularAcceleration.changeFrame(this.desiredPelvisAngularAcceleration.getReferenceFrame());


      desiredPelvisOrientation.set(desiredOrientation);
      desiredPelvisAngularVelocity.set(desiredAngularVelocity);
      desiredPelvisAngularAcceleration.set(desiredAngularAcceleration);
   }

   protected void doChestControl()
   {
      if (chestOrientationProvider.checkForNewPose())
      {
         chestOrientationTrajectoryGenerator.compute(yoTime.getDoubleValue() - chestTrajectoryStartTime);
         FrameOrientation previousDesiredChestOrientation = new FrameOrientation(chestPositionControlFrame);
         chestOrientationTrajectoryGenerator.get(previousDesiredChestOrientation);
         initialDesiredChestOrientation.setOrientation(previousDesiredChestOrientation);
         
         finalDesiredChestOrientation.setOrientation(chestOrientationProvider.getDesiredChestOrientation());
         chestTrajectoryStartTime = yoTime.getDoubleValue();

         chestOrientationTrajectoryGenerator.initialize();
      }

      chestOrientationTrajectoryGenerator.compute(yoTime.getDoubleValue() - chestTrajectoryStartTime);
      FrameOrientation desiredOrientation = new FrameOrientation(chestPositionControlFrame);
      FrameVector desiredAngularVelocity = new FrameVector(chestPositionControlFrame);
      FrameVector desiredAngularAcceleration = new FrameVector(chestPositionControlFrame);
      chestOrientationTrajectoryGenerator.get(desiredOrientation);
      chestOrientationTrajectoryGenerator.packAngularVelocity(desiredAngularVelocity);
      chestOrientationTrajectoryGenerator.packAngularAcceleration(desiredAngularAcceleration);

      chestOrientationManager.setDesireds(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);

      super.doChestControl();
   }

   protected void doFootControl()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody foot = feet.get(robotSide);

         if (footPoseProvider.checkForNewPose(robotSide) && !requestedFootLoadBearing.get(robotSide).getBooleanValue())
         {
            FramePose oldFootPose = new FramePose(foot.getBodyFrame());
            desiredFootConfigurationProviders.get(foot).get(oldFootPose);
            initialFootConfigurationProviders.get(foot).set(oldFootPose);
            
            FramePose newFootPose = footPoseProvider.getDesiredFootPose(robotSide);
            desiredFootConfigurationProviders.get(foot).set(newFootPose);
            footEndEffectorControlModules.get(foot).resetCurrentState();
         }
      }

      super.doFootControl();
   }

   private void doContactPointControl()
   {
      for (ContactablePlaneBody body : bodiesInContact)
      {
         GeometricJacobian jacobian = contactJacobians.get(body);
         jacobian.compute();
         FrameVector desiredAcceleration = new FrameVector(jacobian.getBaseFrame(), 0.0, 0.0, 0.0);
         DenseMatrix64F selectionMatrix = new DenseMatrix64F(1, 3);
         selectionMatrix.set(0, 2, 1.0);
         for (FramePoint contactPoint : body.getContactPoints())
         {
            momentumBasedController.setDesiredPointAcceleration(jacobian, contactPoint, desiredAcceleration, selectionMatrix);
         }
      }
   }

   private void addBodyInContact(ContactablePlaneBody contactablePlaneBody)
   {
      for (int i = 0; i < bodiesInContact.size(); i++)
      {
         if (contactablePlaneBody.equals(bodiesInContact.get(i)))
            return;
      }
      bodiesInContact.add(contactablePlaneBody);
      RigidBody rigidBody = contactablePlaneBody.getRigidBody();
      contactJacobians.put(contactablePlaneBody, new GeometricJacobian(elevator, rigidBody, elevator.getBodyFixedFrame()));
   }

   private void removeBodyInContact(ContactablePlaneBody contactablePlaneBody)
   {
      bodiesInContact.remove(contactablePlaneBody);
      contactJacobians.remove(contactablePlaneBody);
   }
   
   private boolean desiredPelvisLoadBearingState;
   
   private void updateLoadBearingStates()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody foot = feet.get(robotSide);
         if (footPoseProvider.checkForNewPose(robotSide))
         {
            if (doUnloadingTransition.get(foot) && footPoseProvider.checkIfPreviousPoseNotConsumed(robotSide))
            {
               doUnloadingTransition.put(foot, false);
               requestedFootLoadBearing.get(robotSide).set(false);
            }
            else
            {
               doUnloadingTransition.put(foot, true);
            }
            // Cancel loading action if any:
            doLoadingTransition.put(foot, false);
            
            
            // When unloading a foot, relatch where all the other foot positions are. 
            // Otherwise there might be a jump.
            
            // Nope. Don't do it, because if the feet positions are "resisting" the estimator moving z
            // away due to errors in tracking (desired accelerations don't equal actual accelerations),
            // then if you reset the positions to current, you get a large jump. So not doing it is the
            // lesser of two evils.
//            momentumBasedController.requestResetEstimatorPositionsToCurrent();
         }
         
         // If the foot is already in load bearing state, do nothing:
         if (footLoadBearingProvider.checkForNewLoadBearingRequest(robotSide))
         {
            if (!requestedFootLoadBearing.get(robotSide).getBooleanValue())
            {
               requestedFootLoadBearing.get(robotSide).set(true);
               doLoadingTransition.put(foot, true);
            }
            else
            {
               doLoadingTransition.put(foot, false);
            }
            // Cancel unloading action if any:
            footPoseProvider.getDesiredFootPose(robotSide);
            doUnloadingTransition.put(feet.get(robotSide), false);
         }

         performSmoothUnloadingTransition(foot, requestedFootLoadBearing.get(robotSide));
         performSmoothLoadingTransition(foot);
         
         if (thighLoadBearingProvider.checkForNewLoadBearingState(robotSide))
         {
            requestedThighLoadBearing.get(robotSide).set(thighLoadBearingProvider.getDesiredThighLoadBearingState(robotSide));
         }
      }

      if (pelvisLoadBearingProvider.checkForNewLoadBearingState())
      {
         desiredPelvisLoadBearingState = pelvisLoadBearingProvider.getDesiredPelvisLoadBearingState();
         if (desiredPelvisLoadBearingState && !requestedPelvisLoadBearing.getBooleanValue()) // Loading
         {
            requestedPelvisLoadBearing.set(desiredPelvisLoadBearingState);
            doLoadingTransition.put(contactablePelvis, true);
            doUnloadingTransition.put(contactablePelvis, false);
            
         }
         else if (!desiredPelvisLoadBearingState && requestedPelvisLoadBearing.getBooleanValue()) // Unloading
         {
            doUnloadingTransition.put(contactablePelvis, true);
            doLoadingTransition.put(contactablePelvis, false);
         }
      }

      performSmoothUnloadingTransition(contactablePelvis, requestedPelvisLoadBearing);
      performSmoothLoadingTransition(contactablePelvis);
   }

   private void performSmoothUnloadingTransition(ContactablePlaneBody contactablePlaneBody, BooleanYoVariable requestedLoadBearing)
   {
      if (!doUnloadingTransition.get(contactablePlaneBody))
      {
         unloadingTransitionHasBeenInitiated.get(contactablePlaneBody).set(false);
         return;
      }
      
      DoubleYoVariable transitionStartTime = loadBearingTransitionStartTime.get(contactablePlaneBody);

      if (!unloadingTransitionHasBeenInitiated.get(contactablePlaneBody).getBooleanValue())
      {
         unloadingTransitionHasBeenInitiated.get(contactablePlaneBody).set(true);
         transitionStartTime.set(yoTime.getDoubleValue());
      }

      double time = yoTime.getDoubleValue() - transitionStartTime.getDoubleValue();

      if (time >= loadBearingTransitionTimeProvider.getValue())
      {
         requestedLoadBearing.set(false);
         doUnloadingTransition.put(contactablePlaneBody, false);
         
         return;
      }

      CubicPolynomialTrajectoryGenerator smoothUnloadingWRhoTrajectoryGenerator = smoothUnloadingWRhoTrajectoryGenerators.get(contactablePlaneBody);

      smoothUnloadingWRhoTrajectoryGenerator.compute(time);
      double wRho = smoothUnloadingWRhoTrajectoryGenerator.getValue();
      momentumBasedController.setPlaneContactState_wRho(contactablePlaneBody, wRho);

   }

   private void performSmoothLoadingTransition(ContactablePlaneBody contactablePlaneBody)
   {
      if (!doLoadingTransition.get(contactablePlaneBody))
      {
         loadingTransitionHasBeenInitiated.get(contactablePlaneBody).set(false);
         return;
      }
      
      DoubleYoVariable transitionStartTime = loadBearingTransitionStartTime.get(contactablePlaneBody);

      if (!loadingTransitionHasBeenInitiated.get(contactablePlaneBody).getBooleanValue())
      {
         loadingTransitionHasBeenInitiated.get(contactablePlaneBody).set(true);
         transitionStartTime.set(yoTime.getDoubleValue());
      }

      double time = yoTime.getDoubleValue() - transitionStartTime.getDoubleValue();

      if (time >= loadBearingTransitionTimeProvider.getValue())
      {
         doLoadingTransition.put(contactablePlaneBody, false);
      }

      CubicPolynomialTrajectoryGenerator smoothLoadingWRhoTrajectoryGenerator = smoothLoadingWRhoTrajectoryGenerators.get(contactablePlaneBody);

      smoothLoadingWRhoTrajectoryGenerator.compute(time);
      double wRho = smoothLoadingWRhoTrajectoryGenerator.getValue();
      momentumBasedController.setPlaneContactState_wRho(contactablePlaneBody, wRho);
   }

   public boolean isContactablePlaneBodyInContact(ContactablePlaneBody contactablePlaneBody)
   {
      if (momentumBasedController.getContactPoints(contactablePlaneBody).size() == 0)
         return false;
      else
         return true;
   }
   
   public void setFootInContact(RobotSide robotSide, boolean inContact)
   {
      if (feet == null)
         return;

      if (inContact)
      {
         setFlatFootContactState(feet.get(robotSide));
      }
      else
      {
         setContactStateForSwing(feet.get(robotSide));
      }
   }

   public void setHandInContact(RobotSide robotSide, boolean inContact)
   {
      ContactablePlaneBody handPalm = handPalms.get(robotSide);
      if (inContact)
      {
         // TODO: If we know the surface normal here, use it.
         FrameVector normalContactVector = null;
         momentumBasedController.setPlaneContactStateFullyConstrained(handPalm, coefficientOfFriction.getDoubleValue(), normalContactVector);
      }
      else
      {
         momentumBasedController.setPlaneContactStateFree(handPalm);
      }
   }

   public void setThighInContact(RobotSide robotSide, boolean inContact)
   {
      ContactablePlaneBody thigh = contactableThighs.get(robotSide);
      if (inContact)
      {
         momentumBasedController.setPlaneContactStateFullyConstrained(thigh, coefficientOfFrictionForBumAndThighs.getDoubleValue(), null);
         addBodyInContact(thigh);
      }
      else
      {
         momentumBasedController.setPlaneContactStateFree(thigh);
         removeBodyInContact(thigh);
      }
   }

   public void setPelvisInContact(boolean inContact)
   {
      if (inContact)
      {
         momentumBasedController.setPlaneContactStateFullyConstrained(contactablePelvis, coefficientOfFrictionForBumAndThighs.getDoubleValue(), null);
         addBodyInContact(contactablePelvis);
      }
      else
      {
         momentumBasedController.setPlaneContactStateFree(contactablePelvis);
         removeBodyInContact(contactablePelvis);
      }
   }

   public void setPelvisBackInContact(boolean inContact)
   {
      if (inContact)
      {
         momentumBasedController.setPlaneContactStateFullyConstrained(contactablePelvisBack, coefficientOfFrictionForBumAndThighs.getDoubleValue(), null);
         addBodyInContact(contactablePelvisBack);
      }
      else
      {
         momentumBasedController.setPlaneContactStateFree(contactablePelvisBack);
         removeBodyInContact(contactablePelvisBack);
      }
   }

   private void setOnToesContactState(ContactablePlaneBody contactableBody)
   {
      footEndEffectorControlModules.get(contactableBody).setContactState(ConstraintType.TOES);
   }

   private void setFlatFootContactState(ContactablePlaneBody contactableBody)
   {
      footEndEffectorControlModules.get(contactableBody).setContactState(ConstraintType.FULL);
   }

   private void setContactStateForSwing(ContactablePlaneBody contactableBody)
   {
      // Initialize desired foot pose to the actual, so no surprising behavior
      ReferenceFrame footFrame = footEndEffectorControlModules.get(contactableBody).getEndEffectorFrame();
      desiredFootConfigurationProviders.get(contactableBody).set(new FramePose(footFrame));

      footEndEffectorControlModules.get(contactableBody).doSingularityEscapeBeforeTransitionToNextState();
      footEndEffectorControlModules.get(contactableBody).setContactState(ConstraintType.UNCONSTRAINED);
   }

   private class LoadBearingVariableChangedListener implements VariableChangedListener
   {
      public void variableChanged(YoVariable v)
      {
         if (!(v instanceof BooleanYoVariable))
            return;

         if (v.equals(requestedPelvisLoadBearing))
            setPelvisInContact(requestedPelvisLoadBearing.getBooleanValue());

         if (v.equals(requestedPelvisBackLoadBearing))
            setPelvisBackInContact(requestedPelvisBackLoadBearing.getBooleanValue());
         
         for (RobotSide robotSide : RobotSide.values)
         {
            if (v.equals(requestedFootLoadBearing.get(robotSide)))
               setFootInContact(robotSide, requestedFootLoadBearing.get(robotSide).getBooleanValue());

            if (v.equals(requestedThighLoadBearing.get(robotSide)))
               setThighInContact(robotSide, requestedThighLoadBearing.get(robotSide).getBooleanValue());
         }
      }
   }


   private class HeelOffYoVariableChangedListener implements VariableChangedListener
   {
      public void variableChanged(YoVariable v)
      {
         if (!(v instanceof BooleanYoVariable))
            return;

         for (RobotSide robotSide : RobotSide.values)
         {
            if (v.equals(doHeelOff.get(robotSide)))
            {
               if (doHeelOff.get(robotSide).getBooleanValue())
               {
                  setOnToesContactState(feet.get(robotSide));
               }
               else
               {
                  setFlatFootContactState(feet.get(robotSide));
               }
            }
         }
      }
   }
}
