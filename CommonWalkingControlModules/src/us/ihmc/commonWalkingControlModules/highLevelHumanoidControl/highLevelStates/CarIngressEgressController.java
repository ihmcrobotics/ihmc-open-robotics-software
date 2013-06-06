package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.calculators.GainCalculator;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.ChestOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.desiredChestOrientation.DesiredChestOrientationProvider;
import us.ihmc.commonWalkingControlModules.controlModules.endEffector.EndEffectorControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.endEffector.EndEffectorControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controlModules.head.HeadOrientationManager;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredFootStateProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredPelvisLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredPelvisPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredThighLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.controllers.LidarControllerInterface;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorTools;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingManagers;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredFootPoseProvider;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController.MomentumControlModuleType;
import us.ihmc.commonWalkingControlModules.trajectories.ChangeableConfigurationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantConfigurationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.CurrentOrientationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.OrientationInterpolationTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.SettableOrientationProvider;
import us.ihmc.commonWalkingControlModules.trajectories.StraightLinePositionTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.ThirdOrderPolynomialTrajectoryGenerator;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.VariableChangedListener;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.trajectory.ConstantDoubleProvider;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleProvider;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleTrajectoryGenerator;

public class CarIngressEgressController extends AbstractHighLevelHumanoidControlPattern
{
   public final static HighLevelState controllerState = HighLevelState.INGRESS_EGRESS;
   private final static MomentumControlModuleType MOMENTUM_CONTROL_MODULE_TO_USE = MomentumControlModuleType.OPTIMIZATION;

   private final DesiredFootPoseProvider footPoseProvider;
   private final DesiredFootStateProvider footLoadBearingProvider;
   private DesiredThighLoadBearingProvider thighLoadBearingProvider;
   private DesiredPelvisLoadBearingProvider pelvisLoadBearingProvider;

   private final DesiredPelvisPoseProvider pelvisPoseProvider;
   private final RigidBodySpatialAccelerationControlModule pelvisController;
   private final ReferenceFrame pelvisPositionControlFrame;
   private final GeometricJacobian pelvisJacobian;
   private final TaskspaceConstraintData pelvisTaskspaceConstraintData = new TaskspaceConstraintData();
   private final ChangeableConfigurationProvider desiredPelvisConfigurationProvider;
   private final StraightLinePositionTrajectoryGenerator pelvisPositionTrajectoryGenerator;
   private final OrientationInterpolationTrajectoryGenerator pelvisOrientationTrajectoryGenerator;
   private double pelvisTrajectoryStartTime = 0.0;

   private final DesiredChestOrientationProvider chestOrientationProvider;
   private final ReferenceFrame chestPositionControlFrame;
   private final SettableOrientationProvider desiredChestOrientation;
   private final OrientationInterpolationTrajectoryGenerator chestOrientationTrajectoryGenerator;
   private double chestTrajectoryStartTime = 0.0;

   private final BooleanYoVariable l_footDoHeelOff = new BooleanYoVariable("l_footDoHeelOff", registry);
   private final BooleanYoVariable r_footDoHeelOff = new BooleanYoVariable("r_footDoHeelOff", registry);
   private final SideDependentList<BooleanYoVariable> doHeelOff = new SideDependentList<BooleanYoVariable>(l_footDoHeelOff, r_footDoHeelOff);

   private final LinkedHashMap<ContactablePlaneBody, ChangeableConfigurationProvider> desiredFootConfigurationProviders =
      new LinkedHashMap<ContactablePlaneBody, ChangeableConfigurationProvider>();
   private final LinkedHashMap<ContactablePlaneBody, StraightLinePositionTrajectoryGenerator> swingPositionTrajectoryGenerators =
      new LinkedHashMap<ContactablePlaneBody, StraightLinePositionTrajectoryGenerator>();
   private final LinkedHashMap<ContactablePlaneBody, OrientationInterpolationTrajectoryGenerator> swingOrientationTrajectoryGenerators =
      new LinkedHashMap<ContactablePlaneBody, OrientationInterpolationTrajectoryGenerator>();

   private final ConstantDoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(1.0);

   private final SideDependentList<ContactablePlaneBody> contactableThighs;
   private final ContactablePlaneBody contactablePelvis, contactablePelvisBack;
   
   private final RigidBody elevator;
   private final List<ContactablePlaneBody> bodiesInContact = new ArrayList<ContactablePlaneBody>();
   private final Map<ContactablePlaneBody, GeometricJacobian> contactJacobians = new LinkedHashMap<ContactablePlaneBody, GeometricJacobian>();

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

   public CarIngressEgressController(VariousWalkingProviders variousWalkingProviders, VariousWalkingManagers variousWalkingManagers,
                                     MomentumBasedController momentumBasedController, WalkingControllerParameters walkingControllerParameters,
                                     LidarControllerInterface lidarControllerInterface,
                                     DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      super(variousWalkingProviders, variousWalkingManagers, null, momentumBasedController, walkingControllerParameters,
            lidarControllerInterface, dynamicGraphicObjectsListRegistry, controllerState);

      setupManagers(variousWalkingManagers);
      
      elevator = fullRobotModel.getElevator();
      contactableThighs = momentumBasedController.getContactablePlaneThighs();
      contactablePelvis = momentumBasedController.getContactablePlanePelvis();
      contactablePelvisBack = momentumBasedController.getContactablePlanePelvisBack();
      
      this.variousWalkingManagers = variousWalkingManagers;
      
      this.pelvisPoseProvider = variousWalkingProviders.getDesiredPelvisPoseProvider();
      this.footPoseProvider = variousWalkingProviders.getDesiredFootPoseProvider();
      this.footLoadBearingProvider = variousWalkingProviders.getDesiredFootStateProvider();
      this.thighLoadBearingProvider = variousWalkingProviders.getDesiredThighLoadBearingProvider();
      this.pelvisLoadBearingProvider = variousWalkingProviders.getDesiredPelvisLoadBearingProvider();
      
      // Setup the pelvis trajectory generator
      pelvisPositionControlFrame = fullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint();
      this.pelvisController = new RigidBodySpatialAccelerationControlModule("pelvis", twistCalculator, fullRobotModel.getPelvis(), pelvisPositionControlFrame,
              registry);

      double kPelvisPosition = 100.0;
      double dPelvisPosition = GainCalculator.computeDerivativeGain(kPelvisPosition, 1.0);
      pelvisController.setPositionProportionalGains(kPelvisPosition, kPelvisPosition, kPelvisPosition);
      pelvisController.setPositionDerivativeGains(dPelvisPosition, dPelvisPosition, dPelvisPosition);

      double kPelvisOrientation = 100.0;
      double dPelvisOrientation = GainCalculator.computeDerivativeGain(kPelvisOrientation, 1.0);
      pelvisController.setOrientationProportionalGains(kPelvisOrientation, kPelvisOrientation, kPelvisOrientation);
      pelvisController.setOrientationDerivativeGains(dPelvisOrientation, dPelvisOrientation, dPelvisOrientation);


      pelvisJacobian = new GeometricJacobian(fullRobotModel.getElevator(), fullRobotModel.getPelvis(), fullRobotModel.getPelvis().getBodyFixedFrame());

      final ConstantConfigurationProvider currentPelvisConfigurationProvider = new ConstantConfigurationProvider(new FramePose(pelvisPositionControlFrame));
      desiredPelvisConfigurationProvider = new ChangeableConfigurationProvider(new FramePose(pelvisPositionControlFrame));

      pelvisPositionTrajectoryGenerator = new StraightLinePositionTrajectoryGenerator("pelvis", worldFrame, trajectoryTimeProvider,
              currentPelvisConfigurationProvider, desiredPelvisConfigurationProvider, registry);
      pelvisOrientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator("pelvis", worldFrame, trajectoryTimeProvider,
              currentPelvisConfigurationProvider, desiredPelvisConfigurationProvider, registry);

      // Set up the chest trajectory generator
      this.chestOrientationProvider = variousWalkingProviders.getDesiredChestOrientationProvider();
      chestPositionControlFrame = fullRobotModel.getChest().getParentJoint().getFrameAfterJoint();
      final CurrentOrientationProvider currentChestOrientationProvider = new CurrentOrientationProvider(worldFrame, chestPositionControlFrame);    // TODO: not sure about that
      desiredChestOrientation = new SettableOrientationProvider("desiredChestProvider", worldFrame, registry);
      chestOrientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator("chest", worldFrame, trajectoryTimeProvider,
              currentChestOrientationProvider, desiredChestOrientation, registry);

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
   }

   protected void setupFootControlModules()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody foot = feet.get(robotSide);
         GeometricJacobian jacobian = legJacobians.get(robotSide);

         String bodyName = foot.getRigidBody().getName();
         String sideString = robotSide.getCamelCaseNameForStartOfExpression();

         final ConstantConfigurationProvider currentConfigurationProvider = new ConstantConfigurationProvider(new FramePose(foot.getBodyFrame()));
         final ChangeableConfigurationProvider desiredConfigurationProvider = new ChangeableConfigurationProvider(new FramePose(foot.getBodyFrame()));

         StraightLinePositionTrajectoryGenerator positionTrajectoryGenerator = new StraightLinePositionTrajectoryGenerator(bodyName, worldFrame,
                                                                                  trajectoryTimeProvider, currentConfigurationProvider,
                                                                                  desiredConfigurationProvider, registry);

         OrientationInterpolationTrajectoryGenerator orientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator(bodyName, worldFrame,
                                                                                         trajectoryTimeProvider, currentConfigurationProvider,
                                                                                         desiredConfigurationProvider, registry);

         desiredFootConfigurationProviders.put(foot, desiredConfigurationProvider);
         swingPositionTrajectoryGenerators.put(foot, positionTrajectoryGenerator);
         swingOrientationTrajectoryGenerators.put(foot, orientationTrajectoryGenerator);

//       DoubleTrajectoryGenerator onToesTrajectory = createDummyDoubleTrajectoryGenerator();

         DoubleProvider onToesInitialPitchProvider = new ConstantDoubleProvider(0.0);
         DoubleProvider onToesInitialPitchVelocityProvider = new ConstantDoubleProvider(0.0);
         DoubleProvider onToesFinalPitchProvider = new ConstantDoubleProvider(Math.PI / 4.0);

         DoubleTrajectoryGenerator onToesTrajectory = new ThirdOrderPolynomialTrajectoryGenerator(sideString + bodyName, onToesInitialPitchProvider,
                                                         onToesInitialPitchVelocityProvider, onToesFinalPitchProvider, trajectoryTimeProvider, registry);


         EndEffectorControlModule endEffectorControlModule = new EndEffectorControlModule(foot, jacobian, positionTrajectoryGenerator, null,
                                                                orientationTrajectoryGenerator, onToesTrajectory, momentumBasedController, registry);
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
      jacobianForChestOrientationControl = chestOrientationManager.createJacobian(fullRobotModel, baseForChestOrientationControl, chestOrientationControlJointNames);
      
      baseForHeadOrientationControl = fullRobotModel.getPelvis();
      HeadOrientationManager headOrientationManager = variousWalkingManagers.getHeadOrientationManager();
      String[] headOrientationControlJointNames = walkingControllerParameters.getDefaultHeadOrientationControlJointNames();
      jacobianForHeadOrientationControl = headOrientationManager.createJacobian(fullRobotModel, baseForHeadOrientationControl, headOrientationControlJointNames);
   }
   
   public void initialize()
   {
      super.initialize();

      momentumBasedController.setMomentumControlModuleToUse(MOMENTUM_CONTROL_MODULE_TO_USE);
      
      ChestOrientationManager chestOrientationManager = variousWalkingManagers.getChestOrientationManager();
      double chestKp = 100.0;
      double chestZeta = 1.0;
      double chestKd = GainCalculator.computeDerivativeGain(chestKp, chestZeta);
      chestOrientationManager.setUp(baseForChestOrientationControl, jacobianForChestOrientationControl, chestKp, chestKp, chestKp, chestKd, chestKd, chestKd);
      
      HeadOrientationManager headOrientationManager = variousWalkingManagers.getHeadOrientationManager();
      double headKp = 40.0;
      double headZeta = 1.0;
      double headKd = GainCalculator.computeDerivativeGain(headKp, headZeta);
      headOrientationManager.setUp(baseForHeadOrientationControl, jacobianForHeadOrientationControl, headKp, headKp, headKp, headKd, headKd, headKd);
      
      initializeContacts();

      FramePose currentPelvisPose = new FramePose(pelvisPositionControlFrame);
      desiredPelvisConfigurationProvider.set(currentPelvisPose);

      pelvisTrajectoryStartTime = yoTime.getDoubleValue();
      pelvisPositionTrajectoryGenerator.initialize();
      pelvisOrientationTrajectoryGenerator.initialize();

      FrameOrientation currentChestOrientation = new FrameOrientation(chestPositionControlFrame);
      currentChestOrientation.changeFrame(worldFrame);
      desiredChestOrientation.setOrientation(currentChestOrientation);

      chestTrajectoryStartTime = yoTime.getDoubleValue();
      chestOrientationTrajectoryGenerator.initialize();
   }

   private void initializeContacts()
   {
      requestedPelvisLoadBearing.set(isContactablePlaneBodyInContact(contactablePelvis));
      requestedPelvisBackLoadBearing.set(isContactablePlaneBodyInContact(contactablePelvisBack));
      
      for (RobotSide robotSide : RobotSide.values)
      {
         requestedFootLoadBearing.get(robotSide).set(isContactablePlaneBodyInContact(feet.get(robotSide)));
         requestedThighLoadBearing.get(robotSide).set(isContactablePlaneBodyInContact(contactableThighs.get(robotSide)));
      }
   }

   public void doMotionControl()
   {
      momentumBasedController.doPrioritaryControl();
      callUpdatables();
      updateLoadBearingStates();

      doContactPointControl();
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
      if (requestedPelvisLoadBearing.getBooleanValue())
         return;
      
      if (pelvisPoseProvider.checkForNewPose())
      {
         desiredPelvisConfigurationProvider.set(pelvisPoseProvider.getDesiredPelvisPose());
         pelvisTrajectoryStartTime = yoTime.getDoubleValue();

         pelvisPositionTrajectoryGenerator.initialize();
         pelvisOrientationTrajectoryGenerator.initialize();
      }

      pelvisPositionTrajectoryGenerator.compute(yoTime.getDoubleValue() - pelvisTrajectoryStartTime);
      pelvisOrientationTrajectoryGenerator.compute(yoTime.getDoubleValue() - pelvisTrajectoryStartTime);

      FramePoint desiredPosition = new FramePoint(pelvisPositionControlFrame);
      FrameVector desiredVelocity = new FrameVector(pelvisPositionControlFrame);
      FrameVector desiredPelvisAcceleration = new FrameVector(pelvisPositionControlFrame);

      pelvisPositionTrajectoryGenerator.get(desiredPosition);
      pelvisPositionTrajectoryGenerator.packVelocity(desiredVelocity);
      pelvisPositionTrajectoryGenerator.packAcceleration(desiredPelvisAcceleration);

      FrameOrientation desiredOrientation = new FrameOrientation(pelvisPositionControlFrame);
      FrameVector desiredAngularVelocity = new FrameVector(pelvisPositionControlFrame);
      FrameVector desiredAngularAcceleration = new FrameVector(pelvisPositionControlFrame);

      pelvisOrientationTrajectoryGenerator.get(desiredOrientation);
      pelvisOrientationTrajectoryGenerator.packAngularVelocity(desiredAngularVelocity);
      pelvisOrientationTrajectoryGenerator.packAngularAcceleration(desiredAngularAcceleration);

      pelvisController.doPositionControl(desiredPosition, desiredOrientation, desiredVelocity, desiredAngularVelocity, desiredPelvisAcceleration,
                                         desiredAngularAcceleration, fullRobotModel.getElevator());

      SpatialAccelerationVector pelvisSpatialAcceleration = new SpatialAccelerationVector();
      pelvisController.packAcceleration(pelvisSpatialAcceleration);

      pelvisSpatialAcceleration.changeBodyFrameNoRelativeAcceleration(fullRobotModel.getPelvis().getBodyFixedFrame());
      pelvisSpatialAcceleration.changeFrameNoRelativeMotion(fullRobotModel.getPelvis().getBodyFixedFrame());

      pelvisTaskspaceConstraintData.set(pelvisSpatialAcceleration);
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
         desiredChestOrientation.setOrientation(chestOrientationProvider.getDesiredChestOrientation());
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

         if (footPoseProvider.checkForNewPose(robotSide))
         {
            FramePose newFootPose = footPoseProvider.getDesiredFootPose(robotSide);
            desiredFootConfigurationProviders.get(foot).set(newFootPose);
            footEndEffectorControlModules.get(foot).resetCurrentState();
         }

         EndEffectorControlModule endEffectorControlModule = footEndEffectorControlModules.get(foot);
         FramePoint2d cop = momentumBasedController.getCoP(foot);
         endEffectorControlModule.setCenterOfPressure(cop);
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
         for (FramePoint contactPoint : body.getContactPoints())
         {
            momentumBasedController.setDesiredPointAcceleration(jacobian, contactPoint, desiredAcceleration);
         }
      }
   }

   private void addBodyInContact(ContactablePlaneBody contactablePlaneBody)
   {
      bodiesInContact.add(contactablePlaneBody);
      RigidBody rigidBody = contactablePlaneBody.getRigidBody();
      contactJacobians.put(contactablePlaneBody, new GeometricJacobian(elevator, rigidBody, elevator.getBodyFixedFrame()));
   }

   private void removeBodyInContact(ContactablePlaneBody contactablePlaneBody)
   {
      bodiesInContact.remove(contactablePlaneBody);
      contactJacobians.remove(contactablePlaneBody);
   }
   
   private void updateLoadBearingStates()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (footPoseProvider.checkForNewPose(robotSide))
         {
            requestedFootLoadBearing.get(robotSide).set(false);
         }
         
         if (footLoadBearingProvider.checkForNewLoadBearingRequest(robotSide))
         {
            requestedFootLoadBearing.get(robotSide).set(true);
         }
         
         if (thighLoadBearingProvider.checkForNewLoadBearingState(robotSide))
         {
            requestedThighLoadBearing.get(robotSide).set(thighLoadBearingProvider.getDesiredThighLoadBearingState(robotSide));
         }
      }

      if (pelvisLoadBearingProvider.checkForNewLoadBearingRequest())
      {
         if (requestedPelvisLoadBearing.getBooleanValue() && requestedPelvisBackLoadBearing.getBooleanValue())
            requestedPelvisBackLoadBearing.set(false);
         else if (requestedPelvisLoadBearing.getBooleanValue())
            requestedPelvisBackLoadBearing.set(true);
         else
            requestedPelvisLoadBearing.set(true);
      }
      
      if (pelvisPoseProvider.checkForNewPose())
      {
         requestedPelvisLoadBearing.set(false);
         requestedPelvisBackLoadBearing.set(false);
      }
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
         momentumBasedController.setPlaneContactState(handPalm, handPalm.getContactPoints2d(), coefficientOfFriction.getDoubleValue(), normalContactVector);
      }
      else
      {
         FrameVector normalContactVector = null;
         momentumBasedController.setPlaneContactState(handPalm, new ArrayList<FramePoint2d>(), coefficientOfFriction.getDoubleValue(), normalContactVector);
      }
   }

   public void setThighInContact(RobotSide robotSide, boolean inContact)
   {
      ContactablePlaneBody thigh = contactableThighs.get(robotSide);
      if (inContact)
      {
         momentumBasedController.setPlaneContactState(thigh, thigh.getContactPoints2d(), coefficientOfFriction.getDoubleValue(), null);
         addBodyInContact(thigh);
      }
      else
      {
         momentumBasedController.setPlaneContactState(thigh, new ArrayList<FramePoint2d>(), coefficientOfFriction.getDoubleValue(), null);
         removeBodyInContact(thigh);
      }
   }

   public void setPelvisInContact(boolean inContact)
   {
      if (inContact)
      {
         momentumBasedController.setPlaneContactState(contactablePelvis, contactablePelvis.getContactPoints2d(), coefficientOfFriction.getDoubleValue(), null);
         addBodyInContact(contactablePelvis);
      }
      else
      {
         momentumBasedController.setPlaneContactState(contactablePelvis, new ArrayList<FramePoint2d>(), coefficientOfFriction.getDoubleValue(), null);
         removeBodyInContact(contactablePelvis);
      }
   }

   public void setPelvisBackInContact(boolean inContact)
   {
      if (inContact)
      {
         momentumBasedController.setPlaneContactState(contactablePelvisBack, contactablePelvisBack.getContactPoints2d(), coefficientOfFriction.getDoubleValue(), null);
         addBodyInContact(contactablePelvisBack);
      }
      else
      {
         momentumBasedController.setPlaneContactState(contactablePelvisBack, new ArrayList<FramePoint2d>(), coefficientOfFriction.getDoubleValue(), null);
         removeBodyInContact(contactablePelvisBack);
      }
   }

   private void setOnToesContactState(ContactablePlaneBody contactableBody)
   {
      FrameVector normalContactVector = new FrameVector(worldFrame, 0.0, 0.0, 1.0);
      List<FramePoint> contactPoints = getContactPointsAccordingToFootConstraint(contactableBody, ConstraintType.TOES);
      List<FramePoint2d> contactPoints2d = getContactPoints2d(contactableBody, contactPoints);
      setFootContactState(contactableBody, contactPoints2d, ConstraintType.TOES, normalContactVector);
   }

   private void setFlatFootContactState(ContactablePlaneBody contactableBody)
   {
      FrameVector normalContactVector = new FrameVector(contactableBody.getPlaneFrame(), 0.0, 0.0, 1.0);
      setFootContactState(contactableBody, contactableBody.getContactPoints2d(), ConstraintType.FULL, normalContactVector);
   }

   private void setContactStateForSwing(ContactablePlaneBody contactableBody)
   {
      // Initialize desired foot pose to the actual, so no surprising behavior
      ReferenceFrame footFrame = footEndEffectorControlModules.get(contactableBody).getEndEffectorFrame();
      desiredFootConfigurationProviders.get(contactableBody).set(new FramePose(footFrame));

      FrameVector normalContactVector = new FrameVector(contactableBody.getPlaneFrame(), 0.0, 0.0, 1.0);
      setFootContactState(contactableBody, new ArrayList<FramePoint2d>(), ConstraintType.UNCONSTRAINED, normalContactVector);
   }

   private List<FramePoint> getContactPointsAccordingToFootConstraint(ContactablePlaneBody contactableBody, ConstraintType constraintType)
   {
      FrameVector direction = new FrameVector(contactableBody.getBodyFrame(), 1.0, 0.0, 0.0);
      if (constraintType == ConstraintType.HEEL)
         direction.scale(-1.0);

      return DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(contactableBody.getContactPoints(), direction, 2);
   }

   private List<FramePoint2d> getContactPoints2d(ContactablePlaneBody contactableBody, List<FramePoint> contactPoints)
   {
      List<FramePoint2d> contactPoints2d = new ArrayList<FramePoint2d>(contactPoints.size());
      for (FramePoint contactPoint : contactPoints)
      {
         contactPoint.changeFrame(contactableBody.getPlaneFrame());
         contactPoints2d.add(contactPoint.toFramePoint2d());
      }

      return contactPoints2d;
   }

   private void setFootContactState(ContactablePlaneBody contactableBody, List<FramePoint2d> contactPoints, ConstraintType constraintType,
                                FrameVector normalContactVector)
   {
      if (contactPoints.size() == 0)
      {
         footEndEffectorControlModules.get(contactableBody).doSingularityEscapeBeforeTransitionToNextState();
      }

      momentumBasedController.setPlaneContactState(contactableBody, contactPoints, coefficientOfFriction.getDoubleValue(), normalContactVector);

      updateFootEndEffectorControlModule(contactableBody, contactPoints, constraintType);
   }

   private void updateFootEndEffectorControlModule(ContactablePlaneBody contactablePlaneBody, List<FramePoint2d> contactPoints, ConstraintType constraintType)
   {
      footEndEffectorControlModules.get(contactablePlaneBody).setContactPoints(contactPoints, constraintType);
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
