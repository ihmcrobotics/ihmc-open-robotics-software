package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import javax.vecmath.Point2d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactPointVisualizer;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.CenterOfPressureResolver;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumControlModuleBridge.MomentumControlModuleType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredJointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredPointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredRateOfChangeOfMomentumCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredSpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumModuleSolution;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumControlModuleException;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.OptimizationMomentumControlModule;
import us.ihmc.commonWalkingControlModules.sensors.ProvidedMassMatrixToolRigidBody;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.WrenchBasedFootSwitch;
import us.ihmc.commonWalkingControlModules.visualizer.WrenchVisualizer;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.YoPDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector2d;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.RateLimitedYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculator;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculatorListener;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.screwTheory.TotalWrenchCalculator;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.sensorProcessing.model.RobotMotionStatusChangedListener;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureListener;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerStateChangedListener;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;


public class MomentumBasedController
{
   private static final boolean DO_PASSIVE_KNEE_CONTROL = true;
   private static final boolean VISUALIZE_ANTI_GRAVITY_JOINT_TORQUES = false;

   public static final boolean SPY_ON_MOMENTUM_BASED_CONTROLLER = false;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final DesiredSpatialAccelerationCommandPool desiredSpatialAccelerationCommandPool = new DesiredSpatialAccelerationCommandPool();
   
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final ReferenceFrame centerOfMassFrame;
   private final FullHumanoidRobotModel fullRobotModel;
   private final CenterOfMassJacobian centerOfMassJacobian;
   private final CommonHumanoidReferenceFrames referenceFrames;
   private final TwistCalculator twistCalculator;

   private final SideDependentList<ContactablePlaneBody> feet, hands, thighs;
   private final ContactablePlaneBody pelvis, pelvisBack;

   private final List<ContactablePlaneBody> contactablePlaneBodyList;
   private final List<YoPlaneContactState> yoPlaneContactStateList = new ArrayList<YoPlaneContactState>();
   private final LinkedHashMap<ContactablePlaneBody, YoPlaneContactState> yoPlaneContactStates = new LinkedHashMap<ContactablePlaneBody, YoPlaneContactState>();

   private final DoubleYoVariable leftPassiveKneeTorque = new DoubleYoVariable("leftPassiveKneeTorque", registry);
   private final DoubleYoVariable rightPassiveKneeTorque = new DoubleYoVariable("rightPassiveKneeTorque", registry);
   private final SideDependentList<DoubleYoVariable> passiveKneeTorque = new SideDependentList<DoubleYoVariable>(leftPassiveKneeTorque, rightPassiveKneeTorque);

   private final DoubleYoVariable passiveQKneeThreshold = new DoubleYoVariable("passiveQKneeThreshold", registry);
   private final DoubleYoVariable passiveKneeMaxTorque = new DoubleYoVariable("passiveKneeMaxTorque", registry);
   private final DoubleYoVariable passiveKneeKv = new DoubleYoVariable("passiveKneeKv", registry);

   private final ArrayList<Updatable> updatables = new ArrayList<Updatable>();
   private final DoubleYoVariable yoTime;
   private final double controlDT;
   private final double gravity;

   private final SideDependentList<CenterOfMassReferenceFrame> handCenterOfMassFrames;
   private final SideDependentList<YoFrameVector> wristRawMeasuredForces;
   private final SideDependentList<YoFrameVector> wristRawMeasuredTorques;
   private final SideDependentList<YoFrameVector> wristForcesHandWeightCancelled;
   private final SideDependentList<YoFrameVector> wristTorquesHandWeightCancelled;
   private final SideDependentList<ReferenceFrame> wristForceSensorMeasurementFrames;
   private final Wrench wristWrenchDueToGravity = new Wrench();
   private final Wrench wristTempWrench = new Wrench();
   private final FrameVector tempWristForce = new FrameVector();
   private final FrameVector tempWristTorque = new FrameVector();

   private final SideDependentList<DoubleYoVariable> handsMass;

   private final FrameVector centroidalMomentumRateSolutionLinearPart;
   private final YoFrameVector qpOutputCoMAcceleration;

   private final YoFrameVector admissibleDesiredGroundReactionTorque;
   private final YoFrameVector admissibleDesiredGroundReactionForce;
   private final YoFramePoint2d admissibleDesiredCenterOfPressure;

   // private final YoFrameVector groundReactionTorqueCheck;
   // private final YoFrameVector groundReactionForceCheck;

   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> preRateLimitedDesiredAccelerations = new LinkedHashMap<OneDoFJoint, DoubleYoVariable>();
   private final LinkedHashMap<OneDoFJoint, RateLimitedYoVariable> rateLimitedDesiredAccelerations = new LinkedHashMap<OneDoFJoint, RateLimitedYoVariable>();

   private final ArrayList<OneDoFJoint> jointsWithDesiredAcceleration = new ArrayList<>();
   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> desiredAccelerationYoVariables = new LinkedHashMap<OneDoFJoint, DoubleYoVariable>();
   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> desiredTorqueYoVariables = new LinkedHashMap<OneDoFJoint, DoubleYoVariable>();

   private final InverseDynamicsCalculator inverseDynamicsCalculator;

   private final OptimizationMomentumControlModule optimizationMomentumControlModule;
   private final MomentumControlModuleBridge momentumControlModuleBridge;

   @Deprecated
   private final EnumYoVariable<RobotSide> upcomingSupportLeg = EnumYoVariable.create("upcomingSupportLeg", "", RobotSide.class, registry, true);    // FIXME: not general enough; this should not be here

   private final PlaneContactWrenchProcessor planeContactWrenchProcessor;
   private final MomentumBasedControllerSpy momentumBasedControllerSpy;
   private final ContactPointVisualizer contactPointVisualizer;
   private final WrenchVisualizer wrenchVisualizer;

   private final GeometricJacobianHolder robotJacobianHolder = new GeometricJacobianHolder();

   private final SideDependentList<FootSwitchInterface> footSwitches;
   private final SideDependentList<ForceSensorDataReadOnly> wristForceSensors;
   private final DoubleYoVariable alphaCoPControl = new DoubleYoVariable("alphaCoPControl", registry);
   private final DoubleYoVariable maxAnkleTorqueCoPControl = new DoubleYoVariable("maxAnkleTorqueCoPControl", registry);
   private final SideDependentList<AlphaFilteredYoFrameVector2d> desiredTorquesForCoPControl;


   private final SideDependentList<Double> xSignsForCoPControl, ySignsForCoPControl;
   private final double minZForceForCoPControlScaling, maxZForceForCoPControlScaling;

   private final SideDependentList<YoFrameVector2d> yoCoPError;
   private final SideDependentList<DoubleYoVariable> yoCoPErrorMagnitude =
      new SideDependentList<DoubleYoVariable>(new DoubleYoVariable("leftFootCoPErrorMagnitude", registry),
                            new DoubleYoVariable("rightFootCoPErrorMagnitude", registry));
   private final DoubleYoVariable gainCoPX = new DoubleYoVariable("gainCoPX", registry);
   private final DoubleYoVariable gainCoPY = new DoubleYoVariable("gainCoPY", registry);
   private final SideDependentList<DoubleYoVariable> copControlScales;

   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private final InverseDynamicsJoint[] controlledJoints;

   private final BooleanYoVariable feetCoPControlIsActive;
   private final DoubleYoVariable footCoPOffsetX, footCoPOffsetY;
   private final EnumYoVariable<RobotSide> footUnderCoPControl;

   private final BooleanYoVariable userActivateFeetForce = new BooleanYoVariable("userActivateFeetForce", registry);
   private final DoubleYoVariable userLateralFeetForce = new DoubleYoVariable("userLateralFeetForce", registry);
   private final DoubleYoVariable userForwardFeetForce = new DoubleYoVariable("userForwardFeetForce", registry);
   private final DoubleYoVariable userYawFeetTorque = new DoubleYoVariable("userYawFeetTorque", registry);
   private final Wrench userAdditionalFeetWrench = new Wrench();

   private final YoFrameVector residualRootJointForce;
   private final YoFrameVector residualRootJointTorque;

   private final YoFrameVector yoRootJointDesiredAngularAcceleration;
   private final YoFrameVector yoRootJointDesiredLinearAcceleration;

   private final YoFrameVector yoRootJointDesiredAngularAccelerationWorld;
   private final YoFrameVector yoRootJointDesiredLinearAccelerationWorld;
   private final YoFrameVector yoRootJointAngularVelocity;
   private final YoFrameVector yoRootJointLinearVelocity;

   private final SideDependentList<YoFrameVector> yoFeetDesiredLinearAccelerations;
   private final SideDependentList<YoFrameVector> yoFeetDesiredAngularAccelerations;

   private final SpatialAccelerationVector tempAcceleration = new SpatialAccelerationVector();
   private final SideDependentList<Wrench> handWrenches = new SideDependentList<>();
   private final SideDependentList<ProvidedMassMatrixToolRigidBody> toolRigidBodies = new SideDependentList<>();

   private final AntiGravityJointTorquesVisualizer antiGravityJointTorquesVisualizer;

   private final double totalMass;

   private final ArrayList<ControllerFailureListener> controllerFailureListeners = new ArrayList<>();
   private final ArrayList<ControllerStateChangedListener> controllerStateChangedListeners = new ArrayList<>();
   private final ArrayList<RobotMotionStatusChangedListener> robotMotionStatusChangedListeners = new ArrayList<>();

   public MomentumBasedController(FullHumanoidRobotModel fullRobotModel, CenterOfMassJacobian centerOfMassJacobian, CommonHumanoidReferenceFrames referenceFrames,
                                  SideDependentList<FootSwitchInterface> footSwitches, SideDependentList<ForceSensorDataReadOnly> wristForceSensors, DoubleYoVariable yoTime, double gravityZ,
                                  TwistCalculator twistCalculator, SideDependentList<ContactablePlaneBody> feet,
                                  SideDependentList<ContactablePlaneBody> handsWithFingersBentBack, SideDependentList<ContactablePlaneBody> thighs,
                                  ContactablePlaneBody pelvis, ContactablePlaneBody pelvisBack, double controlDT,
                                  OldMomentumControlModule oldMomentumControlModule, ArrayList<Updatable> updatables,
                                  ArmControllerParameters armControllerParameters, WalkingControllerParameters walkingControllerParameters,
                                  YoGraphicsListRegistry yoGraphicsListRegistry, InverseDynamicsJoint... jointsToIgnore)
   {
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;

      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
      totalMass = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator());

      this.footSwitches = footSwitches;
      this.wristForceSensors = wristForceSensors;

      if (SPY_ON_MOMENTUM_BASED_CONTROLLER)
         momentumBasedControllerSpy = new MomentumBasedControllerSpy(registry);
      else
         momentumBasedControllerSpy = null;

      MathTools.checkIfInRange(gravityZ, 0.0, Double.POSITIVE_INFINITY);

      this.fullRobotModel = fullRobotModel;
      this.centerOfMassJacobian = centerOfMassJacobian;
      this.referenceFrames = referenceFrames;
      this.twistCalculator = twistCalculator;
      this.controlDT = controlDT;
      this.gravity = gravityZ;
      this.yoTime = yoTime;

      // Initialize the contactable bodies
      this.feet = feet;
      this.hands = handsWithFingersBentBack;
      this.thighs = thighs;
      this.pelvis = pelvis;
      this.pelvisBack = pelvisBack;

      RigidBody elevator = fullRobotModel.getElevator();

      inverseDynamicsCalculator = new InverseDynamicsCalculator(twistCalculator, gravityZ);

      if (VISUALIZE_ANTI_GRAVITY_JOINT_TORQUES)
      {
         SideDependentList<WrenchBasedFootSwitch> wrenchBasedFootSwitches = new SideDependentList<>();
         for (RobotSide robotSide : RobotSide.values)
         {
            wrenchBasedFootSwitches.put(robotSide, (WrenchBasedFootSwitch) footSwitches.get(robotSide));
         }

         antiGravityJointTorquesVisualizer = new AntiGravityJointTorquesVisualizer(fullRobotModel, twistCalculator, wrenchBasedFootSwitches, registry,
                 gravityZ);
      }
      else
         antiGravityJointTorquesVisualizer = null;

      double totalMass = TotalMassCalculator.computeSubTreeMass(elevator);

      this.residualRootJointForce = new YoFrameVector("residualRootJointForce", "", centerOfMassFrame, registry);
      this.residualRootJointTorque = new YoFrameVector("residualRootJointTorque", "", centerOfMassFrame, registry);

      centroidalMomentumRateSolutionLinearPart = new FrameVector(centerOfMassFrame);
      this.qpOutputCoMAcceleration = new YoFrameVector("qpOutputCoMAcceleration", "", centerOfMassFrame, registry);

      this.admissibleDesiredGroundReactionTorque = new YoFrameVector("admissibleDesiredGroundReactionTorque", centerOfMassFrame, registry);
      this.admissibleDesiredGroundReactionForce = new YoFrameVector("admissibleDesiredGroundReactionForce", centerOfMassFrame, registry);
      this.admissibleDesiredCenterOfPressure = new YoFramePoint2d("admissibleDesiredCenterOfPressure", worldFrame, registry);
      admissibleDesiredCenterOfPressure.setToNaN();
      yoGraphicsListRegistry.registerArtifact("Desired Center of Pressure", new YoGraphicPosition("Desired Overall Center of Pressure", admissibleDesiredCenterOfPressure, 0.008, YoAppearance.Navy(), GraphicType.BALL).createArtifact());

      // this.groundReactionTorqueCheck = new YoFrameVector("groundReactionTorqueCheck", centerOfMassFrame, registry);
      // this.groundReactionForceCheck = new YoFrameVector("groundReactionForceCheck", centerOfMassFrame, registry);

      if (updatables != null)
      {
         this.updatables.addAll(updatables);
      }

      double coefficientOfFriction = 1.0;    // TODO: magic number...

      // TODO: get rid of the null checks
      this.contactablePlaneBodyList = new ArrayList<ContactablePlaneBody>();

      if (feet != null)
      {
         this.contactablePlaneBodyList.addAll(feet.values());    // leftSole and rightSole
      }

      if (handsWithFingersBentBack != null)
      {
         this.contactablePlaneBodyList.addAll(handsWithFingersBentBack.values());
      }

      if (thighs != null)
      {
         this.contactablePlaneBodyList.addAll(thighs.values());
      }

      if (pelvis != null)
         this.contactablePlaneBodyList.add(pelvis);

      if (pelvisBack != null)
         this.contactablePlaneBodyList.add(pelvisBack);

      for (ContactablePlaneBody contactablePlaneBody : this.contactablePlaneBodyList)
      {
         RigidBody rigidBody = contactablePlaneBody.getRigidBody();
         YoPlaneContactState contactState = new YoPlaneContactState(contactablePlaneBody.getSoleFrame().getName(), rigidBody,
                                               contactablePlaneBody.getSoleFrame(), contactablePlaneBody.getContactPoints2d(), coefficientOfFriction, registry);
         yoPlaneContactStates.put(contactablePlaneBody, contactState);
         yoPlaneContactStateList.add(contactState);
      }

      InverseDynamicsJoint[] jointsToOptimizeFor = HighLevelHumanoidControllerFactoryHelper.computeJointsToOptimizeFor(fullRobotModel, jointsToIgnore);
      MomentumOptimizationSettings momentumOptimizationSettings = new MomentumOptimizationSettings(jointsToOptimizeFor, registry);
      walkingControllerParameters.setupMomentumOptimizationSettings(momentumOptimizationSettings);

      controlledJoints = momentumOptimizationSettings.getJointsToOptimizeFor();

      for (InverseDynamicsJoint joint : controlledJoints)
      {
         if (joint instanceof OneDoFJoint)
         {
            jointsWithDesiredAcceleration.add((OneDoFJoint) joint);
            desiredAccelerationYoVariables.put((OneDoFJoint) joint, new DoubleYoVariable(joint.getName() + "qdd_d", registry));
            desiredTorqueYoVariables.put((OneDoFJoint) joint, new DoubleYoVariable(joint.getName() + "tau_d", registry));
            rateLimitedDesiredAccelerations.put((OneDoFJoint) joint, new RateLimitedYoVariable(joint.getName() + "_rl_qdd_d", registry, 10000.0, controlDT));
            preRateLimitedDesiredAccelerations.put((OneDoFJoint) joint, new DoubleYoVariable(joint.getName() + "_prl_qdd_d", registry));
         }
      }

      this.planeContactWrenchProcessor = new PlaneContactWrenchProcessor(this.contactablePlaneBodyList, yoGraphicsListRegistry, registry);

      if (yoGraphicsListRegistry != null)
      {
         contactPointVisualizer = new ContactPointVisualizer(new ArrayList<YoPlaneContactState>(yoPlaneContactStateList), yoGraphicsListRegistry, registry);

         // TODO: Don't need for all of the bodies. Just the ones that might be in contact.
         List<RigidBody> rigidBodies = Arrays.asList(ScrewTools.computeSupportAndSubtreeSuccessors(fullRobotModel.getRootJoint().getSuccessor()));
         wrenchVisualizer = new WrenchVisualizer("DesiredExternalWrench", rigidBodies, 1.0, yoGraphicsListRegistry, registry);
      }
      else
      {
         contactPointVisualizer = null;
         wrenchVisualizer = null;
      }

      optimizationMomentumControlModule = new OptimizationMomentumControlModule(fullRobotModel.getRootJoint(),
            referenceFrames.getCenterOfMassFrame(), controlDT, gravityZ, momentumOptimizationSettings, twistCalculator, robotJacobianHolder,
            yoPlaneContactStateList, yoGraphicsListRegistry, registry);

      momentumControlModuleBridge = new MomentumControlModuleBridge(optimizationMomentumControlModule, oldMomentumControlModule, centerOfMassFrame, registry);

      if (DO_PASSIVE_KNEE_CONTROL)
      {
         passiveQKneeThreshold.set(0.55);
         passiveKneeMaxTorque.set(60.0);
         passiveKneeKv.set(5.0);
      }

      desiredTorquesForCoPControl = new SideDependentList<AlphaFilteredYoFrameVector2d>();
      yoCoPError = new SideDependentList<YoFrameVector2d>();
      xSignsForCoPControl = new SideDependentList<Double>();
      ySignsForCoPControl = new SideDependentList<Double>();
      copControlScales = new SideDependentList<DoubleYoVariable>();

      for (RobotSide robotSide : RobotSide.values())
      {
         OneDoFJoint anklePitchJoint = fullRobotModel.getLegJoint(robotSide, LegJointName.ANKLE_PITCH);
         OneDoFJoint ankleRollJoint = fullRobotModel.getLegJoint(robotSide, LegJointName.ANKLE_ROLL);

         FrameVector pitchJointAxis = anklePitchJoint.getJointAxis();
         FrameVector rollJointAxis = ankleRollJoint.getJointAxis();

         xSignsForCoPControl.put(robotSide, pitchJointAxis.getY());
         ySignsForCoPControl.put(robotSide, rollJointAxis.getY());

         copControlScales.put(robotSide, new DoubleYoVariable(robotSide.getCamelCaseNameForStartOfExpression() + "CoPControlScale", registry));
      }

      minZForceForCoPControlScaling = 0.20 * totalMass * 9.81;
      maxZForceForCoPControlScaling = 0.45 * totalMass * 9.81;

      alphaCoPControl.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(16.0, controlDT));
      maxAnkleTorqueCoPControl.set(10.0);

      for (RobotSide robotSide : RobotSide.values)
      {
         desiredTorquesForCoPControl.put(robotSide,
                                         AlphaFilteredYoFrameVector2d.createAlphaFilteredYoFrameVector2d("desired"
                                            + robotSide.getCamelCaseNameForMiddleOfExpression() + "AnkleTorqueForCoPControl", "", registry, alphaCoPControl,
                                               feet.get(robotSide).getSoleFrame()));
         yoCoPError.put(robotSide,
                        new YoFrameVector2d(robotSide.getCamelCaseNameForStartOfExpression() + "FootCoPError", feet.get(robotSide).getSoleFrame(), registry));

         RigidBody hand = fullRobotModel.getHand(robotSide);
         if (hand != null)
         {
            toolRigidBodies.put(robotSide,
                                new ProvidedMassMatrixToolRigidBody(robotSide, getFullRobotModel(), gravityZ, armControllerParameters, registry,
                                   yoGraphicsListRegistry));
            handWrenches.put(robotSide, new Wrench());
         }
      }

      feetCoPControlIsActive = new BooleanYoVariable("feetCoPControlIsActive", registry);
      footCoPOffsetX = new DoubleYoVariable("footCoPOffsetX", registry);
      footCoPOffsetY = new DoubleYoVariable("footCoPOffsetY", registry);
      footUnderCoPControl = new EnumYoVariable<RobotSide>("footUnderCoPControl", registry, RobotSide.class);

      yoRootJointDesiredAngularAcceleration = new YoFrameVector("rootJointDesiredAngularAcceleration", fullRobotModel.getRootJoint().getFrameAfterJoint(),
              registry);
      yoRootJointDesiredLinearAcceleration = new YoFrameVector("rootJointDesiredLinearAcceleration", fullRobotModel.getRootJoint().getFrameAfterJoint(),
              registry);

      yoRootJointDesiredAngularAccelerationWorld = new YoFrameVector("rootJointDesiredAngularAccelerationWorld", ReferenceFrame.getWorldFrame(), registry);
      yoRootJointDesiredLinearAccelerationWorld = new YoFrameVector("rootJointDesiredLinearAccelerationWorld", ReferenceFrame.getWorldFrame(), registry);

      yoRootJointAngularVelocity = new YoFrameVector("rootJointAngularVelocity", fullRobotModel.getRootJoint().getFrameAfterJoint(), registry);
      yoRootJointLinearVelocity = new YoFrameVector("rootJointLinearVelocity", ReferenceFrame.getWorldFrame(), registry);


      YoFrameVector leftFootDesiredLinearAcceleration = new YoFrameVector("leftFootDesiredLinearAcceleration",
                                                           fullRobotModel.getFoot(RobotSide.LEFT).getBodyFixedFrame(), registry);
      YoFrameVector leftFootDesiredAngularAcceleration = new YoFrameVector("leftFootDesiredAngularAcceleration",
                                                            fullRobotModel.getFoot(RobotSide.LEFT).getBodyFixedFrame(), registry);

      YoFrameVector rightFootDesiredLinearAcceleration = new YoFrameVector("rightFootDesiredLinearAcceleration",
                                                            fullRobotModel.getFoot(RobotSide.RIGHT).getBodyFixedFrame(), registry);
      YoFrameVector rightFootDesiredAngularAcceleration = new YoFrameVector("rightFootDesiredAngularAcceleration",
                                                             fullRobotModel.getFoot(RobotSide.RIGHT).getBodyFixedFrame(), registry);

      yoFeetDesiredLinearAccelerations = new SideDependentList<YoFrameVector>(leftFootDesiredLinearAcceleration, rightFootDesiredLinearAcceleration);
      yoFeetDesiredAngularAccelerations = new SideDependentList<YoFrameVector>(leftFootDesiredAngularAcceleration, rightFootDesiredAngularAcceleration);

      if (wristForceSensors == null)
      {
         wristForceSensorMeasurementFrames = null;
         wristRawMeasuredForces = null;
         wristRawMeasuredTorques = null;
         wristForcesHandWeightCancelled = null;
         wristTorquesHandWeightCancelled = null;
         handCenterOfMassFrames = null;
         handsMass = null;
      }
      else
      {
         wristForceSensorMeasurementFrames = new SideDependentList<>();
         wristRawMeasuredForces = new SideDependentList<>();
         wristRawMeasuredTorques = new SideDependentList<>();
         wristForcesHandWeightCancelled = new SideDependentList<>();
         wristTorquesHandWeightCancelled = new SideDependentList<>();
         handCenterOfMassFrames = new SideDependentList<>();
         handsMass = new SideDependentList<>();

         for (RobotSide robotSide : RobotSide.values)
         {
            ForceSensorDataReadOnly wristForceSensor = wristForceSensors.get(robotSide);
            ReferenceFrame measurementFrame = wristForceSensor.getMeasurementFrame();
            RigidBody measurementLink = wristForceSensor.getMeasurementLink();
            wristForceSensorMeasurementFrames.put(robotSide, measurementFrame);

            String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
            String namePrefix = sidePrefix + "WristSensor";
            wristRawMeasuredForces.put(robotSide, new YoFrameVector(namePrefix + "Force", measurementFrame, registry));
            wristRawMeasuredTorques.put(robotSide, new YoFrameVector(namePrefix + "Torque", measurementFrame, registry));
            wristForcesHandWeightCancelled.put(robotSide, new YoFrameVector(namePrefix + "ForceHandWeightCancelled", measurementFrame, registry));
            wristTorquesHandWeightCancelled.put(robotSide, new YoFrameVector(namePrefix + "TorqueHandWeightCancelled", measurementFrame, registry));

            RigidBody[] handBodies = ScrewTools.computeRigidBodiesAfterThisJoint(measurementLink.getParentJoint());
            CenterOfMassReferenceFrame handCoMFrame = new CenterOfMassReferenceFrame(sidePrefix + "HandCoMFrame", measurementFrame, handBodies);
            handCenterOfMassFrames.put(robotSide, handCoMFrame);
            DoubleYoVariable handMass = new DoubleYoVariable(sidePrefix + "HandTotalMass", registry);
            handsMass.put(robotSide, handMass);
            handMass.set(TotalMassCalculator.computeSubTreeMass(measurementLink));
         }
      }

      enableCoPSmootherForShakies.set(momentumOptimizationSettings.getEnableCoPSmootherControlForShakies());
      defaultWRhoSmoother.set(momentumOptimizationSettings.getRateOfChangeOfRhoPlaneContactRegularization());
      maxWRhoSmoother.set(momentumOptimizationSettings.getMaxWRhoSmoother());
      copSmootherControlDuration.set(momentumOptimizationSettings.getCoPSmootherDuration());
      copSmootherErrorThreshold.set(momentumOptimizationSettings.getCopErrorThresholdToTriggerSmoother());
   }

   public void setInverseDynamicsCalculatorListener(InverseDynamicsCalculatorListener inverseDynamicsCalculatorListener)
   {
      this.inverseDynamicsCalculator.setInverseDynamicsCalculatorListener(inverseDynamicsCalculatorListener);
   }

   public void getFeetContactStates(ArrayList<PlaneContactState> feetContactStatesToPack)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         feetContactStatesToPack.add(yoPlaneContactStates.get(feet.get(robotSide)));
      }
   }

   public void setExternalWrenchToCompensateFor(RigidBody rigidBody, Wrench wrench)
   {
      if (momentumBasedControllerSpy != null)
      {
         momentumBasedControllerSpy.setExternalWrenchToCompensateFor(rigidBody, wrench);
      }

      // momentumControlModuleBridge.getActiveMomentumControlModule().setExternalWrenchToCompensateFor(rigidBody, wrench);
      momentumControlModuleBridge.setExternalWrenchToCompensateFor(rigidBody, wrench);
   }

   // TODO: Temporary method for a big refactor allowing switching between high level behaviors
   public void doPrioritaryControl()
   {
      if (momentumBasedControllerSpy != null)
      {
         momentumBasedControllerSpy.doPrioritaryControl();
      }

      robotJacobianHolder.compute();

      callUpdatables();

      inverseDynamicsCalculator.reset();
      momentumControlModuleBridge.reset();

      readWristSensorData();
   }

   private final Wrench admissibleGroundReactionWrench = new Wrench();
   private final TotalWrenchCalculator totalWrenchCalculator = new TotalWrenchCalculator();

   // TODO: Temporary method for a big refactor allowing switching between high level behaviors
   public void doSecondaryControl()
   {
      if (antiGravityJointTorquesVisualizer != null)
         antiGravityJointTorquesVisualizer.computeAntiGravityJointTorques();

      if (contactPointVisualizer != null)
         contactPointVisualizer.update();

      updateMomentumBasedControllerSpy();

      MomentumModuleSolution momentumModuleSolution;

      if (feetCoPControlIsActive.getBooleanValue())
      {
         if (footCoPOffsetX.isNaN() || footCoPOffsetY.isNaN())
         {
            footCoPOffsetX.set(0.0);
            footCoPOffsetY.set(0.0);
         }

         ReferenceFrame selectedFootSoleFrame = feet.get(footUnderCoPControl.getEnumValue()).getSoleFrame();
         Vector3d copOffset = new Vector3d(footCoPOffsetX.getDoubleValue(), footCoPOffsetY.getDoubleValue(), 0.0);
         String sideName = footUnderCoPControl.getEnumValue().name();
         ReferenceFrame desiredCoPFrame = ReferenceFrame.constructBodyFrameWithUnchangingTranslationFromParent(sideName + "_desiredCoPFrame",
                                             selectedFootSoleFrame, copOffset);

         momentumControlModuleBridge.setFootCoPControlData(footUnderCoPControl.getEnumValue(), desiredCoPFrame);
      }
      else
      {
         footCoPOffsetX.set(Double.NaN);
         footCoPOffsetY.set(Double.NaN);
         momentumControlModuleBridge.setFootCoPControlData(footUnderCoPControl.getEnumValue(), null);
      }

      try
      {
         momentumModuleSolution = momentumControlModuleBridge.compute(this.yoPlaneContactStates, upcomingSupportLeg.getEnumValue());
      }
      catch (MomentumControlModuleException momentumControlModuleException)
      {
         if (momentumBasedControllerSpy != null)
            momentumBasedControllerSpy.printMomentumCommands(System.err);

         // Don't crash and burn. Instead do the best you can with what you have.
         // Or maybe just use the previous ticks solution.
         // Need to test these.
         momentumModuleSolution = momentumControlModuleException.getMomentumModuleSolution();

         // throw new RuntimeException(momentumControlModuleException);
      }

      SpatialForceVector centroidalMomentumRateSolution = momentumModuleSolution.getCentroidalMomentumRateSolution();
      centroidalMomentumRateSolution.packLinearPart(centroidalMomentumRateSolutionLinearPart);
      centroidalMomentumRateSolutionLinearPart.scale(1.0 / totalMass);
      qpOutputCoMAcceleration.set(centroidalMomentumRateSolutionLinearPart);

      Map<RigidBody, Wrench> externalWrenches = momentumModuleSolution.getExternalWrenchSolution();

      if (userActivateFeetForce.getBooleanValue())
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            RigidBody foot = fullRobotModel.getFoot(robotSide);
            userAdditionalFeetWrench.setToZero(foot.getBodyFixedFrame(), referenceFrames.getMidFeetZUpFrame());
            userAdditionalFeetWrench.setLinearPartX(robotSide.negateIfRightSide(userForwardFeetForce.getDoubleValue()));
            userAdditionalFeetWrench.setLinearPartY(robotSide.negateIfRightSide(userLateralFeetForce.getDoubleValue()));
            userAdditionalFeetWrench.setAngularPartZ(robotSide.negateIfRightSide(userYawFeetTorque.getDoubleValue()));

            Wrench externalWrench = externalWrenches.get(foot);
            userAdditionalFeetWrench.changeFrame(externalWrench.getExpressedInFrame());
            externalWrench.add(userAdditionalFeetWrench);
         }
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody hand = fullRobotModel.getHand(robotSide);
         if (hand != null)
         {
            Wrench handWrench = handWrenches.get(robotSide);
            inverseDynamicsCalculator.getSpatialAccelerationCalculator().packAccelerationOfBody(tempAcceleration, hand);
            toolRigidBodies.get(robotSide).control(tempAcceleration, handWrench);

            if (externalWrenches.containsKey(hand))
            {
               externalWrenches.get(hand).add(handWrench);
            }
            else
            {
               externalWrenches.put(hand, handWrench);
            }
         }
      }

      for (RigidBody rigidBody : externalWrenches.keySet())
      {
         inverseDynamicsCalculator.setExternalWrench(rigidBody, externalWrenches.get(rigidBody));
      }

      planeContactWrenchProcessor.compute(externalWrenches);
      if (wrenchVisualizer != null)
         wrenchVisualizer.visualize(externalWrenches);

      totalWrenchCalculator.computeTotalWrench(admissibleGroundReactionWrench, externalWrenches.values(), centerOfMassFrame);
      admissibleDesiredGroundReactionTorque.set(admissibleGroundReactionWrench.getAngularPartX(), admissibleGroundReactionWrench.getAngularPartY(),
              admissibleGroundReactionWrench.getAngularPartZ());
      admissibleDesiredGroundReactionForce.set(admissibleGroundReactionWrench.getLinearPartX(), admissibleGroundReactionWrench.getLinearPartY(),
              admissibleGroundReactionWrench.getLinearPartZ());

      computeDesiredCenterOfPressure();

      // SpatialForceVector groundReactionWrenchCheck = inverseDynamicsCalculator.computeTotalExternalWrench(centerOfMassFrame);
      // groundReactionTorqueCheck.set(groundReactionWrenchCheck.getAngularPartCopy());
      // groundReactionForceCheck.set(groundReactionWrenchCheck.getLinearPartCopy());

      inverseDynamicsCalculator.compute();

      updateYoVariables();
      desiredSpatialAccelerationCommandPool.recycleObjectPool();
   }

   private final FramePoint2d desiredCenterOfPressure = new FramePoint2d();
   private final CenterOfPressureResolver centerOfPressureResolver = new CenterOfPressureResolver();

   private void computeDesiredCenterOfPressure()
   {
      centerOfPressureResolver.resolveCenterOfPressureAndNormalTorque(desiredCenterOfPressure, admissibleGroundReactionWrench, worldFrame);
      admissibleDesiredCenterOfPressure.set(desiredCenterOfPressure);
   }

   // FIXME GET RID OF THAT HACK!!!

   /**
    * Call this method after doSecondaryControl() to generate a small torque of flexion at the knees when almost straight.
    * This helps a lot when working near singularities but it is kinda hackish.
    */
   private final FrameVector tempVector = new FrameVector();

   public void doPassiveKneeControl()
   {
      double maxPassiveTorque = passiveKneeMaxTorque.getDoubleValue();
      double kneeLimit = passiveQKneeThreshold.getDoubleValue();
      double kdKnee = passiveKneeKv.getDoubleValue();

      for (RobotSide robotSide : RobotSide.values)
      {
         OneDoFJoint kneeJoint = fullRobotModel.getLegJoint(robotSide, LegJointName.KNEE);
         kneeJoint.getJointAxis(tempVector);
         double sign = Math.signum(tempVector.getY());
         double tauKnee = kneeJoint.getTau();
         double qKnee = kneeJoint.getQ() * sign;
         double qdKnee = kneeJoint.getQd() * sign;
         if (qKnee < kneeLimit)
         {
            double percent = 1.0 - qKnee / kneeLimit;
            percent = MathTools.clipToMinMax(percent, 0.0, 1.0);
            passiveKneeTorque.get(robotSide).set(sign * maxPassiveTorque * MathTools.square(percent) - kdKnee * qdKnee);
            tauKnee += passiveKneeTorque.get(robotSide).getDoubleValue();
            kneeJoint.setTau(tauKnee);
         }
         else
         {
            passiveKneeTorque.get(robotSide).set(0.0);
         }
      }
   }

   private final FramePoint2d copDesired = new FramePoint2d();
   private final FramePoint2d copActual = new FramePoint2d();
   private final FrameVector2d copError = new FrameVector2d();
   private final Wrench footWrench = new Wrench();
   private final FrameVector footForceVector = new FrameVector();

   public final void doProportionalControlOnCoP()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody contactablePlaneBody = feet.get(robotSide);
         ReferenceFrame planeFrame = contactablePlaneBody.getSoleFrame();
         AlphaFilteredYoFrameVector2d desiredTorqueForCoPControl = desiredTorquesForCoPControl.get(robotSide);

         FramePoint2d cop = planeContactWrenchProcessor.getCoP(contactablePlaneBody);

         if ((cop == null) || cop.containsNaN())
         {
            desiredTorqueForCoPControl.setToZero();

            return;
         }

         copDesired.setIncludingFrame(cop);
         FootSwitchInterface footSwitch = footSwitches.get(robotSide);
         footSwitch.computeAndPackCoP(copActual);

         if (copActual.containsNaN())
         {
            desiredTorqueForCoPControl.setToZero();

            return;
         }

         copError.setToZero(planeFrame);
         copError.sub(copDesired, copActual);
         yoCoPError.get(robotSide).set(copError);
         yoCoPErrorMagnitude.get(robotSide).set(copError.length());

         double xSignForCoPControl = xSignsForCoPControl.get(robotSide);
         double ySignForCoPControl = ySignsForCoPControl.get(robotSide);

         footSwitch.computeAndPackFootWrench(footWrench);
         footForceVector.setToZero(footWrench.getExpressedInFrame());
         footWrench.packLinearPart(footForceVector);
         footForceVector.changeFrame(ReferenceFrame.getWorldFrame());

         double zForce = footForceVector.getZ();
         double scale = (zForce - minZForceForCoPControlScaling) / (maxZForceForCoPControlScaling - minZForceForCoPControlScaling);
         scale = MathTools.clipToMinMax(scale, 0.0, 1.0);

         copControlScales.get(robotSide).set(scale);

         copError.scale(scale * xSignForCoPControl * gainCoPX.getDoubleValue(), scale * ySignForCoPControl * gainCoPY.getDoubleValue());
         copError.clipMaxLength(maxAnkleTorqueCoPControl.getDoubleValue());

         desiredTorqueForCoPControl.update(copError);

         OneDoFJoint anklePitchJoint = fullRobotModel.getLegJoint(robotSide, LegJointName.ANKLE_PITCH);
         anklePitchJoint.setTau(anklePitchJoint.getTau() + desiredTorqueForCoPControl.getX());

         OneDoFJoint ankleRollJoint = fullRobotModel.getLegJoint(robotSide, LegJointName.ANKLE_ROLL);
         ankleRollJoint.setTau(ankleRollJoint.getTau() + desiredTorqueForCoPControl.getY());
      }
   }

   private final BooleanYoVariable enableCoPSmootherForShakies = new BooleanYoVariable("enableCoPSmootherForShakies", registry);
   private final BooleanYoVariable isCoPControlBad = new BooleanYoVariable("isCoPControlBad", registry);
   private final BooleanYoVariable isDesiredCoPBeingSmoothened = new BooleanYoVariable("isDesiredCoPBeingSmoothened", registry);
   private final DoubleYoVariable copSmootherErrorThreshold = new DoubleYoVariable("copSmootherErrorThreshold", registry);
   private final DoubleYoVariable copSmootherControlStartTime = new DoubleYoVariable("copSmootherControlStartTime", registry);
   private final DoubleYoVariable defaultWRhoSmoother = new DoubleYoVariable("defaultWRhoSmoother", registry);
   private final DoubleYoVariable maxWRhoSmoother = new DoubleYoVariable("maxWRhoSmoother", registry);
   private final DoubleYoVariable copSmootherControlDuration = new DoubleYoVariable("copSmootherControlDuration", registry);
   private final DoubleYoVariable copSmootherPercent = new DoubleYoVariable("copSmootherPercent", registry);

   public void smoothDesiredCoPIfNeeded()
   {
      boolean atLeastOneFootWithBadCoPControl = false;

      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody contactablePlaneBody = feet.get(robotSide);
         ReferenceFrame planeFrame = contactablePlaneBody.getSoleFrame();

         FramePoint2d cop = planeContactWrenchProcessor.getCoP(contactablePlaneBody);

         if ((cop == null) || cop.containsNaN())
         {
            yoCoPError.get(robotSide).setToZero();
            yoCoPErrorMagnitude.get(robotSide).set(0.0);
         }

         copDesired.setIncludingFrame(cop);
         FootSwitchInterface footSwitch = footSwitches.get(robotSide);
         footSwitch.computeAndPackCoP(copActual);

         if (copActual.containsNaN())
         {
            yoCoPError.get(robotSide).setToZero();
            yoCoPErrorMagnitude.get(robotSide).set(0.0);
         }

         copError.setToZero(planeFrame);
         copError.sub(copDesired, copActual);
         yoCoPError.get(robotSide).set(copError);
         yoCoPErrorMagnitude.get(robotSide).set(copError.length());

         if (yoCoPErrorMagnitude.get(robotSide).getDoubleValue() > copSmootherErrorThreshold.getDoubleValue())
         {
            atLeastOneFootWithBadCoPControl = true;
         }
      }

      isCoPControlBad.set(atLeastOneFootWithBadCoPControl);

      if (atLeastOneFootWithBadCoPControl && !isDesiredCoPBeingSmoothened.getBooleanValue())
      {
         isDesiredCoPBeingSmoothened.set(true);
         copSmootherControlStartTime.set(yoTime.getDoubleValue());
      }

      if (isDesiredCoPBeingSmoothened.getBooleanValue())
      {
         double deltaTime = yoTime.getDoubleValue() - copSmootherControlStartTime.getDoubleValue();
         double percent = MathTools.clipToMinMax(deltaTime / copSmootherControlDuration.getDoubleValue(), 0.0, 1.0);
         copSmootherPercent.set(percent);

         if (enableCoPSmootherForShakies.getBooleanValue())
         {
            double wRhoSmoother = percent * defaultWRhoSmoother.getDoubleValue() + (1.0 - percent) * maxWRhoSmoother.getDoubleValue();
            optimizationMomentumControlModule.setWRhoSmoother(wRhoSmoother);
         }

         if (percent >= 1.0)
         {
            optimizationMomentumControlModule.setWRhoSmoother(defaultWRhoSmoother.getDoubleValue());
            isDesiredCoPBeingSmoothened.set(false);
         }
      }
   }

   private void updateMomentumBasedControllerSpy()
   {
      if (momentumBasedControllerSpy != null)
      {
         for (int i = 0; i < contactablePlaneBodyList.size(); i++)
         {
            ContactablePlaneBody contactablePlaneBody = contactablePlaneBodyList.get(i);
            YoPlaneContactState contactState = yoPlaneContactStates.get(contactablePlaneBody);
            if (contactState.inContact())
            {
               momentumBasedControllerSpy.setPlaneContactState(contactablePlaneBody, contactState.getContactFramePoints2dInContactCopy(),
                       contactState.getCoefficientOfFriction(), contactState.getContactNormalFrameVectorCopy());
            }
         }

         momentumBasedControllerSpy.doSecondaryControl();
      }
   }

   public void callUpdatables()
   {
      double time = yoTime.getDoubleValue();
      for (int i = 0; i < updatables.size(); i++)
      {
         updatables.get(i).update(time);
      }
   }

   public void addUpdatable(Updatable updatable)
   {
      updatables.add(updatable);
   }

   public void doPDControl(OneDoFJoint[] joints, YoPDGains gains)
   {
      double kp = gains.getKp();
      double kd = gains.getKd();
      double maxAcceleration = gains.getMaximumAcceleration();
      double maxJerk = gains.getMaximumJerk();
      doPDControl(joints, kp, kd, maxAcceleration, maxJerk);
   }

   public void doPDControl(OneDoFJoint[] joints, double kp, double kd, double maxAcceleration, double maxJerk)
   {
      for (OneDoFJoint joint : joints)
      {
         doPDControl(joint, kp, kd, joint.getqDesired(), 0.0, maxAcceleration, maxJerk);
      }
   }

   public void doPDControl(OneDoFJoint joint, double desiredPosition, double desiredVelocity, YoPDGains gains)
   {
      double kp = gains.getKp();
      double kd = gains.getKd();
      double maxAcceleration = gains.getMaximumAcceleration();
      double maxJerk = gains.getMaximumJerk();
      doPDControl(joint, kp, kd, desiredPosition, desiredVelocity, maxAcceleration, maxJerk);
   }

   public void doPDControl(OneDoFJoint joint, double kp, double kd, double desiredPosition, double desiredVelocity, double maxAcceleration, double maxJerk)
   {
      double desiredAcceleration = computeDesiredAcceleration(kp, kd, desiredPosition, desiredVelocity, joint);

      desiredAcceleration = MathTools.clipToMinMax(desiredAcceleration, maxAcceleration);
      preRateLimitedDesiredAccelerations.get(joint).set(desiredAcceleration);

      RateLimitedYoVariable rateLimitedDesiredAcceleration = this.rateLimitedDesiredAccelerations.get(joint);
      rateLimitedDesiredAcceleration.setMaxRate(maxJerk);
      rateLimitedDesiredAcceleration.update(desiredAcceleration);

      setOneDoFJointAcceleration(joint, rateLimitedDesiredAcceleration.getDoubleValue());
   }

   private static double computeDesiredAcceleration(double k, double d, double qDesired, double qdDesired, OneDoFJoint joint)
   {
      return k * (qDesired - joint.getQ()) + d * (qdDesired - joint.getQd());
   }

   private final Map<OneDoFJoint, DenseMatrix64F> tempJointAcceleration = new LinkedHashMap<OneDoFJoint, DenseMatrix64F>();
   private final Map<OneDoFJoint, DesiredJointAccelerationCommand> tempDesiredJointAccelerationCommands = new LinkedHashMap<>();

   public void setOneDoFJointAcceleration(OneDoFJoint joint, double desiredAcceleration)
   {
      DenseMatrix64F jointAcceleration = tempJointAcceleration.get(joint);
      if (jointAcceleration == null)
      {
         jointAcceleration = new DenseMatrix64F(joint.getDegreesOfFreedom(), 1);
         tempJointAcceleration.put(joint, jointAcceleration);
      }

      jointAcceleration.set(0, 0, desiredAcceleration);

      if (momentumBasedControllerSpy != null)
      {
         momentumBasedControllerSpy.setDesiredJointAcceleration(joint, jointAcceleration);
      }

      DesiredJointAccelerationCommand desiredJointAccelerationCommand = tempDesiredJointAccelerationCommands.get(joint);

      if (desiredJointAccelerationCommand == null)
      {
         desiredJointAccelerationCommand = new DesiredJointAccelerationCommand(joint, jointAcceleration);
         tempDesiredJointAccelerationCommands.put(joint, desiredJointAccelerationCommand);
      }
      else
      {
         desiredJointAccelerationCommand.setDesiredAcceleration(jointAcceleration);
      }

      momentumControlModuleBridge.setDesiredJointAcceleration(desiredJointAccelerationCommand);
   }

   private final SpatialAccelerationVector rootJointDesiredAcceleration = new SpatialAccelerationVector();
   private final FrameVector rootJointDesiredAngularAcceleration = new FrameVector();
   private final FrameVector rootJointDesiredLinearAcceleration = new FrameVector();
   private final FrameVector rootJointLinearAccelerationInWorld = new FrameVector();

   private final Twist rootJointTwist = new Twist();
   private final FrameVector twistOfRootJointAngularPart = new FrameVector();
   private final FrameVector twistOfRootJointLinearPart = new FrameVector();

   private final Wrench residualRootJointWrench = new Wrench();

   private final SpatialAccelerationVector spatialAccelerationOfFoot = new SpatialAccelerationVector();
   private final FrameVector footSpatialAccelerationAngularPart = new FrameVector();
   private final FrameVector footSpatialAccelerationLinearPart = new FrameVector();


   private void updateYoVariables()
   {
      // TODO: Make a visualizer that gets attached rather than having it in this class.

      RigidBody elevator = fullRobotModel.getElevator();
      SpatialAccelerationCalculator spatialAccelerationCalculator = inverseDynamicsCalculator.getSpatialAccelerationCalculator();

      // Foot Accelerations
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody foot = fullRobotModel.getFoot(robotSide);

         spatialAccelerationCalculator.compute();
         spatialAccelerationCalculator.packRelativeAcceleration(spatialAccelerationOfFoot, elevator, foot);

         spatialAccelerationOfFoot.packAngularPart(footSpatialAccelerationAngularPart);
         yoFeetDesiredAngularAccelerations.get(robotSide).set(footSpatialAccelerationAngularPart);

         spatialAccelerationOfFoot.packLinearPart(footSpatialAccelerationLinearPart);
         yoFeetDesiredLinearAccelerations.get(robotSide).set(footSpatialAccelerationLinearPart);
      }

      // Root Joint Accelerations
      SixDoFJoint rootJoint = fullRobotModel.getRootJoint();
      rootJoint.packDesiredJointAcceleration(rootJointDesiredAcceleration);

      rootJointDesiredAcceleration.packAngularPart(rootJointDesiredAngularAcceleration);
      rootJointDesiredAcceleration.packLinearPart(rootJointDesiredLinearAcceleration);

      yoRootJointDesiredAngularAcceleration.set(rootJointDesiredAngularAcceleration);
      yoRootJointDesiredLinearAcceleration.set(rootJointDesiredLinearAcceleration);

      rootJoint.packJointTwist(rootJointTwist);
      rootJointDesiredAcceleration.getLinearAccelerationFromOriginAcceleration(rootJointTwist, rootJointLinearAccelerationInWorld);
      rootJointLinearAccelerationInWorld.changeFrame(ReferenceFrame.getWorldFrame());
      yoRootJointDesiredLinearAccelerationWorld.set(rootJointLinearAccelerationInWorld);

      rootJointDesiredAngularAcceleration.changeFrame(ReferenceFrame.getWorldFrame());
      yoRootJointDesiredAngularAccelerationWorld.set(rootJointDesiredAngularAcceleration);

      // Root Joint Twist
      rootJointTwist.packAngularPart(twistOfRootJointAngularPart);
      yoRootJointAngularVelocity.set(twistOfRootJointAngularPart);

      rootJointTwist.packLinearPart(twistOfRootJointLinearPart);
      twistOfRootJointLinearPart.changeFrame(ReferenceFrame.getWorldFrame());
      yoRootJointLinearVelocity.set(twistOfRootJointLinearPart);

      // Root Joint Residual Wrench
      rootJoint.packWrench(residualRootJointWrench);
      residualRootJointWrench.changeFrame(referenceFrames.getCenterOfMassFrame());
      residualRootJointForce.set(residualRootJointWrench.getLinearPartX(), residualRootJointWrench.getLinearPartY(), residualRootJointWrench.getLinearPartZ());
      residualRootJointTorque.set(residualRootJointWrench.getAngularPartX(), residualRootJointWrench.getAngularPartY(),
                                  residualRootJointWrench.getAngularPartZ());

      // Desired torques and accelerations of oneDoFJoints
      for (int i = 0; i < jointsWithDesiredAcceleration.size(); i++)
      {
         OneDoFJoint joint = jointsWithDesiredAcceleration.get(i);
         desiredTorqueYoVariables.get(joint).set(joint.getTau());
         desiredAccelerationYoVariables.get(joint).set(joint.getQddDesired());
      }
   }

   public void initialize()
   {
      // When you initialize into this controller, reset the estimator positions to current. Otherwise it might be in a bad state
      // where the feet are all jacked up. For example, after falling and getting back up.
      inverseDynamicsCalculator.compute();
      momentumControlModuleBridge.initialize();
      planeContactWrenchProcessor.initialize();
      callUpdatables();
   }

   private void readWristSensorData()
   {
      if (wristForceSensors == null)
         return;

      for (RobotSide robotSide : RobotSide.values)
      {
         ForceSensorDataReadOnly wristForceSensor = wristForceSensors.get(robotSide);
         ReferenceFrame measurementFrame = wristForceSensor.getMeasurementFrame();
         wristForceSensor.packWrench(wristTempWrench);

         wristTempWrench.packLinearPartIncludingFrame(tempWristForce);
         wristTempWrench.packAngularPartIncludingFrame(tempWristTorque);

         wristRawMeasuredForces.get(robotSide).setAndMatchFrame(tempWristForce);
         wristRawMeasuredTorques.get(robotSide).setAndMatchFrame(tempWristTorque);

         cancelHandWeight(robotSide, wristTempWrench, measurementFrame);

         wristTempWrench.packLinearPartIncludingFrame(tempWristForce);
         wristTempWrench.packAngularPartIncludingFrame(tempWristTorque);

         wristForcesHandWeightCancelled.get(robotSide).setAndMatchFrame(tempWristForce);
         wristTorquesHandWeightCancelled.get(robotSide).setAndMatchFrame(tempWristTorque);
      }
   }

   private void cancelHandWeight(RobotSide robotSide, Wrench wrenchToSubstractHandWeightTo, ReferenceFrame measurementFrame)
   {
      CenterOfMassReferenceFrame handCoMFrame = handCenterOfMassFrames.get(robotSide);
      handCoMFrame.update();
      tempWristForce.setIncludingFrame(worldFrame, 0.0, 0.0, -handsMass.get(robotSide).getDoubleValue() * gravity);
      tempWristForce.changeFrame(handCoMFrame);
      wristWrenchDueToGravity.setToZero(measurementFrame, handCoMFrame);
      wristWrenchDueToGravity.setLinearPart(tempWristForce);
      wristWrenchDueToGravity.changeFrame(measurementFrame);

      wrenchToSubstractHandWeightTo.sub(wristWrenchDueToGravity);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return getName();
   }

   public FramePoint2d getDesiredCoP(ContactablePlaneBody contactablePlaneBody)
   {
      return planeContactWrenchProcessor.getCoP(contactablePlaneBody);
   }

   public void updateContactPointsForUpcomingFootstep(Footstep nextFootstep)
   {
      RobotSide robotSide = nextFootstep.getRobotSide();

      List<Point2d> predictedContactPoints = nextFootstep.getPredictedContactPoints();

      if ((predictedContactPoints != null) && (!predictedContactPoints.isEmpty()))
      {
         setFootPlaneContactPoints(robotSide, predictedContactPoints);
      }
      else
      {
         resetFootPlaneContactPoint(robotSide);
      }
   }

   public void setFootstepsContactPointsBasedOnFootContactStatePoints(Footstep footstep)
   {
      RobotSide robotSide = footstep.getRobotSide();
      ContactablePlaneBody foot = feet.get(robotSide);
      YoPlaneContactState footContactState = yoPlaneContactStates.get(foot);
      List<YoContactPoint> contactPoints = footContactState.getContactPoints();

      ArrayList<FramePoint2d> contactPointList = new ArrayList<FramePoint2d>();

      for (YoContactPoint contactPoint : contactPoints)
      {
         FramePoint2d framePoint2 = new FramePoint2d();
         contactPoint.getPosition2d(framePoint2);
         contactPointList.add(framePoint2);
      }

      footstep.setPredictedContactPointsFromFramePoint2ds(contactPointList);
   }

   public void getCenterOfFootContactPoints(RobotSide robotSide, FramePoint2d centroidToPack)
   {
      ContactablePlaneBody foot = feet.get(robotSide);
      YoPlaneContactState footContactState = yoPlaneContactStates.get(foot);
      footContactState.getContactPointCentroid(centroidToPack);
   }

   private void resetFootPlaneContactPoint(RobotSide robotSide)
   {
      ContactablePlaneBody foot = feet.get(robotSide);
      YoPlaneContactState footContactState = yoPlaneContactStates.get(foot);
      footContactState.setContactFramePoints(foot.getContactPoints2d());
   }

   private void setFootPlaneContactPoints(RobotSide robotSide, List<Point2d> predictedContactPoints)
   {
      ContactablePlaneBody foot = feet.get(robotSide);
      YoPlaneContactState footContactState = yoPlaneContactStates.get(foot);
      footContactState.setContactPoints(predictedContactPoints);
   }

   public void setPlaneContactCoefficientOfFriction(ContactablePlaneBody contactableBody, double coefficientOfFriction)
   {
      YoPlaneContactState yoPlaneContactState = yoPlaneContactStates.get(contactableBody);
      yoPlaneContactState.setCoefficientOfFriction(coefficientOfFriction);
   }

   public void setPlaneContactStateNormalContactVector(ContactablePlaneBody contactableBody, FrameVector normalContactVector)
   {
      YoPlaneContactState yoPlaneContactState = yoPlaneContactStates.get(contactableBody);
      yoPlaneContactState.setContactNormalVector(normalContactVector);
   }

   public void setPlaneContactState(ContactablePlaneBody contactableBody, boolean[] newContactPointStates)
   {
      YoPlaneContactState yoPlaneContactState = yoPlaneContactStates.get(contactableBody);
      yoPlaneContactState.setContactPointsInContact(newContactPointStates);
   }

   public void setPlaneContactState(ContactablePlaneBody contactableBody, boolean[] newContactPointStates, FrameVector normalContactVector)
   {
      YoPlaneContactState yoPlaneContactState = yoPlaneContactStates.get(contactableBody);
      yoPlaneContactState.setContactPointsInContact(newContactPointStates);
      yoPlaneContactState.setContactNormalVector(normalContactVector);
   }

   public void setPlaneContactStateFullyConstrained(ContactablePlaneBody contactableBody)
   {
      YoPlaneContactState yoPlaneContactState = yoPlaneContactStates.get(contactableBody);
      yoPlaneContactState.setFullyConstrained();
   }

   public void setPlaneContactStateFullyConstrained(ContactablePlaneBody contactableBody, double coefficientOfFriction, FrameVector normalContactVector)
   {
      YoPlaneContactState yoPlaneContactState = yoPlaneContactStates.get(contactableBody);
      yoPlaneContactState.setFullyConstrained();
      yoPlaneContactState.setCoefficientOfFriction(coefficientOfFriction);
      yoPlaneContactState.setContactNormalVector(normalContactVector);
   }

   public void setPlaneContactStateFree(ContactablePlaneBody contactableBody)
   {
      YoPlaneContactState yoPlaneContactState = yoPlaneContactStates.get(contactableBody);
      if (yoPlaneContactState != null)
         yoPlaneContactState.clear();
   }

   public ReferenceFrame getCenterOfMassFrame()
   {
      return centerOfMassFrame;
   }


   public void setDesiredSpatialAcceleration(int jacobianId, TaskspaceConstraintData taskspaceConstraintData)
   {
      GeometricJacobian jacobian = getJacobian(jacobianId);
      setDesiredSpatialAcceleration(jacobian, taskspaceConstraintData);
   }

   public void setDesiredSpatialAcceleration(GeometricJacobian jacobian, TaskspaceConstraintData taskspaceConstraintData)
   {
      if (momentumBasedControllerSpy != null)
      {
         momentumBasedControllerSpy.setDesiredSpatialAcceleration(jacobian, taskspaceConstraintData);
      }

      DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand = desiredSpatialAccelerationCommandPool.getUnusedDesiredSpatialAccelerationCommand(jacobian, taskspaceConstraintData);
      momentumControlModuleBridge.setDesiredSpatialAcceleration(desiredSpatialAccelerationCommand);
   }

   public void setDesiredPointAcceleration(int rootToEndEffectorJacobianId, FramePoint contactPoint, FrameVector desiredAcceleration)
   {
      GeometricJacobian rootToEndEffectorJacobian = getJacobian(rootToEndEffectorJacobianId);

      if (momentumBasedControllerSpy != null)
      {
         momentumBasedControllerSpy.setDesiredPointAcceleration(rootToEndEffectorJacobian, contactPoint, desiredAcceleration);
      }

      DesiredPointAccelerationCommand desiredPointAccelerationCommand = new DesiredPointAccelerationCommand(rootToEndEffectorJacobian, contactPoint,
                                                                           desiredAcceleration);
      momentumControlModuleBridge.setDesiredPointAcceleration(desiredPointAccelerationCommand);
   }

   public void setDesiredPointAcceleration(int rootToEndEffectorJacobianId, FramePoint contactPoint, FrameVector desiredAcceleration,
           DenseMatrix64F selectionMatrix)
   {
      GeometricJacobian rootToEndEffectorJacobian = getJacobian(rootToEndEffectorJacobianId);

      if (momentumBasedControllerSpy != null)
      {
         momentumBasedControllerSpy.setDesiredPointAcceleration(rootToEndEffectorJacobian, contactPoint, desiredAcceleration);
      }

      DesiredPointAccelerationCommand desiredPointAccelerationCommand = new DesiredPointAccelerationCommand(rootToEndEffectorJacobian, contactPoint,
                                                                           desiredAcceleration, selectionMatrix);
      momentumControlModuleBridge.setDesiredPointAcceleration(desiredPointAccelerationCommand);
   }

   public void setDesiredRateOfChangeOfMomentum(MomentumRateOfChangeData momentumRateOfChangeData)
   {
      if (momentumBasedControllerSpy != null)
      {
         momentumBasedControllerSpy.setDesiredRateOfChangeOfMomentum(momentumRateOfChangeData);
      }

      DesiredRateOfChangeOfMomentumCommand desiredRateOfChangeOfMomentumCommand = new DesiredRateOfChangeOfMomentumCommand(momentumRateOfChangeData);
      momentumControlModuleBridge.setDesiredRateOfChangeOfMomentum(desiredRateOfChangeOfMomentumCommand);
   }

   public ReferenceFrame getPelvisZUpFrame()
   {
      return referenceFrames.getPelvisZUpFrame();
   }

   @Deprecated
   public EnumYoVariable<RobotSide> getUpcomingSupportLeg()
   {
      return upcomingSupportLeg;
   }

   public CommonHumanoidReferenceFrames getReferenceFrames()
   {
      return referenceFrames;
   }

   public DoubleYoVariable getYoTime()
   {
      return yoTime;
   }

   public double getGravityZ()
   {
      return gravity;
   }

   public double getControlDT()
   {
      return controlDT;
   }

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   public TwistCalculator getTwistCalculator()
   {
      return twistCalculator;
   }

   public InverseDynamicsCalculator getInverseDynamicsCalculator()
   {
      return inverseDynamicsCalculator;
   }

   public CenterOfMassJacobian getCenterOfMassJacobian()
   {
      return centerOfMassJacobian;
   }

   public void getCenterOfMassVelocity(FrameVector centerOfMassVelocityToPack)
   {
      centerOfMassJacobian.packCenterOfMassVelocity(centerOfMassVelocityToPack);
   }

   public void getAdmissibleDesiredGroundReactionWrench(SpatialForceVector admissibleDesiredGroundReactionWrenchToPack)
   {
      admissibleDesiredGroundReactionWrenchToPack.setToZero(centerOfMassFrame);
      admissibleDesiredGroundReactionWrenchToPack.set(admissibleDesiredGroundReactionForce.getFrameTuple(),
              admissibleDesiredGroundReactionTorque.getFrameTuple());
   }

   public SideDependentList<ContactablePlaneBody> getContactableFeet()
   {
      return feet;
   }

   public SideDependentList<ContactablePlaneBody> getContactableHands()
   {
      return hands;
   }

   public SideDependentList<ContactablePlaneBody> getContactablePlaneThighs()
   {
      return thighs;
   }

   public ContactablePlaneBody getContactablePelvis()
   {
      return pelvis;
   }

   public ContactablePlaneBody getContactablePelvisBack()
   {
      return pelvisBack;
   }

   public void getContactPoints(ContactablePlaneBody contactablePlaneBody, List<FramePoint> contactPointListToPack)
   {
      yoPlaneContactStates.get(contactablePlaneBody).getContactFramePointsInContact(contactPointListToPack);
   }

   public YoPlaneContactState getContactState(ContactablePlaneBody contactablePlaneBody)
   {
      return yoPlaneContactStates.get(contactablePlaneBody);
   }

   public List<? extends PlaneContactState> getPlaneContactStates()
   {
      return yoPlaneContactStateList;
   }

   public List<ContactablePlaneBody> getContactablePlaneBodyList()
   {
      return contactablePlaneBodyList;
   }

   public void clearContacts()
   {
      for (int i = 0; i < yoPlaneContactStateList.size(); i++)
      {
         yoPlaneContactStateList.get(i).clear();
      }
   }

   public void setSideOfFootUnderCoPControl(RobotSide side)
   {
      footUnderCoPControl.set(side);
   }

   public void setFeetCoPControlIsActive(boolean isActive)
   {
      feetCoPControlIsActive.set(isActive);
   }

   public void setFootCoPOffsetX(double offseX)
   {
      footCoPOffsetX.set(offseX);
   }

   public void setFootCoPOffsetY(double offseY)
   {
      footCoPOffsetY.set(offseY);
   }

   public void setMomentumControlModuleToUse(MomentumControlModuleType momentumControlModuleToUse)
   {
      momentumControlModuleBridge.setMomentumControlModuleToUse(momentumControlModuleToUse);
   }

   public int getOrCreateGeometricJacobian(RigidBody ancestor, RigidBody descendant, ReferenceFrame jacobianFrame)
   {
      return robotJacobianHolder.getOrCreateGeometricJacobian(ancestor, descendant, jacobianFrame);
   }

   public int getOrCreateGeometricJacobian(InverseDynamicsJoint[] joints, ReferenceFrame jacobianFrame)
   {
      return robotJacobianHolder.getOrCreateGeometricJacobian(joints, jacobianFrame);
   }

   /**
    * Return a jacobian previously created with the getOrCreate method using a jacobianId.
    * @param jacobianId
    * @return
    */
   public GeometricJacobian getJacobian(int jacobianId)
   {
      return robotJacobianHolder.getJacobian(jacobianId);
   }

   public SideDependentList<FootSwitchInterface> getFootSwitches()
   {
      return footSwitches;
   }

   public SideDependentList<ForceSensorDataReadOnly> getWristForceSensors()
   {
      return wristForceSensors;
   }

   public ForceSensorDataReadOnly getWristForceSensor(RobotSide robotSide)
   {
      if(wristForceSensors != null)
      {
         return wristForceSensors.get(robotSide);
      }

      return null;
   }

   public YoGraphicsListRegistry getDynamicGraphicObjectsListRegistry()
   {
      return yoGraphicsListRegistry;
   }

   public InverseDynamicsJoint[] getControlledJoints()
   {
      return controlledJoints;
   }

   public void attachControllerFailureListener(ControllerFailureListener listener)
   {
      this.controllerFailureListeners.add(listener);
   }

   public void reportControllerFailureToListeners(FrameVector2d fallingDirection)
   {
      for (int i = 0; i < controllerFailureListeners.size(); i++)
      {
         controllerFailureListeners.get(i).controllerFailed(fallingDirection);
      }
   }

   public void attachControllerStateChangedListener(ControllerStateChangedListener listener)
   {
      this.controllerStateChangedListeners.add(listener);
   }

   public void attachControllerStateChangedListeners(List<ControllerStateChangedListener> listeners)
   {
      for (int i = 0; i < listeners.size(); i++)
      {
         attachControllerStateChangedListener(listeners.get(i));
      }
   }

   public void reportControllerStateChangeToListeners(Enum<?> oldState, Enum<?> newState)
   {
      for (int i = 0; i < controllerStateChangedListeners.size(); i++)
      {
         controllerStateChangedListeners.get(i).controllerStateHasChanged(oldState, newState);
      }
   }

   public void attachRobotMotionStatusChangedListener(RobotMotionStatusChangedListener listener)
   {
      robotMotionStatusChangedListeners.add(listener);
   }

   public void reportChangeOfRobotMotionStatus(RobotMotionStatus newStatus)
   {
      for (int i = 0; i < robotMotionStatusChangedListeners.size(); i++)
      {
         robotMotionStatusChangedListeners.get(i).robotMotionStatusHasChanged(newStatus, yoTime.getDoubleValue());
      }
   }

   public SideDependentList<ProvidedMassMatrixToolRigidBody> getToolRigitBodies()
   {
      return toolRigidBodies;
   }

   public void getWristRawMeasuredWrench(Wrench wrenchToPack, RobotSide robotSide)
   {
      if (wristRawMeasuredForces == null || wristRawMeasuredTorques == null)
         return;

      wristRawMeasuredForces.get(robotSide).getFrameTupleIncludingFrame(tempWristForce);
      wristRawMeasuredTorques.get(robotSide).getFrameTupleIncludingFrame(tempWristTorque);
      ReferenceFrame measurementFrames = wristForceSensorMeasurementFrames.get(robotSide);
      wrenchToPack.setToZero(measurementFrames, measurementFrames);
      wrenchToPack.setLinearPart(tempWristForce);
      wrenchToPack.setAngularPart(tempWristTorque);
   }

   public void getWristMeasuredWrenchHandWeightCancelled(Wrench wrenchToPack, RobotSide robotSide)
   {
      if (wristForcesHandWeightCancelled == null || wristTorquesHandWeightCancelled == null)
         return;

      wristForcesHandWeightCancelled.get(robotSide).getFrameTupleIncludingFrame(tempWristForce);
      wristTorquesHandWeightCancelled.get(robotSide).getFrameTupleIncludingFrame(tempWristTorque);
      ReferenceFrame measurementFrames = wristForceSensorMeasurementFrames.get(robotSide);
      wrenchToPack.setToZero(measurementFrames, measurementFrames);
      wrenchToPack.setLinearPart(tempWristForce);
      wrenchToPack.setAngularPart(tempWristTorque);
   }
}
