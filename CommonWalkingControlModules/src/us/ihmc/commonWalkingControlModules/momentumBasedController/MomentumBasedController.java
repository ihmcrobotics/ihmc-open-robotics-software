package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

import javax.vecmath.Point2d;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactPointVisualizer;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.WrenchBasedFootSwitch;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector2d;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculatorListener;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
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

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final ReferenceFrame centerOfMassFrame;
   private final FullHumanoidRobotModel fullRobotModel;
   private final CenterOfMassJacobian centerOfMassJacobian;
   private final CommonHumanoidReferenceFrames referenceFrames;
   private final TwistCalculator twistCalculator;

   private final SideDependentList<ContactableFoot> feet;
   private final SideDependentList<ContactablePlaneBody> hands;

   private final List<ContactablePlaneBody> contactablePlaneBodyList;
   private final List<YoPlaneContactState> yoPlaneContactStateList = new ArrayList<YoPlaneContactState>();
   private final LinkedHashMap<ContactablePlaneBody, YoFramePoint2d> footDesiredCenterOfPressures = new LinkedHashMap<>();
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

   private final YoFrameVector admissibleDesiredGroundReactionTorque;
   private final YoFrameVector admissibleDesiredGroundReactionForce;
   private final YoFramePoint2d admissibleDesiredCenterOfPressure;

   // private final YoFrameVector groundReactionTorqueCheck;
   // private final YoFrameVector groundReactionForceCheck;

   private final ArrayList<OneDoFJoint> jointsWithDesiredAcceleration = new ArrayList<>();
   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> desiredAccelerationYoVariables = new LinkedHashMap<OneDoFJoint, DoubleYoVariable>();
   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> desiredTorqueYoVariables = new LinkedHashMap<OneDoFJoint, DoubleYoVariable>();

   private final ContactPointVisualizer contactPointVisualizer;

   private final GeometricJacobianHolder robotJacobianHolder;

   private final SideDependentList<FootSwitchInterface> footSwitches;
   private final SideDependentList<ForceSensorDataReadOnly> wristForceSensors;
   private final DoubleYoVariable alphaCoPControl = new DoubleYoVariable("alphaCoPControl", registry);
   private final DoubleYoVariable maxAnkleTorqueCoPControl = new DoubleYoVariable("maxAnkleTorqueCoPControl", registry);
   private final SideDependentList<AlphaFilteredYoFrameVector2d> desiredTorquesForCoPControl;

   private final SideDependentList<Double> xSignsForCoPControl, ySignsForCoPControl;
   private final double minZForceForCoPControlScaling, maxZForceForCoPControlScaling;

   private final SideDependentList<YoFrameVector2d> yoCoPError;
   private final SideDependentList<DoubleYoVariable> yoCoPErrorMagnitude = new SideDependentList<DoubleYoVariable>(
         new DoubleYoVariable("leftFootCoPErrorMagnitude", registry), new DoubleYoVariable("rightFootCoPErrorMagnitude", registry));
   private final DoubleYoVariable gainCoPX = new DoubleYoVariable("gainCoPX", registry);
   private final DoubleYoVariable gainCoPY = new DoubleYoVariable("gainCoPY", registry);
   private final SideDependentList<DoubleYoVariable> copControlScales;

   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private final InverseDynamicsJoint[] controlledJoints;

   private final SideDependentList<Wrench> handWrenches = new SideDependentList<>();

   private final AntiGravityJointTorquesVisualizer antiGravityJointTorquesVisualizer;

   private final ArrayList<ControllerFailureListener> controllerFailureListeners = new ArrayList<>();
   private final ArrayList<ControllerStateChangedListener> controllerStateChangedListeners = new ArrayList<>();
   private final ArrayList<RobotMotionStatusChangedListener> robotMotionStatusChangedListeners = new ArrayList<>();

   public MomentumBasedController(FullHumanoidRobotModel fullRobotModel, GeometricJacobianHolder robotJacobianHolder, CenterOfMassJacobian centerOfMassJacobian,
         CommonHumanoidReferenceFrames referenceFrames, SideDependentList<FootSwitchInterface> footSwitches,
         SideDependentList<ForceSensorDataReadOnly> wristForceSensors, DoubleYoVariable yoTime, double gravityZ, TwistCalculator twistCalculator,
         SideDependentList<ContactableFoot> feet, SideDependentList<ContactablePlaneBody> hands, double controlDT, ArrayList<Updatable> updatables,
         ArmControllerParameters armControllerParameters, WalkingControllerParameters walkingControllerParameters,
         YoGraphicsListRegistry yoGraphicsListRegistry, InverseDynamicsJoint... jointsToIgnore)
   {
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;

      this.robotJacobianHolder = robotJacobianHolder;
      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();

      this.footSwitches = footSwitches;
      this.wristForceSensors = wristForceSensors;

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
      this.hands = hands;

      RigidBody elevator = fullRobotModel.getElevator();

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

      this.admissibleDesiredGroundReactionTorque = new YoFrameVector("admissibleDesiredGroundReactionTorque", centerOfMassFrame, registry);
      this.admissibleDesiredGroundReactionForce = new YoFrameVector("admissibleDesiredGroundReactionForce", centerOfMassFrame, registry);
      this.admissibleDesiredCenterOfPressure = new YoFramePoint2d("admissibleDesiredCenterOfPressure", worldFrame, registry);
      admissibleDesiredCenterOfPressure.setToNaN();
      yoGraphicsListRegistry.registerArtifact("Desired Center of Pressure",
            new YoGraphicPosition("Desired Overall Center of Pressure", admissibleDesiredCenterOfPressure, 0.008, YoAppearance.Navy(), GraphicType.BALL)
                  .createArtifact());

      // this.groundReactionTorqueCheck = new YoFrameVector("groundReactionTorqueCheck", centerOfMassFrame, registry);
      // this.groundReactionForceCheck = new YoFrameVector("groundReactionForceCheck", centerOfMassFrame, registry);

      for (RobotSide robotSide : RobotSide.values)
      {
         ContactableFoot contactableFoot = feet.get(robotSide);
         ReferenceFrame soleFrame = contactableFoot.getSoleFrame();
         String namePrefix = soleFrame.getName() + "DesiredCoP";
         YoFramePoint2d yoDesiredCenterOfPressure = new YoFramePoint2d(namePrefix, soleFrame, registry);
         footDesiredCenterOfPressures.put(contactableFoot, yoDesiredCenterOfPressure);
      }

      if (updatables != null)
      {
         this.updatables.addAll(updatables);
      }

      double coefficientOfFriction = 1.0; // TODO: magic number...

      // TODO: get rid of the null checks
      this.contactablePlaneBodyList = new ArrayList<ContactablePlaneBody>();

      if (feet != null)
      {
         this.contactablePlaneBodyList.addAll(feet.values()); // leftSole and rightSole
      }

      if (hands != null)
      {
         this.contactablePlaneBodyList.addAll(hands.values());
      }

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
         }
      }

      contactPointVisualizer = new ContactPointVisualizer(new ArrayList<YoPlaneContactState>(yoPlaneContactStateList), yoGraphicsListRegistry, registry);

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
               AlphaFilteredYoFrameVector2d.createAlphaFilteredYoFrameVector2d(
                     "desired" + robotSide.getCamelCaseNameForMiddleOfExpression() + "AnkleTorqueForCoPControl", "", registry, alphaCoPControl,
                     feet.get(robotSide).getSoleFrame()));
         yoCoPError.put(robotSide,
               new YoFrameVector2d(robotSide.getCamelCaseNameForStartOfExpression() + "FootCoPError", feet.get(robotSide).getSoleFrame(), registry));

         RigidBody hand = fullRobotModel.getHand(robotSide);
         if (hand != null)
         {
            handWrenches.put(robotSide, new Wrench());
         }
      }

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
      throw new RuntimeException("Sylvain was there.");
   }

   public void getFeetContactStates(ArrayList<PlaneContactState> feetContactStatesToPack)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         feetContactStatesToPack.add(yoPlaneContactStates.get(feet.get(robotSide)));
      }
   }

   // TODO: Temporary method for a big refactor allowing switching between high level behaviors
   public void doPrioritaryControl()
   {
      robotJacobianHolder.compute();

      callUpdatables();

      readWristSensorData();
   }

   // TODO: Temporary method for a big refactor allowing switching between high level behaviors
   public void doSecondaryControl()
   {
      if (antiGravityJointTorquesVisualizer != null)
         antiGravityJointTorquesVisualizer.computeAntiGravityJointTorques();

      if (contactPointVisualizer != null)
         contactPointVisualizer.update(yoTime.getDoubleValue());

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

   public final void doProportionalControlOnCoP(SideDependentList<FramePoint2d> desiredCoPs)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody contactablePlaneBody = feet.get(robotSide);
         ReferenceFrame planeFrame = contactablePlaneBody.getSoleFrame();
         AlphaFilteredYoFrameVector2d desiredTorqueForCoPControl = desiredTorquesForCoPControl.get(robotSide);
         copDesired.setIncludingFrame(desiredCoPs.get(robotSide));

         if (copDesired.containsNaN())
         {
            desiredTorqueForCoPControl.setToZero();
            return;
         }

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
         footWrench.getLinearPart(footForceVector);
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
   private final DoubleYoVariable currentWRhoSmootherForShakies = new DoubleYoVariable("currentWRhoSmootherForShakies", registry);
   private final BooleanYoVariable isCoPControlBad = new BooleanYoVariable("isCoPControlBad", registry);
   private final BooleanYoVariable isDesiredCoPBeingSmoothened = new BooleanYoVariable("isDesiredCoPBeingSmoothened", registry);
   private final DoubleYoVariable copSmootherErrorThreshold = new DoubleYoVariable("copSmootherErrorThreshold", registry);
   private final DoubleYoVariable copSmootherControlStartTime = new DoubleYoVariable("copSmootherControlStartTime", registry);
   private final DoubleYoVariable defaultWRhoSmoother = new DoubleYoVariable("defaultWRhoSmoother", registry);
   private final DoubleYoVariable maxWRhoSmoother = new DoubleYoVariable("maxWRhoSmoother", registry);
   private final DoubleYoVariable copSmootherControlDuration = new DoubleYoVariable("copSmootherControlDuration", registry);
   private final DoubleYoVariable copSmootherPercent = new DoubleYoVariable("copSmootherPercent", registry);

   public double smoothDesiredCoPIfNeeded(SideDependentList<FramePoint2d> desiredCoPs)
   {
      boolean atLeastOneFootWithBadCoPControl = false;

      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody contactablePlaneBody = feet.get(robotSide);
         ReferenceFrame planeFrame = contactablePlaneBody.getSoleFrame();

         copDesired.setIncludingFrame(desiredCoPs.get(robotSide));

         if (copDesired.containsNaN())
         {
            yoCoPError.get(robotSide).setToZero();
            yoCoPErrorMagnitude.get(robotSide).set(0.0);
         }

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
            currentWRhoSmootherForShakies.set(wRhoSmoother);
         }

         if (percent >= 1.0)
         {
            currentWRhoSmootherForShakies.set(defaultWRhoSmoother.getDoubleValue());
            isDesiredCoPBeingSmoothened.set(false);
         }
      }

      return currentWRhoSmootherForShakies.getDoubleValue();
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

   public void addUpdatables(List<Updatable> updatables)
   {
      for (int i = 0; i < updatables.size(); i++)
         this.updatables.add(updatables.get(i));
   }

   public void initialize()
   {
      // When you initialize into this controller, reset the estimator positions to current. Otherwise it might be in a bad state
      // where the feet are all jacked up. For example, after falling and getting back up.
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
         wristForceSensor.getWrench(wristTempWrench);

         wristTempWrench.getLinearPartIncludingFrame(tempWristForce);
         wristTempWrench.getAngularPartIncludingFrame(tempWristTorque);

         wristRawMeasuredForces.get(robotSide).setAndMatchFrame(tempWristForce);
         wristRawMeasuredTorques.get(robotSide).setAndMatchFrame(tempWristTorque);

         cancelHandWeight(robotSide, wristTempWrench, measurementFrame);

         wristTempWrench.getLinearPartIncludingFrame(tempWristForce);
         wristTempWrench.getAngularPartIncludingFrame(tempWristTorque);

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

   public void setDesiredCenterOfPressure(ContactablePlaneBody contactablePlaneBody, FramePoint2d desiredCoP)
   {
      YoFramePoint2d cop = footDesiredCenterOfPressures.get(contactablePlaneBody);
      if (cop != null)
         cop.set(desiredCoP);
   }

   public void getDesiredCenterOfPressure(ContactablePlaneBody contactablePlaneBody, FramePoint2d desiredCoPToPack)
   {
      footDesiredCenterOfPressures.get(contactablePlaneBody).getFrameTuple2dIncludingFrame(desiredCoPToPack);
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

   public ReferenceFrame getPelvisZUpFrame()
   {
      return referenceFrames.getPelvisZUpFrame();
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

   public CenterOfMassJacobian getCenterOfMassJacobian()
   {
      return centerOfMassJacobian;
   }

   public void getCenterOfMassVelocity(FrameVector centerOfMassVelocityToPack)
   {
      centerOfMassJacobian.getCenterOfMassVelocity(centerOfMassVelocityToPack);
   }

   public void getAdmissibleDesiredGroundReactionWrench(SpatialForceVector admissibleDesiredGroundReactionWrenchToPack)
   {
      admissibleDesiredGroundReactionWrenchToPack.setToZero(centerOfMassFrame);
      admissibleDesiredGroundReactionWrenchToPack.set(admissibleDesiredGroundReactionForce.getFrameTuple(),
            admissibleDesiredGroundReactionTorque.getFrameTuple());
   }

   public SideDependentList<ContactableFoot> getContactableFeet()
   {
      return feet;
   }

   public SideDependentList<ContactablePlaneBody> getContactableHands()
   {
      return hands;
   }

   public void getContactPoints(ContactablePlaneBody contactablePlaneBody, List<FramePoint> contactPointListToPack)
   {
      yoPlaneContactStates.get(contactablePlaneBody).getContactFramePointsInContact(contactPointListToPack);
   }

   public YoPlaneContactState getContactState(ContactablePlaneBody contactablePlaneBody)
   {
      return yoPlaneContactStates.get(contactablePlaneBody);
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

   public long getOrCreateGeometricJacobian(RigidBody ancestor, RigidBody descendant, ReferenceFrame jacobianFrame)
   {
      return robotJacobianHolder.getOrCreateGeometricJacobian(ancestor, descendant, jacobianFrame);
   }

   public long getOrCreateGeometricJacobian(InverseDynamicsJoint[] joints, ReferenceFrame jacobianFrame)
   {
      return robotJacobianHolder.getOrCreateGeometricJacobian(joints, jacobianFrame);
   }

   /**
    * Return a jacobian previously created with the getOrCreate method using a jacobianId.
    * @param jacobianId
    * @return
    */
   public GeometricJacobian getJacobian(long jacobianId)
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
      if (wristForceSensors != null)
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
