package us.ihmc.commonWalkingControlModules.momentumBasedController;

import static us.ihmc.robotics.lists.FrameTuple2dArrayList.createFramePoint2dArrayList;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactPointVisualizer;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonHumanoidReferenceFramesVisualizer;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex2DSupplier;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Momentum;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.controllers.ControllerFailureListener;
import us.ihmc.robotics.controllers.ControllerStateChangedListener;
import us.ihmc.robotics.lists.FrameTuple2dArrayList;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFramePoint2d;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.MomentumCalculator;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.sensorProcessing.model.RobotMotionStatusChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector2D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class HighLevelHumanoidControllerToolbox
{
   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final ReferenceFrame centerOfMassFrame;
   private final FullHumanoidRobotModel fullRobotModel;
   private final CapturePointCalculator capturePointCalculator;

   private final CommonHumanoidReferenceFrames referenceFrames;
   private final CommonHumanoidReferenceFramesVisualizer referenceFramesVisualizer;

   protected final SideDependentList<ContactableFoot> feet;
   protected final List<ContactablePlaneBody> contactableBodies;
   private final SideDependentList<YoPlaneContactState> footContactStates = new SideDependentList<>();
   private final SideDependentList<FrameConvexPolygon2D> defaultFootPolygons = new SideDependentList<>();

   private final ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver;

   protected final LinkedHashMap<ContactablePlaneBody, YoFramePoint2D> footDesiredCenterOfPressures = new LinkedHashMap<>();
   private final YoDouble desiredCoPAlpha;
   private final LinkedHashMap<ContactablePlaneBody, AlphaFilteredYoFramePoint2d> filteredFootDesiredCenterOfPressures = new LinkedHashMap<>();

   private final ArrayList<Updatable> updatables = new ArrayList<Updatable>();
   private final YoDouble yoTime;
   private final double controlDT;
   private final double gravity;

   private final SideDependentList<CenterOfMassReferenceFrame> handCenterOfMassFrames;
   private final SideDependentList<YoFrameVector3D> wristRawMeasuredForces;
   private final SideDependentList<YoFrameVector3D> wristRawMeasuredTorques;
   private final SideDependentList<YoFrameVector3D> wristForcesHandWeightCancelled;
   private final SideDependentList<YoFrameVector3D> wristTorquesHandWeightCancelled;
   private final SideDependentList<ReferenceFrame> wristForceSensorMeasurementFrames;
   private final Wrench wristWrenchDueToGravity = new Wrench();
   private final Wrench wristTempWrench = new Wrench();
   private final FrameVector3D tempWristForce = new FrameVector3D();
   private final FrameVector3D tempWristTorque = new FrameVector3D();

   private final SideDependentList<YoDouble> handsMass;

   private final SideDependentList<FootSwitchInterface> footSwitches;
   private final SideDependentList<ForceSensorDataReadOnly> wristForceSensors;
   private final YoDouble alphaCoPControl = new YoDouble("alphaCoPControl", registry);
   private final YoDouble maxAnkleTorqueCoPControl = new YoDouble("maxAnkleTorqueCoPControl", registry);

   private final SideDependentList<Double> xSignsForCoPControl, ySignsForCoPControl;
   private final double minZForceForCoPControlScaling;

   private final SideDependentList<YoFrameVector2D> yoCoPError;
   private final SideDependentList<YoDouble> yoCoPErrorMagnitude = new SideDependentList<YoDouble>(new YoDouble("leftFootCoPErrorMagnitude",
                                                                                                                                        registry),
                                                                                                                   new YoDouble("rightFootCoPErrorMagnitude",
                                                                                                                                        registry));
   private final SideDependentList<YoDouble> copControlScales;

   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private final JointBasics[] controlledJoints;
   private final OneDoFJointBasics[] controlledOneDoFJoints;

   private final SideDependentList<Wrench> handWrenches = new SideDependentList<>();

   private final ArrayList<ControllerFailureListener> controllerFailureListeners = new ArrayList<>();
   private final ArrayList<ControllerStateChangedListener> controllerStateChangedListeners = new ArrayList<>();
   private final ArrayList<RobotMotionStatusChangedListener> robotMotionStatusChangedListeners = new ArrayList<>();

   private final BipedSupportPolygons bipedSupportPolygons;

   private final SideDependentList<FrameTuple2dArrayList<FramePoint2D>> previousFootContactPoints = new SideDependentList<>(createFramePoint2dArrayList(),
                                                                                                                            createFramePoint2dArrayList());

   protected final YoFramePoint3D yoCapturePoint = new YoFramePoint3D("capturePoint", worldFrame, registry);

   private final YoDouble omega0 = new YoDouble("omega0", registry);

   private final MomentumCalculator momentumCalculator;
   private final YoFrameVector3D yoAngularMomentum;
   private final AlphaFilteredYoFrameVector filteredYoAngularMomentum;
   private final YoDouble totalMass = new YoDouble("TotalMass", registry);

   private final FramePoint2D centerOfPressure = new FramePoint2D();
   private final YoFramePoint2D yoCenterOfPressure = new YoFramePoint2D("CenterOfPressure", worldFrame, registry);

   private WalkingMessageHandler walkingMessageHandler;

   private final YoBoolean controllerFailed = new YoBoolean("controllerFailed", registry);

   public HighLevelHumanoidControllerToolbox(FullHumanoidRobotModel fullRobotModel, CommonHumanoidReferenceFrames referenceFrames,
                                             SideDependentList<? extends FootSwitchInterface> footSwitches,
                                             SideDependentList<ForceSensorDataReadOnly> wristForceSensors, YoDouble yoTime, double gravityZ, double omega0,
                                             SideDependentList<ContactableFoot> feet, double controlDT, List<Updatable> updatables,
                                             List<ContactablePlaneBody> contactableBodies, YoGraphicsListRegistry yoGraphicsListRegistry,
                                             JointBasics... jointsToIgnore)
   {
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;

      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();

      bipedSupportPolygons = new BipedSupportPolygons(referenceFrames, registry, yoGraphicsListRegistry);

      this.footSwitches = new SideDependentList<>(footSwitches);
      this.wristForceSensors = wristForceSensors;

      referenceFrameHashCodeResolver = new ReferenceFrameHashCodeResolver(fullRobotModel, referenceFrames);

      capturePointCalculator = new CapturePointCalculator(centerOfMassFrame, fullRobotModel.getElevator());

      MathTools.checkIntervalContains(gravityZ, 0.0, Double.POSITIVE_INFINITY);

      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = referenceFrames;
      this.controlDT = controlDT;
      this.gravity = gravityZ;
      this.yoTime = yoTime;
      this.omega0.set(omega0);

      if (yoGraphicsListRegistry != null)
      {
         referenceFramesVisualizer = new CommonHumanoidReferenceFramesVisualizer(referenceFrames, yoGraphicsListRegistry, registry);
      }
      else
      {
         referenceFramesVisualizer = null;
      }

      // Initialize the contactable bodies
      this.feet = feet;
      this.contactableBodies = contactableBodies;

      RigidBodyBasics elevator = fullRobotModel.getElevator();
      double totalMass = TotalMassCalculator.computeSubTreeMass(elevator);

      desiredCoPAlpha = new YoDouble("desiredCoPAlpha", registry);
      desiredCoPAlpha.set(0.9);
      for (RobotSide robotSide : RobotSide.values)
      {
         ContactableFoot contactableFoot = feet.get(robotSide);
         ReferenceFrame soleFrame = contactableFoot.getSoleFrame();
         String namePrefix = soleFrame.getName() + "DesiredCoP";
         YoFramePoint2D yoDesiredCenterOfPressure = new YoFramePoint2D(namePrefix, soleFrame, registry);
         AlphaFilteredYoFramePoint2d yoFilteredDesiredCenterOfPressure = AlphaFilteredYoFramePoint2d.createAlphaFilteredYoFramePoint2d("filtered"
               + namePrefix, "", registry, desiredCoPAlpha, yoDesiredCenterOfPressure);
         footDesiredCenterOfPressures.put(contactableFoot, yoDesiredCenterOfPressure);
         filteredFootDesiredCenterOfPressures.put(contactableFoot, yoFilteredDesiredCenterOfPressure);
      }

      if (updatables != null)
      {
         this.updatables.addAll(updatables);
      }

      double coefficientOfFriction = 1.0; // TODO: magic number...

      for (RobotSide robotSide : RobotSide.values)
      {
         ContactableFoot contactableFoot = feet.get(robotSide);
         RigidBodyBasics rigidBody = contactableFoot.getRigidBody();
         YoPlaneContactState contactState = new YoPlaneContactState(contactableFoot.getSoleFrame().getName(), rigidBody, contactableFoot.getSoleFrame(),
                                                                    contactableFoot.getContactPoints2d(), coefficientOfFriction, registry);

         footContactStates.put(robotSide, contactState);
         previousFootContactPoints.get(robotSide).copyFromListAndTrimSize(contactableFoot.getContactPoints2d());

         FrameConvexPolygon2D defaultFootPolygon = new FrameConvexPolygon2D(FrameVertex2DSupplier.asFrameVertex2DSupplier(contactableFoot.getContactPoints2d()));
         defaultFootPolygons.put(robotSide, defaultFootPolygon);
      }

      controlledJoints = computeJointsToOptimizeFor(fullRobotModel, jointsToIgnore);
      controlledOneDoFJoints = MultiBodySystemTools.filterJoints(controlledJoints, OneDoFJointBasics.class);

      if (yoGraphicsListRegistry != null)
      {
         ArrayList<YoPlaneContactState> planeContactStateList = new ArrayList<>();
         for (RobotSide robotSide : RobotSide.values)
            planeContactStateList.add(footContactStates.get(robotSide));
         ContactPointVisualizer contactPointVisualizer = new ContactPointVisualizer(planeContactStateList, yoGraphicsListRegistry, registry);
         addUpdatable(contactPointVisualizer);
      }

      yoCoPError = new SideDependentList<YoFrameVector2D>();
      xSignsForCoPControl = new SideDependentList<Double>();
      ySignsForCoPControl = new SideDependentList<Double>();
      copControlScales = new SideDependentList<YoDouble>();

      for (RobotSide robotSide : RobotSide.values())
      {
         OneDoFJointBasics anklePitchJoint = fullRobotModel.getLegJoint(robotSide, LegJointName.ANKLE_PITCH);
         OneDoFJointBasics ankleRollJoint = fullRobotModel.getLegJoint(robotSide, LegJointName.ANKLE_ROLL);

         FrameVector3DReadOnly pitchJointAxis;
         FrameVector3DReadOnly rollJointAxis;
         if (anklePitchJoint != null)
         {
            pitchJointAxis = anklePitchJoint.getJointAxis();
            xSignsForCoPControl.put(robotSide, pitchJointAxis.getY());
         }
         else
         {
            xSignsForCoPControl.put(robotSide, 0.0);
         }
         if (ankleRollJoint != null)
         {
            rollJointAxis = ankleRollJoint.getJointAxis();
            ySignsForCoPControl.put(robotSide, rollJointAxis.getY());
         }
         else
         {
            ySignsForCoPControl.put(robotSide, 0.0);
         }

         copControlScales.put(robotSide, new YoDouble(robotSide.getCamelCaseNameForStartOfExpression() + "CoPControlScale", registry));
      }

      minZForceForCoPControlScaling = 0.20 * totalMass * gravityZ;

      alphaCoPControl.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(16.0, controlDT));
      maxAnkleTorqueCoPControl.set(10.0);

      for (RobotSide robotSide : RobotSide.values)
      {
         yoCoPError.put(robotSide,
                        new YoFrameVector2D(robotSide.getCamelCaseNameForStartOfExpression() + "FootCoPError", feet.get(robotSide).getSoleFrame(), registry));

         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
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
            RigidBodyBasics measurementLink = wristForceSensor.getMeasurementLink();
            wristForceSensorMeasurementFrames.put(robotSide, measurementFrame);

            String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
            String namePrefix = sidePrefix + "WristSensor";
            wristRawMeasuredForces.put(robotSide, new YoFrameVector3D(namePrefix + "Force", measurementFrame, registry));
            wristRawMeasuredTorques.put(robotSide, new YoFrameVector3D(namePrefix + "Torque", measurementFrame, registry));
            wristForcesHandWeightCancelled.put(robotSide, new YoFrameVector3D(namePrefix + "ForceHandWeightCancelled", measurementFrame, registry));
            wristTorquesHandWeightCancelled.put(robotSide, new YoFrameVector3D(namePrefix + "TorqueHandWeightCancelled", measurementFrame, registry));

            CenterOfMassReferenceFrame handCoMFrame = new CenterOfMassReferenceFrame(sidePrefix + "HandCoMFrame", measurementFrame, measurementLink);
            handCenterOfMassFrames.put(robotSide, handCoMFrame);
            YoDouble handMass = new YoDouble(sidePrefix + "HandTotalMass", registry);
            handsMass.put(robotSide, handMass);
            handMass.set(TotalMassCalculator.computeSubTreeMass(measurementLink));
         }
      }

      String graphicListName = getClass().getSimpleName();
      if (yoGraphicsListRegistry != null)
      {
         YoArtifactPosition copViz = new YoArtifactPosition("Controller CoP", yoCenterOfPressure.getYoX(), yoCenterOfPressure.getYoY(), GraphicType.DIAMOND,
                                                            Color.BLACK, 0.005);
         yoGraphicsListRegistry.registerArtifact(graphicListName, copViz);
      }
      yoCenterOfPressure.setToNaN();

      this.totalMass.set(totalMass);
      momentumCalculator = new MomentumCalculator(fullRobotModel.getElevator().subtreeArray());
      yoAngularMomentum = new YoFrameVector3D("AngularMomentum", centerOfMassFrame, registry);
      YoDouble alpha = new YoDouble("filteredAngularMomentumAlpha", registry);
      alpha.set(0.95); // switch to break frequency and move to walking parameters
      filteredYoAngularMomentum = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector("filteredAngularMomentum", "", registry, alpha,
                                                                                              yoAngularMomentum);
      momentumGain.set(0.0);

      attachControllerFailureListener(fallingDirection -> controllerFailed.set(true));
   }

   public static JointBasics[] computeJointsToOptimizeFor(FullHumanoidRobotModel fullRobotModel, JointBasics... jointsToRemove)
   {
      List<JointBasics> joints = new ArrayList<JointBasics>();
      JointBasics[] allJoints = MultiBodySystemTools.collectSupportAndSubtreeJoints(fullRobotModel.getRootJoint().getSuccessor());
      joints.addAll(Arrays.asList(allJoints));

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         if (hand != null)
         {
            List<JointBasics> fingerJoints = Arrays.asList(MultiBodySystemTools.collectSubtreeJoints(hand));
            joints.removeAll(fingerJoints);
         }
      }

      if (jointsToRemove != null)
      {
         for (JointBasics joint : jointsToRemove)
         {
            joints.remove(joint);
         }
      }

      return joints.toArray(new JointBasics[joints.size()]);
   }

   public SideDependentList<YoPlaneContactState> getFootContactStates()
   {
      return footContactStates;
   }

   public void update()
   {
      referenceFrames.updateFrames();

      if (referenceFramesVisualizer != null)
         referenceFramesVisualizer.update();

      computeCop();
      computeCapturePoint();
      updateBipedSupportPolygons();
      readWristSensorData();

      computeAngularMomentum();

      for (int i = 0; i < updatables.size(); i++)
         updatables.get(i).update(yoTime.getDoubleValue());

      for (RobotSide robotSide : RobotSide.values)
         footSwitches.get(robotSide).updateCoP();
   }

   private final FramePoint2D tempFootCop2d = new FramePoint2D();
   private final FramePoint3D tempFootCop = new FramePoint3D();
   private final Wrench tempFootWrench = new Wrench();

   private void computeCop()
   {
      double force = 0.0;
      centerOfPressure.setToZero(worldFrame);
      for (RobotSide robotSide : RobotSide.values)
      {
         footSwitches.get(robotSide).computeAndPackCoP(tempFootCop2d);
         if (tempFootCop2d.containsNaN())
            continue;
         footSwitches.get(robotSide).computeAndPackFootWrench(tempFootWrench);
         double footForce = tempFootWrench.getLinearPartZ();
         force += footForce;
         tempFootCop.setIncludingFrame(tempFootCop2d.getReferenceFrame(), tempFootCop2d.getX(), tempFootCop2d.getY(), 0.0);
         tempFootCop.changeFrame(worldFrame);
         tempFootCop.scale(footForce);
         centerOfPressure.add(tempFootCop.getX(), tempFootCop.getY());
      }
      centerOfPressure.scale(1.0 / force);
      yoCenterOfPressure.set(centerOfPressure);
   }

   public void updateBipedSupportPolygons()
   {
      bipedSupportPolygons.updateUsingContactStates(footContactStates);
   }

   private final FramePoint2DBasics capturePoint2d = new FramePoint2D(worldFrame);

   private void computeCapturePoint()
   {
      capturePointCalculator.compute(capturePoint2d, omega0.getValue());
      capturePoint2d.changeFrame(yoCapturePoint.getReferenceFrame());
      yoCapturePoint.set(capturePoint2d, 0.0);
   }

   private final FrameVector3D angularMomentum = new FrameVector3D();
   private final Momentum robotMomentum = new Momentum();

   private void computeAngularMomentum()
   {
      robotMomentum.setToZero(centerOfMassFrame);
      momentumCalculator.computeAndPack(robotMomentum);
      angularMomentum.setIncludingFrame(robotMomentum.getAngularPart());
      yoAngularMomentum.set(angularMomentum);
      filteredYoAngularMomentum.update();
   }

   private final FramePoint2D localDesiredCapturePoint = new FramePoint2D();
   private final YoDouble momentumGain = new YoDouble("MomentumGain", registry);

   public void getAdjustedDesiredCapturePoint(FramePoint2D desiredCapturePoint, FramePoint2D adjustedDesiredCapturePoint)
   {
      angularMomentum.set(filteredYoAngularMomentum);
      ReferenceFrame comFrame = angularMomentum.getReferenceFrame();
      localDesiredCapturePoint.setIncludingFrame(desiredCapturePoint);
      localDesiredCapturePoint.changeFrameAndProjectToXYPlane(comFrame);

      double scaleFactor = momentumGain.getDoubleValue() * omega0.getDoubleValue() / (totalMass.getDoubleValue() * gravity);

      adjustedDesiredCapturePoint.setIncludingFrame(comFrame, -angularMomentum.getY(), angularMomentum.getX());
      adjustedDesiredCapturePoint.scale(scaleFactor);
      adjustedDesiredCapturePoint.add(localDesiredCapturePoint);
      adjustedDesiredCapturePoint.changeFrameAndProjectToXYPlane(desiredCapturePoint.getReferenceFrame());
   }

   public void getCapturePoint(FramePoint2D capturePointToPack)
   {
      capturePointToPack.setIncludingFrame(yoCapturePoint);
   }

   public void getCapturePoint(FramePoint3D capturePointToPack)
   {
      capturePointToPack.setIncludingFrame(yoCapturePoint);
   }

   private final FramePoint2D copDesired = new FramePoint2D();
   private final FramePoint2D copActual = new FramePoint2D();
   private final FrameVector2D copError = new FrameVector2D();
   private final Wrench footWrench = new Wrench();
   private final FrameVector3D footForceVector = new FrameVector3D();

   private final YoBoolean enableHighCoPDampingForShakies = new YoBoolean("enableHighCoPDampingForShakies", registry);
   private final YoBoolean isCoPTrackingBad = new YoBoolean("isCoPTrackingBad", registry);
   private final YoDouble highCoPDampingErrorTrigger = new YoDouble("highCoPDampingErrorTrigger", registry);
   private final YoDouble highCoPDampingStartTime = new YoDouble("highCoPDampingStartTime", registry);
   private final YoDouble highCoPDampingDuration = new YoDouble("highCoPDampingDuration", registry);

   public boolean estimateIfHighCoPDampingNeeded(SideDependentList<FramePoint2D> desiredCoPs)
   {
      if (!enableHighCoPDampingForShakies.getBooleanValue())
         return false;

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

         footSwitch.computeAndPackFootWrench(footWrench);
         footForceVector.setIncludingFrame(footWrench.getLinearPart());
         footForceVector.changeFrame(ReferenceFrame.getWorldFrame());

         if (footForceVector.getZ() > minZForceForCoPControlScaling
               && yoCoPErrorMagnitude.get(robotSide).getDoubleValue() > highCoPDampingErrorTrigger.getDoubleValue())
         {
            atLeastOneFootWithBadCoPControl = true;
         }
      }

      isCoPTrackingBad.set(atLeastOneFootWithBadCoPControl);

      boolean isCoPDampened = yoTime.getDoubleValue() - highCoPDampingStartTime.getDoubleValue() <= highCoPDampingDuration.getDoubleValue();

      if (atLeastOneFootWithBadCoPControl && !isCoPDampened)
      {
         highCoPDampingStartTime.set(yoTime.getDoubleValue());
         isCoPDampened = true;
      }

      return isCoPDampened;
   }

   public void setHighCoPDampingParameters(boolean enable, double duration, double copErrorThreshold)
   {
      enableHighCoPDampingForShakies.set(enable);
      highCoPDampingDuration.set(duration);
      highCoPDampingErrorTrigger.set(copErrorThreshold);
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
      update();

      // This removes rate objectives from the inverse dynamics QP solver in case an old solution is around from a fall and the
      // robot is reinitialized.
      for (RobotSide robotSide : RobotSide.values)
      {
         footContactStates.get(robotSide).notifyContactStateHasChanged();
      }
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

         tempWristForce.setIncludingFrame(wristTempWrench.getLinearPart());
         tempWristTorque.setIncludingFrame(wristTempWrench.getAngularPart());

         wristRawMeasuredForces.get(robotSide).setMatchingFrame(tempWristForce);
         wristRawMeasuredTorques.get(robotSide).setMatchingFrame(tempWristTorque);

         cancelHandWeight(robotSide, wristTempWrench, measurementFrame);

         tempWristForce.setIncludingFrame(wristTempWrench.getLinearPart());
         tempWristTorque.setIncludingFrame(wristTempWrench.getAngularPart());

         wristForcesHandWeightCancelled.get(robotSide).setMatchingFrame(tempWristForce);
         wristTorquesHandWeightCancelled.get(robotSide).setMatchingFrame(tempWristTorque);
      }
   }

   private void cancelHandWeight(RobotSide robotSide, Wrench wrenchToSubstractHandWeightTo, ReferenceFrame measurementFrame)
   {
      CenterOfMassReferenceFrame handCoMFrame = handCenterOfMassFrames.get(robotSide);
      handCoMFrame.update();
      tempWristForce.setIncludingFrame(worldFrame, 0.0, 0.0, -handsMass.get(robotSide).getDoubleValue() * gravity);
      tempWristForce.changeFrame(handCoMFrame);
      wristWrenchDueToGravity.setToZero(measurementFrame, handCoMFrame);
      wristWrenchDueToGravity.getLinearPart().set(tempWristForce);
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

   public void setDesiredCenterOfPressure(ContactablePlaneBody contactablePlaneBody, FramePoint2DReadOnly desiredCoP)
   {
      YoFramePoint2D cop = footDesiredCenterOfPressures.get(contactablePlaneBody);
      if (cop != null)
      {
         cop.set(desiredCoP);

         if (!cop.containsNaN())
            filteredFootDesiredCenterOfPressures.get(contactablePlaneBody).update();
         else
            filteredFootDesiredCenterOfPressures.get(contactablePlaneBody).reset();
      }
   }

   public void getDesiredCenterOfPressure(ContactablePlaneBody contactablePlaneBody, FramePoint2D desiredCoPToPack)
   {
      desiredCoPToPack.setIncludingFrame(footDesiredCenterOfPressures.get(contactablePlaneBody));
   }

   public void getFilteredDesiredCenterOfPressure(ContactablePlaneBody contactablePlaneBody, FramePoint2D desiredCoPToPack)
   {
      desiredCoPToPack.setIncludingFrame(filteredFootDesiredCenterOfPressures.get(contactablePlaneBody));
   }

   public void updateContactPointsForUpcomingFootstep(Footstep nextFootstep)
   {
      RobotSide robotSide = nextFootstep.getRobotSide();

      List<Point2D> predictedContactPoints = nextFootstep.getPredictedContactPoints();

      if ((predictedContactPoints != null) && (!predictedContactPoints.isEmpty()))
      {
         setFootPlaneContactPoints(robotSide, predictedContactPoints);
      }
      else
      {
         resetFootPlaneContactPoint(robotSide);
      }
   }

   public void resetFootPlaneContactPoint(RobotSide robotSide)
   {
      ContactablePlaneBody foot = feet.get(robotSide);
      YoPlaneContactState footContactState = footContactStates.get(robotSide);
      List<FramePoint2D> defaultContactPoints = foot.getContactPoints2d();
      previousFootContactPoints.get(robotSide).copyFromListAndTrimSize(defaultContactPoints);
      footContactState.setContactFramePoints(defaultContactPoints);
   }

   private void setFootPlaneContactPoints(RobotSide robotSide, List<Point2D> predictedContactPoints)
   {
      YoPlaneContactState footContactState = footContactStates.get(robotSide);
      footContactState.getContactFramePoint2dsInContact(previousFootContactPoints.get(robotSide));
      footContactState.setContactPoints(predictedContactPoints);
   }

   public void restorePreviousFootContactPoints(RobotSide robotSide)
   {
      YoPlaneContactState footContactState = footContactStates.get(robotSide);
      footContactState.setContactFramePoints(previousFootContactPoints.get(robotSide));
   }

   public void setFootContactCoefficientOfFriction(RobotSide robotSide, double coefficientOfFriction)
   {
      YoPlaneContactState yoPlaneContactState = footContactStates.get(robotSide);
      yoPlaneContactState.setCoefficientOfFriction(coefficientOfFriction);
   }

   public void setFootContactStateNormalContactVector(RobotSide robotSide, FrameVector3D normalContactVector)
   {
      YoPlaneContactState yoPlaneContactState = footContactStates.get(robotSide);
      yoPlaneContactState.setContactNormalVector(normalContactVector);
   }

   public void setFootContactState(RobotSide robotSide, boolean[] newContactPointStates, FrameVector3D normalContactVector)
   {
      YoPlaneContactState yoPlaneContactState = footContactStates.get(robotSide);
      yoPlaneContactState.setContactPointsInContact(newContactPointStates);
      yoPlaneContactState.setContactNormalVector(normalContactVector);
   }

   public void setFootContactStateFullyConstrained(RobotSide robotSide)
   {
      YoPlaneContactState yoPlaneContactState = footContactStates.get(robotSide);
      yoPlaneContactState.setFullyConstrained();
   }

   public void setFootContactStateFree(RobotSide robotSide)
   {
      YoPlaneContactState yoPlaneContactState = footContactStates.get(robotSide);
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

   public YoDouble getYoTime()
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

   public SideDependentList<ContactableFoot> getContactableFeet()
   {
      return feet;
   }

   public List<? extends ContactablePlaneBody> getContactablePlaneBodies()
   {
      return contactableBodies;
   }

   public ContactablePlaneBody getContactableBody(RigidBodyBasics body)
   {
      for (ContactablePlaneBody contactableBody : contactableBodies)
         if (contactableBody.getRigidBody().getName().equals(body.getName()))
            return contactableBody;
      return null;
   }

   public YoPlaneContactState getFootContactState(RobotSide robotSide)
   {
      return footContactStates.get(robotSide);
   }

   public void clearContacts()
   {
      for (RobotSide robotSide : RobotSide.values)
         footContactStates.get(robotSide).clear();
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

   public BipedSupportPolygons getBipedSupportPolygons()
   {
      return bipedSupportPolygons;
   }

   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return yoGraphicsListRegistry;
   }

   public JointBasics[] getControlledJoints()
   {
      return controlledJoints;
   }

   public OneDoFJointBasics[] getControlledOneDoFJoints()
   {
      return controlledOneDoFJoints;
   }

   public YoBoolean getControllerFailedBoolean()
   {
      return controllerFailed;
   }

   public void attachControllerFailureListener(ControllerFailureListener listener)
   {
      this.controllerFailureListeners.add(listener);
   }

   public void reportControllerFailureToListeners(FrameVector2D fallingDirection)
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

   public void detachRobotMotionStatusChangedListener(RobotMotionStatusChangedListener listener)
   {
      robotMotionStatusChangedListeners.remove(listener);
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

      tempWristForce.setIncludingFrame(wristRawMeasuredForces.get(robotSide));
      tempWristTorque.setIncludingFrame(wristRawMeasuredTorques.get(robotSide));
      ReferenceFrame measurementFrames = wristForceSensorMeasurementFrames.get(robotSide);
      wrenchToPack.setToZero(measurementFrames, measurementFrames);
      wrenchToPack.getLinearPart().set(tempWristForce);
      wrenchToPack.getAngularPart().set(tempWristTorque);
   }

   public void getWristMeasuredWrenchHandWeightCancelled(Wrench wrenchToPack, RobotSide robotSide)
   {
      if (wristForcesHandWeightCancelled == null || wristTorquesHandWeightCancelled == null)
         return;

      tempWristForce.setIncludingFrame(wristForcesHandWeightCancelled.get(robotSide));
      tempWristTorque.setIncludingFrame(wristTorquesHandWeightCancelled.get(robotSide));
      ReferenceFrame measurementFrames = wristForceSensorMeasurementFrames.get(robotSide);
      wrenchToPack.setToZero(measurementFrames, measurementFrames);
      wrenchToPack.getLinearPart().set(tempWristForce);
      wrenchToPack.getAngularPart().set(tempWristTorque);
   }

   public void getDefaultFootPolygon(RobotSide robotSide, FrameConvexPolygon2D polygonToPack)
   {
      polygonToPack.set(defaultFootPolygons.get(robotSide));
   }

   public SideDependentList<FrameConvexPolygon2D> getDefaultFootPolygons()
   {
      return defaultFootPolygons;
   }

   private final FramePoint3D tempPosition = new FramePoint3D();

   public void resetFootSupportPolygon(RobotSide robotSide)
   {
      YoPlaneContactState contactState = footContactStates.get(robotSide);
      List<YoContactPoint> contactPoints = contactState.getContactPoints();
      FrameConvexPolygon2D defaultSupportPolygon = defaultFootPolygons.get(robotSide);

      for (int i = 0; i < defaultSupportPolygon.getNumberOfVertices(); i++)
      {
         tempPosition.setIncludingFrame(defaultSupportPolygon.getVertex(i), 0.0);
         contactPoints.get(i).setPosition(tempPosition);
      }
      contactState.notifyContactStateHasChanged();
   }

   public double getOmega0()
   {
      return omega0.getDoubleValue();
   }

   public void getCoP(FramePoint3D copToPack)
   {
      copToPack.setIncludingFrame(yoCenterOfPressure, 0.0);
   }

   public void getCoP(FramePoint2D copToPack)
   {
      copToPack.setIncludingFrame(yoCenterOfPressure);
   }

   public void getAngularMomentum(FrameVector3D upperBodyAngularMomentumToPack)
   {
      upperBodyAngularMomentumToPack.setIncludingFrame(angularMomentum);
   }

   public ReferenceFrameHashCodeResolver getReferenceFrameHashCodeResolver()
   {
      return referenceFrameHashCodeResolver;
   }

   public void setWalkingMessageHandler(WalkingMessageHandler walkingMessageHandler)
   {
      this.walkingMessageHandler = walkingMessageHandler;
   }

   public WalkingMessageHandler getWalkingMessageHandler()
   {
      return walkingMessageHandler;
   }

}
