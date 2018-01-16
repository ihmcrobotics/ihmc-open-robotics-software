package us.ihmc.commonWalkingControlModules.momentumBasedController;

import static us.ihmc.graphicsDescription.appearance.YoAppearance.*;
import static us.ihmc.robotics.lists.FrameTuple2dArrayList.*;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactPointVisualizer;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPlane;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPolygons;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonHumanoidReferenceFramesVisualizer;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.controllers.ControllerFailureListener;
import us.ihmc.robotics.controllers.ControllerStateChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.lists.FrameTuple2dArrayList;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFramePoint2d;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculatorListener;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.Momentum;
import us.ihmc.robotics.screwTheory.MomentumCalculator;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.CenterOfMassDataHolderReadOnly;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.sensorProcessing.model.RobotMotionStatusChangedListener;

public class HighLevelHumanoidControllerToolbox
{
   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final ReferenceFrame centerOfMassFrame;
   private final FullHumanoidRobotModel fullRobotModel;
   private final CenterOfMassJacobian centerOfMassJacobian;
   private final CommonHumanoidReferenceFrames referenceFrames;
   private final CommonHumanoidReferenceFramesVisualizer referenceFramesVisualizer;

   protected final SideDependentList<ContactableFoot> feet;
   protected final List<ContactablePlaneBody> contactableBodies;
   private final SideDependentList<YoPlaneContactState> footContactStates = new SideDependentList<>();
   private final SideDependentList<FrameConvexPolygon2d> defaultFootPolygons = new SideDependentList<>();

   private final ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver;
   private final Collection<ReferenceFrame> trajectoryFrames;

   protected final LinkedHashMap<ContactablePlaneBody, YoFramePoint2d> footDesiredCenterOfPressures = new LinkedHashMap<>();
   private final YoDouble desiredCoPAlpha;
   private final LinkedHashMap<ContactablePlaneBody, AlphaFilteredYoFramePoint2d> filteredFootDesiredCenterOfPressures = new LinkedHashMap<>();

   private final ArrayList<Updatable> updatables = new ArrayList<Updatable>();
   private final YoDouble yoTime;
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
   private final FrameVector3D tempWristForce = new FrameVector3D();
   private final FrameVector3D tempWristTorque = new FrameVector3D();

   private final SideDependentList<YoDouble> handsMass;

   private final SideDependentList<FootSwitchInterface> footSwitches;
   private final SideDependentList<ForceSensorDataReadOnly> wristForceSensors;
   private final YoDouble alphaCoPControl = new YoDouble("alphaCoPControl", registry);
   private final YoDouble maxAnkleTorqueCoPControl = new YoDouble("maxAnkleTorqueCoPControl", registry);

   private final SideDependentList<Double> xSignsForCoPControl, ySignsForCoPControl;
   private final double minZForceForCoPControlScaling;

   private final SideDependentList<YoFrameVector2d> yoCoPError;
   private final SideDependentList<YoDouble> yoCoPErrorMagnitude = new SideDependentList<YoDouble>(new YoDouble("leftFootCoPErrorMagnitude",
                                                                                                                                        registry),
                                                                                                                   new YoDouble("rightFootCoPErrorMagnitude",
                                                                                                                                        registry));
   private final SideDependentList<YoDouble> copControlScales;

   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private final InverseDynamicsJoint[] controlledJoints;

   private final SideDependentList<Wrench> handWrenches = new SideDependentList<>();

   private final ArrayList<ControllerFailureListener> controllerFailureListeners = new ArrayList<>();
   private final ArrayList<ControllerStateChangedListener> controllerStateChangedListeners = new ArrayList<>();
   private final ArrayList<RobotMotionStatusChangedListener> robotMotionStatusChangedListeners = new ArrayList<>();

   private final BipedSupportPolygons bipedSupportPolygons;
   private final ICPControlPolygons icpControlPolygons;
   private final ICPControlPlane icpControlPlane;

   private final SideDependentList<FrameTuple2dArrayList<FramePoint2D>> previousFootContactPoints = new SideDependentList<>(createFramePoint2dArrayList(),
                                                                                                                            createFramePoint2dArrayList());

   protected final YoFramePoint yoCapturePoint = new YoFramePoint("capturePoint", worldFrame, registry);
   private final YoDouble omega0 = new YoDouble("omega0", registry);

   private final MomentumCalculator momentumCalculator;
   private final YoFrameVector yoAngularMomentum;
   private final AlphaFilteredYoFrameVector filteredYoAngularMomentum;
   private final YoDouble totalMass = new YoDouble("TotalMass", registry);

   private final FramePoint2D centerOfPressure = new FramePoint2D();
   private final YoFramePoint2d yoCenterOfPressure = new YoFramePoint2d("CenterOfPressure", worldFrame, registry);

   private final CenterOfMassDataHolderReadOnly centerOfMassDataHolder;
   private WalkingMessageHandler walkingMessageHandler;
   
   private final YoBoolean controllerFailed = new YoBoolean("controllerFailed", registry);

   public HighLevelHumanoidControllerToolbox(FullHumanoidRobotModel fullRobotModel, CommonHumanoidReferenceFrames referenceFrames,
                                             SideDependentList<FootSwitchInterface> footSwitches, CenterOfMassDataHolderReadOnly centerOfMassDataHolder,
                                             SideDependentList<ForceSensorDataReadOnly> wristForceSensors, YoDouble yoTime, double gravityZ,
                                             double omega0, SideDependentList<ContactableFoot> feet, double controlDT,
                                             ArrayList<Updatable> updatables, List<ContactablePlaneBody> contactableBodies,
                                             YoGraphicsListRegistry yoGraphicsListRegistry, InverseDynamicsJoint... jointsToIgnore)
   {
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;

      this.centerOfMassDataHolder = centerOfMassDataHolder;
      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();

      this.icpControlPlane = new ICPControlPlane(this.omega0, centerOfMassFrame, gravityZ, registry);
      SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<>(referenceFrames.getAnkleZUpReferenceFrames());
      ReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
      SideDependentList<ReferenceFrame> soleZUpFrames = new SideDependentList<>(referenceFrames.getSoleZUpFrames());
      bipedSupportPolygons = new BipedSupportPolygons(ankleZUpFrames, midFeetZUpFrame, soleZUpFrames, registry, yoGraphicsListRegistry);
      icpControlPolygons = new ICPControlPolygons(icpControlPlane, midFeetZUpFrame, registry, yoGraphicsListRegistry);

      this.footSwitches = footSwitches;
      this.wristForceSensors = wristForceSensors;

      referenceFrameHashCodeResolver = new ReferenceFrameHashCodeResolver(fullRobotModel, referenceFrames);
      trajectoryFrames = new ArrayList<ReferenceFrame>();
      trajectoryFrames.addAll(referenceFrameHashCodeResolver.getAllReferenceFrames());

      MathTools.checkIntervalContains(gravityZ, 0.0, Double.POSITIVE_INFINITY);

      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = referenceFrames;
      this.controlDT = controlDT;
      this.gravity = gravityZ;
      this.yoTime = yoTime;
      this.omega0.set(omega0);

      this.centerOfMassJacobian = new CenterOfMassJacobian(fullRobotModel.getElevator());
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

      RigidBody elevator = fullRobotModel.getElevator();
      double totalMass = TotalMassCalculator.computeSubTreeMass(elevator);

      desiredCoPAlpha = new YoDouble("desiredCoPAlpha", registry);
      desiredCoPAlpha.set(0.9);
      for (RobotSide robotSide : RobotSide.values)
      {
         ContactableFoot contactableFoot = feet.get(robotSide);
         ReferenceFrame soleFrame = contactableFoot.getSoleFrame();
         String namePrefix = soleFrame.getName() + "DesiredCoP";
         YoFramePoint2d yoDesiredCenterOfPressure = new YoFramePoint2d(namePrefix, soleFrame, registry);
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
         RigidBody rigidBody = contactableFoot.getRigidBody();
         YoPlaneContactState contactState = new YoPlaneContactState(contactableFoot.getSoleFrame().getName(), rigidBody, contactableFoot.getSoleFrame(),
                                                                    contactableFoot.getContactPoints2d(), coefficientOfFriction, registry);

         footContactStates.put(robotSide, contactState);
         previousFootContactPoints.get(robotSide).copyFromListAndTrimSize(contactableFoot.getContactPoints2d());

         FrameConvexPolygon2d defaultFootPolygon = new FrameConvexPolygon2d(contactableFoot.getContactPoints2d());
         defaultFootPolygons.put(robotSide, defaultFootPolygon);
      }

      controlledJoints = computeJointsToOptimizeFor(fullRobotModel, jointsToIgnore);

      if (yoGraphicsListRegistry != null)
      {
         ArrayList<YoPlaneContactState> planeContactStateList = new ArrayList<>();
         for (RobotSide robotSide : RobotSide.values)
            planeContactStateList.add(footContactStates.get(robotSide));
         ContactPointVisualizer contactPointVisualizer = new ContactPointVisualizer(planeContactStateList, yoGraphicsListRegistry, registry);
         addUpdatable(contactPointVisualizer);
      }

      yoCoPError = new SideDependentList<YoFrameVector2d>();
      xSignsForCoPControl = new SideDependentList<Double>();
      ySignsForCoPControl = new SideDependentList<Double>();
      copControlScales = new SideDependentList<YoDouble>();

      for (RobotSide robotSide : RobotSide.values())
      {
         OneDoFJoint anklePitchJoint = fullRobotModel.getLegJoint(robotSide, LegJointName.ANKLE_PITCH);
         OneDoFJoint ankleRollJoint = fullRobotModel.getLegJoint(robotSide, LegJointName.ANKLE_ROLL);

         FrameVector3D pitchJointAxis;
         FrameVector3D rollJointAxis;
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
            YoDouble handMass = new YoDouble(sidePrefix + "HandTotalMass", registry);
            handsMass.put(robotSide, handMass);
            handMass.set(TotalMassCalculator.computeSubTreeMass(measurementLink));
         }
      }

      String graphicListName = getClass().getSimpleName();
      if (yoGraphicsListRegistry != null)
      {
         YoGraphicPosition capturePointViz = new YoGraphicPosition("Capture Point", yoCapturePoint, 0.01, Blue(), GraphicType.BALL_WITH_ROTATED_CROSS);
         yoGraphicsListRegistry.registerArtifact(graphicListName, capturePointViz.createArtifact());

         YoArtifactPosition copViz = new YoArtifactPosition("Controller CoP", yoCenterOfPressure.getYoX(), yoCenterOfPressure.getYoY(), GraphicType.DIAMOND,
                                                            Color.BLACK, 0.005);
         yoGraphicsListRegistry.registerArtifact(graphicListName, copViz);
      }
      yoCenterOfPressure.setToNaN();

      this.totalMass.set(totalMass);
      momentumCalculator = new MomentumCalculator(ScrewTools.computeSubtreeSuccessors(fullRobotModel.getElevator()));
      yoAngularMomentum = new YoFrameVector("AngularMomentum", centerOfMassFrame, registry);
      YoDouble alpha = new YoDouble("filteredAngularMomentumAlpha", registry);
      alpha.set(0.95); // switch to break frequency and move to walking parameters
      filteredYoAngularMomentum = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector("filteredAngularMomentum", "", registry, alpha,
                                                                                              yoAngularMomentum);
      momentumGain.set(0.0);
      
      attachControllerFailureListener(new ControllerFailureListener()
      {
         @Override
         public void controllerFailed(FrameVector2D fallingDirection)
         {
            reportControllerFailed();
         }
      });
   }

   public void reportControllerFailed()
   {
      controllerFailed.set(true);
   }
   
   public YoBoolean getControllerFailedBoolean()
   {
      return controllerFailed;
   }
   
   public static InverseDynamicsJoint[] computeJointsToOptimizeFor(FullHumanoidRobotModel fullRobotModel, InverseDynamicsJoint... jointsToRemove)
   {
      List<InverseDynamicsJoint> joints = new ArrayList<InverseDynamicsJoint>();
      InverseDynamicsJoint[] allJoints = ScrewTools.computeSupportAndSubtreeJoints(fullRobotModel.getRootJoint().getSuccessor());
      joints.addAll(Arrays.asList(allJoints));

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody hand = fullRobotModel.getHand(robotSide);
         if (hand != null)
         {
            List<InverseDynamicsJoint> fingerJoints = Arrays.asList(ScrewTools.computeSubtreeJoints(hand));
            joints.removeAll(fingerJoints);
         }
      }

      if (jointsToRemove != null)
      {
         for (InverseDynamicsJoint joint : jointsToRemove)
         {
            joints.remove(joint);
         }
      }

      return joints.toArray(new InverseDynamicsJoint[joints.size()]);
   }

   public void setInverseDynamicsCalculatorListener(InverseDynamicsCalculatorListener inverseDynamicsCalculatorListener)
   {
      throw new RuntimeException("Sylvain was there.");
   }

   public SideDependentList<YoPlaneContactState> getFootContactStates()
   {
      return footContactStates;
   }

   public void update()
   {
      referenceFrames.updateFrames();
      centerOfMassJacobian.compute();
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
      icpControlPolygons.updateUsingContactStates(footContactStates);
   }

   private final FramePoint2D capturePoint2d = new FramePoint2D();
   private final FramePoint3D centerOfMassPosition = new FramePoint3D();
   private final FrameVector3D centerOfMassVelocity = new FrameVector3D();
   private final FramePoint2D centerOfMassPosition2d = new FramePoint2D();
   private final FrameVector2D centerOfMassVelocity2d = new FrameVector2D();

   private void computeCapturePoint()
   {
      centerOfMassPosition.setToZero(centerOfMassFrame);

      if (centerOfMassDataHolder != null)
      {
         centerOfMassDataHolder.getCenterOfMassVelocity(centerOfMassVelocity);
      }
      else
      {
         centerOfMassJacobian.getCenterOfMassVelocity(centerOfMassVelocity);
      }

      centerOfMassPosition.changeFrame(worldFrame);
      centerOfMassVelocity.changeFrame(worldFrame);

      centerOfMassPosition2d.setIncludingFrame(centerOfMassPosition);
      centerOfMassVelocity2d.setIncludingFrame(centerOfMassVelocity);

      CapturePointCalculator.computeCapturePoint(capturePoint2d, centerOfMassPosition2d, centerOfMassVelocity2d, omega0.getDoubleValue());

      capturePoint2d.changeFrame(yoCapturePoint.getReferenceFrame());
      yoCapturePoint.set(capturePoint2d, 0.0);
   }

   private final FrameVector3D angularMomentum = new FrameVector3D();
   private final Momentum robotMomentum = new Momentum();

   private void computeAngularMomentum()
   {
      robotMomentum.setToZero(centerOfMassFrame);
      momentumCalculator.computeAndPack(robotMomentum);
      robotMomentum.getAngularPartIncludingFrame(angularMomentum);
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
      yoCapturePoint.getFrameTuple2dIncludingFrame(capturePointToPack);
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
         footWrench.getLinearPartIncludingFrame(footForceVector);
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

   public void setDesiredCenterOfPressure(ContactablePlaneBody contactablePlaneBody, FramePoint2D desiredCoP)
   {
      YoFramePoint2d cop = footDesiredCenterOfPressures.get(contactablePlaneBody);
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
      footDesiredCenterOfPressures.get(contactablePlaneBody).getFrameTuple2dIncludingFrame(desiredCoPToPack);
   }

   public void getFilteredDesiredCenterOfPressure(ContactablePlaneBody contactablePlaneBody, FramePoint2D desiredCoPToPack)
   {
      filteredFootDesiredCenterOfPressures.get(contactablePlaneBody).getFrameTuple2dIncludingFrame(desiredCoPToPack);
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

   private void resetFootPlaneContactPoint(RobotSide robotSide)
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

   public CenterOfMassJacobian getCenterOfMassJacobian()
   {
      return centerOfMassJacobian;
   }

   public SideDependentList<ContactableFoot> getContactableFeet()
   {
      return feet;
   }

   public List<? extends ContactablePlaneBody> getContactablePlaneBodies()
   {
      return contactableBodies;
   }

   public ContactablePlaneBody getContactableBody(RigidBody body)
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

   public ICPControlPolygons getICPControlPolygons()
   {
      return icpControlPolygons;
   }

   public YoGraphicsListRegistry getYoGraphicsListRegistry()
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

   public void getDefaultFootPolygon(RobotSide robotSide, FrameConvexPolygon2d polygonToPack)
   {
      polygonToPack.set(defaultFootPolygons.get(robotSide));
   }

   public SideDependentList<FrameConvexPolygon2d> getDefaultFootPolygons()
   {
      return defaultFootPolygons;
   }

   private final FramePoint3D tempPosition = new FramePoint3D();

   public void resetFootSupportPolygon(RobotSide robotSide)
   {
      YoPlaneContactState contactState = footContactStates.get(robotSide);
      List<YoContactPoint> contactPoints = contactState.getContactPoints();
      FrameConvexPolygon2d defaultSupportPolygon = defaultFootPolygons.get(robotSide);

      for (int i = 0; i < defaultSupportPolygon.getNumberOfVertices(); i++)
      {
         defaultSupportPolygon.getFrameVertexXY(i, tempPosition);
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
      yoCenterOfPressure.getFrameTupleIncludingFrame(copToPack);
   }

   public void getCoP(FramePoint2D copToPack)
   {
      yoCenterOfPressure.getFrameTuple2dIncludingFrame(copToPack);
   }

   public void getAngularMomentum(FrameVector3D upperBodyAngularMomentumToPack)
   {
      upperBodyAngularMomentumToPack.setIncludingFrame(angularMomentum);
   }

   public ReferenceFrameHashCodeResolver getReferenceFrameHashCodeResolver()
   {
      return referenceFrameHashCodeResolver;
   }

   public Collection<ReferenceFrame> getTrajectoryFrames()
   {
      return trajectoryFrames;
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
