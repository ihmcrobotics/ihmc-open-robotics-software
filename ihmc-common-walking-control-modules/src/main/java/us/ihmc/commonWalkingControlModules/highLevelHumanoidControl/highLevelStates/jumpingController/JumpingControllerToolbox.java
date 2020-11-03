package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

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
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointCalculator;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonHumanoidReferenceFramesVisualizer;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.frames.MovingCenterOfMassReferenceFrame;
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
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class JumpingControllerToolbox
{
   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);

   private final YoDouble standingHeight = new YoDouble("StandingHeight", registry);
   private final YoDouble squattingHeight = new YoDouble("SquattingHeight", registry);

   private final MovingCenterOfMassReferenceFrame centerOfMassFrame;

   private final FullHumanoidRobotModel fullRobotModel;
   private final CapturePointCalculator capturePointCalculator;

   private final CommonHumanoidReferenceFrames referenceFrames;
   private final CommonHumanoidReferenceFramesVisualizer referenceFramesVisualizer;

   private final SideDependentList<ContactableFoot> feet;
   private final SideDependentList<YoPlaneContactState> footContactStates = new SideDependentList<>();
   private final SideDependentList<FrameConvexPolygon2D> defaultFootPolygons = new SideDependentList<>();

   private final ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver;

   private final YoDouble finalTransferTime;
   private final ArrayList<Updatable> updatables = new ArrayList<Updatable>();
   private final YoDouble yoTime;
   private final double controlDT;
   private final double gravity;

   private final SideDependentList<FootSwitchInterface> footSwitches;

   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private final JointBasics[] controlledJoints;
   private final OneDoFJointBasics[] controlledOneDoFJoints;
   private final OneDoFJointBasics[] uncontrolledOneDoFJoints;

   private final ArrayList<ControllerFailureListener> controllerFailureListeners = new ArrayList<>();
   private final ArrayList<ControllerStateChangedListener> controllerStateChangedListeners = new ArrayList<>();
   private final ArrayList<RobotMotionStatusChangedListener> robotMotionStatusChangedListeners = new ArrayList<>();

   private final BipedSupportPolygons bipedSupportPolygons;

   private final SideDependentList<FrameTuple2dArrayList<FramePoint2D>> previousFootContactPoints = new SideDependentList<>(createFramePoint2dArrayList(),
                                                                                                                            createFramePoint2dArrayList());

   private final YoFramePoint3D yoCapturePoint = new YoFramePoint3D("capturePoint", worldFrame, registry);

   private final YoDouble omega0 = new YoDouble("omega0", registry);

   private final MomentumCalculator momentumCalculator;
   private final YoFrameVector3D yoAngularMomentum;
   private final AlphaFilteredYoFrameVector filteredYoAngularMomentum;
   private final YoDouble totalMass = new YoDouble("TotalMass", registry);

   private final FramePoint2D centerOfPressure = new FramePoint2D();
   private final YoFramePoint2D yoCenterOfPressure = new YoFramePoint2D("CenterOfPressure", worldFrame, registry);

   private final YoBoolean controllerFailed = new YoBoolean("controllerFailed", registry);

   public JumpingControllerToolbox(FullHumanoidRobotModel fullRobotModel,
                                   CommonHumanoidReferenceFrames referenceFrames,
                                   WalkingControllerParameters walkingControllerParameters,
                                   SideDependentList<? extends FootSwitchInterface> footSwitches, YoDouble yoTime, double gravityZ, double omega0,
                                   SideDependentList<ContactableFoot> feet, double controlDT, List<Updatable> updatables,
                                   YoGraphicsListRegistry yoGraphicsListRegistry,
                                   JointBasics... jointsToIgnore)
   {
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;

      standingHeight.set(1.05);
      squattingHeight.set(0.6);

      finalTransferTime = new YoDouble("finalTransferTime", registry);
      finalTransferTime.set(0.25);

      bipedSupportPolygons = new BipedSupportPolygons(referenceFrames, registry, yoGraphicsListRegistry);

      this.footSwitches = new SideDependentList<>(footSwitches);

      referenceFrameHashCodeResolver = new ReferenceFrameHashCodeResolver(fullRobotModel, referenceFrames);

      centerOfMassFrame = new MovingCenterOfMassReferenceFrame("centerOfMassFrame", worldFrame, fullRobotModel.getElevator());
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
         referenceFramesVisualizer = new CommonHumanoidReferenceFramesVisualizer(referenceFrames,
                                                                                 yoGraphicsListRegistry,
                                                                                 registry);
      }
      else
      {
         referenceFramesVisualizer = null;
      }

      // Initialize the contactable bodies
      this.feet = feet;

      RigidBodyBasics elevator = fullRobotModel.getElevator();
      double totalMass = TotalMassCalculator.computeSubTreeMass(elevator);

      if (updatables != null)
      {
         this.updatables.addAll(updatables);
      }

      double coefficientOfFriction = 1.0; // TODO: magic number...

      for (RobotSide robotSide : RobotSide.values)
      {
         ContactableFoot contactableFoot = feet.get(robotSide);
         RigidBodyBasics rigidBody = contactableFoot.getRigidBody();
         YoPlaneContactState contactState = new YoPlaneContactState(contactableFoot.getSoleFrame().getName(),
                                                                    rigidBody,
                                                                    contactableFoot.getSoleFrame(),
                                                                    contactableFoot.getContactPoints2d(),
                                                                    coefficientOfFriction,
                                                                    registry);

         footContactStates.put(robotSide, contactState);
         previousFootContactPoints.get(robotSide).copyFromListAndTrimSize(contactableFoot.getContactPoints2d());

         FrameConvexPolygon2D defaultFootPolygon = new FrameConvexPolygon2D(FrameVertex2DSupplier.asFrameVertex2DSupplier(contactableFoot.getContactPoints2d()));
         defaultFootPolygons.put(robotSide, defaultFootPolygon);
      }

      controlledJoints = computeJointsToOptimizeFor(fullRobotModel, jointsToIgnore);
      controlledOneDoFJoints = MultiBodySystemTools.filterJoints(controlledJoints, OneDoFJointBasics.class);
      uncontrolledOneDoFJoints = MultiBodySystemTools.filterJoints(jointsToIgnore, OneDoFJointBasics.class);

      if (yoGraphicsListRegistry != null)
      {
         ArrayList<YoPlaneContactState> planeContactStateList = new ArrayList<>();
         for (RobotSide robotSide : RobotSide.values)
            planeContactStateList.add(footContactStates.get(robotSide));
         ContactPointVisualizer contactPointVisualizer = new ContactPointVisualizer(planeContactStateList, yoGraphicsListRegistry, registry);
         addUpdatable(contactPointVisualizer);
      }

      String graphicListName = getClass().getSimpleName();
      if (yoGraphicsListRegistry != null)
      {
         YoArtifactPosition copViz = new YoArtifactPosition("Controller CoP",
                                                            yoCenterOfPressure.getYoX(),
                                                            yoCenterOfPressure.getYoY(),
                                                            GraphicType.DIAMOND,
                                                            Color.BLACK,
                                                            0.005);
         yoGraphicsListRegistry.registerArtifact(graphicListName, copViz);
      }
      yoCenterOfPressure.setToNaN();

      this.totalMass.set(totalMass);
      momentumCalculator = new MomentumCalculator(fullRobotModel.getElevator().subtreeArray());
      yoAngularMomentum = new YoFrameVector3D("AngularMomentum", centerOfMassFrame, registry);
      YoDouble alpha = new YoDouble("filteredAngularMomentumAlpha", registry);
      alpha.set(0.95); // switch to break frequency and move to walking parameters
      filteredYoAngularMomentum = new AlphaFilteredYoFrameVector("filteredAngularMomentum", "", registry, alpha, yoAngularMomentum);

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
      centerOfMassFrame.update();

      if (referenceFramesVisualizer != null)
         referenceFramesVisualizer.update();

      computeCop();
      computeCapturePoint();
      updateBipedSupportPolygons();

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

   public CenterOfMassJacobian getCenterOfMassJacobian()
   {
      return capturePointCalculator.getCenterOfMassJacobian();
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

   public void getCapturePoint(FixedFramePoint2DBasics capturePointToPack)
   {
      capturePointToPack.set(yoCapturePoint);
   }

   public FramePoint3DReadOnly getCapturePoint()
   {
      return yoCapturePoint;
   }

   public double getStandingHeight()
   {
      return standingHeight.getDoubleValue();
   }

   public double getSquattingHeight()
   {
      return squattingHeight.getDoubleValue();
   }

   public void getCapturePoint(FixedFramePoint3DBasics capturePointToPack)
   {
      capturePointToPack.setMatchingFrame(yoCapturePoint);
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

   public YoRegistry getYoVariableRegistry()
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

   public void resetFootPlaneContactPoint(RobotSide robotSide)
   {
      ContactablePlaneBody foot = feet.get(robotSide);
      YoPlaneContactState footContactState = footContactStates.get(robotSide);
      List<FramePoint2D> defaultContactPoints = foot.getContactPoints2d();
      previousFootContactPoints.get(robotSide).copyFromListAndTrimSize(defaultContactPoints);
      footContactState.setContactFramePoints(defaultContactPoints);
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

   public OneDoFJointBasics[] getUncontrolledOneDoFJoints()
   {
      return uncontrolledOneDoFJoints;
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


   private final FramePoint3D tempPosition = new FramePoint3D();

   public void resetFootSupportPolygon(RobotSide robotSide)
   {
      YoPlaneContactState contactState = footContactStates.get(robotSide);
      List<YoContactPoint> contactPoints = contactState.getContactPoints();
      FrameConvexPolygon2D defaultSupportPolygon = defaultFootPolygons.get(robotSide);

      for (int i = 0; i < defaultSupportPolygon.getNumberOfVertices(); i++)
      {
         tempPosition.setIncludingFrame(defaultSupportPolygon.getVertex(i), 0.0);
         contactPoints.get(i).setMatchingFrame(tempPosition);
      }
      contactState.notifyContactStateHasChanged();
   }

   public double getOmega0()
   {
      return omega0.getDoubleValue();
   }
   
   public YoDouble getOmega0Provider()
   {
      return omega0;
   }

   public void getAngularMomentum(FrameVector3D upperBodyAngularMomentumToPack)
   {
      upperBodyAngularMomentumToPack.setIncludingFrame(angularMomentum);
   }

   public ReferenceFrameHashCodeResolver getReferenceFrameHashCodeResolver()
   {
      return referenceFrameHashCodeResolver;
   }

   public double getFinalTransferTime()
   {
      return finalTransferTime.getDoubleValue();
   }

}
