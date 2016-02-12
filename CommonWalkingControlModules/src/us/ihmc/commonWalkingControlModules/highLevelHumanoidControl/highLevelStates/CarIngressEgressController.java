package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.ChestOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodyPositionControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingManagers;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumControlModuleBridge.MomentumControlModuleType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.commonWalkingControlModules.packetConsumers.ChestOrientationProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredFootStateProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredPelvisLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredThighLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.FootPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.PelvisPoseProvider;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;
import us.ihmc.robotics.controllers.GainCalculator;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.OrientationInterpolationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.providers.YoQuaternionProvider;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;
import us.ihmc.robotics.trajectories.providers.ConstantDoubleProvider;

public class CarIngressEgressController extends AbstractHighLevelHumanoidControlPattern
{
   public final static HighLevelState controllerState = HighLevelState.INGRESS_EGRESS;
   private final static MomentumControlModuleType MOMENTUM_CONTROL_MODULE_TO_USE = MomentumControlModuleType.OPT_NULLSPACE;

   private final FootPoseProvider footPoseProvider;
   private final DesiredFootStateProvider footLoadBearingProvider;
   private DesiredThighLoadBearingProvider thighLoadBearingProvider;
   private DesiredPelvisLoadBearingProvider pelvisLoadBearingProvider;

   private final PelvisPoseProvider pelvisPoseProvider;
   private final RigidBodyPositionControlModule pelvisController;
   private final ReferenceFrame pelvisPositionControlFrame;
   private final int pelvisJacobianId;
   private final TaskspaceConstraintData pelvisTaskspaceConstraintData = new TaskspaceConstraintData();

   private final FramePoint desiredPelvisWaypointPosition = new FramePoint();
   private final FrameVector desiredPelvisWaypointLinearVelocity = new FrameVector();

   private final MultipleWaypointsPositionTrajectoryGenerator pelvisPositionTrajectoryGenerator;
   private DoubleYoVariable pelvisTrajectoryStartTime = new DoubleYoVariable("pelvisTrajectoryStartTime", registry);

   private final ChestOrientationProvider chestOrientationProvider;
   private final ReferenceFrame chestPositionControlFrame;
   private final YoQuaternionProvider finalDesiredChestOrientation, initialDesiredChestOrientation;
   private final OrientationInterpolationTrajectoryGenerator chestOrientationTrajectoryGenerator;
   private double chestTrajectoryStartTime = 0.0;

   private final BooleanYoVariable l_footDoToeOff = new BooleanYoVariable("l_footDoToeOff", registry);
   private final BooleanYoVariable r_footDoToeOff = new BooleanYoVariable("r_footDoToeOff", registry);
   private final SideDependentList<BooleanYoVariable> doToeOff = new SideDependentList<BooleanYoVariable>(l_footDoToeOff, r_footDoToeOff);

   private final ConstantDoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(2.0);

   private final SideDependentList<ContactablePlaneBody> contactableThighs;
   private final ContactablePlaneBody contactablePelvis, contactablePelvisBack;

   private final DoubleYoVariable coefficientOfFrictionForBumAndThighs = new DoubleYoVariable("coefficientOfFrictionForBumAndThighs", registry);
   private final DoubleYoVariable coefficientOfFrictionForFeet = new DoubleYoVariable("coefficientOfFrictionForFeet", registry);

   private final RigidBody elevator;
   private final RigidBody pelvis;
   private final List<ContactablePlaneBody> bodiesInContact = new ArrayList<ContactablePlaneBody>();
   private final Map<ContactablePlaneBody, Integer> contactJacobians = new LinkedHashMap<ContactablePlaneBody, Integer>();

   private BooleanYoVariable requestedPelvisLoadBearing = new BooleanYoVariable("requestedPelvisLoadBearing", registry);
   private BooleanYoVariable requestedPelvisBackLoadBearing = new BooleanYoVariable("requestedPelvisBackLoadBearing", registry);
   private BooleanYoVariable requestedLeftThighLoadBearing = new BooleanYoVariable("requestedLeftThighLoadBearing", registry);
   private BooleanYoVariable requestedRightThighLoadBearing = new BooleanYoVariable("requestedRightThighLoadBearing", registry);
   private SideDependentList<BooleanYoVariable> requestedThighLoadBearing = new SideDependentList<BooleanYoVariable>(requestedLeftThighLoadBearing,
         requestedRightThighLoadBearing);
   private final DoubleYoVariable carIngressPelvisPositionKp = new DoubleYoVariable("carIngressPelvisPositionKp", registry);
   private final DoubleYoVariable carIngressPelvisPositionZeta = new DoubleYoVariable("carIngressPelvisPositionZeta", registry);
   private final DoubleYoVariable carIngressPelvisOrientationKp = new DoubleYoVariable("carIngressPelvisOrientationKp", registry);
   private final DoubleYoVariable carIngressPelvisOrientationZeta = new DoubleYoVariable("carIngressPelvisOrientationZeta", registry);

   private final DoubleYoVariable carIngressChestOrientationKp = new DoubleYoVariable("carIngressChestOrientationKp", registry);
   private final DoubleYoVariable carIngressChestOrientationZeta = new DoubleYoVariable("carIngressChestOrientationZeta", registry);

   private final YoFrameVector yoPelvisLinearAcceleration = new YoFrameVector("pelvisLinearAcceleration", worldFrame, registry);

   public CarIngressEgressController(VariousWalkingProviders variousWalkingProviders, VariousWalkingManagers variousWalkingManagers,
         MomentumBasedController momentumBasedController, WalkingControllerParameters walkingControllerParameters)
   {
      super(variousWalkingProviders, variousWalkingManagers, momentumBasedController, walkingControllerParameters, controllerState);

      setupManagers(variousWalkingManagers);

      coefficientOfFrictionForBumAndThighs.set(0.0);
      coefficientOfFrictionForFeet.set(0.6);

      elevator = fullRobotModel.getElevator();
      pelvis = fullRobotModel.getPelvis();
      contactableThighs = momentumBasedController.getContactablePlaneThighs();
      contactablePelvis = momentumBasedController.getContactablePelvis();
      contactablePelvisBack = momentumBasedController.getContactablePelvisBack();

      this.pelvisPoseProvider = variousWalkingProviders.getDesiredPelvisPoseProvider();
      this.footPoseProvider = variousWalkingProviders.getDesiredFootPoseProvider();
      this.footLoadBearingProvider = variousWalkingProviders.getDesiredFootStateProvider();
      this.thighLoadBearingProvider = variousWalkingProviders.getDesiredThighLoadBearingProvider();
      this.pelvisLoadBearingProvider = variousWalkingProviders.getDesiredPelvisLoadBearingProvider();

      // Setup the pelvis trajectory generator
      pelvisPositionControlFrame = fullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint();
      this.pelvisController = new RigidBodyPositionControlModule("pelvis", twistCalculator, fullRobotModel.getPelvis(), pelvisPositionControlFrame, controlDT,
            registry);

      carIngressPelvisPositionKp.set(100.0);
      carIngressPelvisPositionZeta.set(1.0);
      carIngressPelvisOrientationKp.set(100.0);
      carIngressPelvisOrientationZeta.set(1.0);

      VariableChangedListener pelvisGainsChangedListener = createPelvisGainsChangedListener();

      pelvisGainsChangedListener.variableChanged(null);

      pelvisJacobianId = momentumBasedController.getOrCreateGeometricJacobian(fullRobotModel.getElevator(), fullRobotModel.getPelvis(), fullRobotModel
            .getPelvis().getBodyFixedFrame());

      pelvisPositionTrajectoryGenerator = new MultipleWaypointsPositionTrajectoryGenerator("pelvis", 50, worldFrame, registry);

      // Set up the chest trajectory generator
      this.chestOrientationProvider = variousWalkingProviders.getDesiredChestOrientationProvider();
      chestPositionControlFrame = fullRobotModel.getChest().getParentJoint().getFrameAfterJoint();
      initialDesiredChestOrientation = new YoQuaternionProvider("initialDesiredChest", worldFrame, registry);
      finalDesiredChestOrientation = new YoQuaternionProvider("finalDesiredChest", worldFrame, registry);
      chestOrientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator("chest", worldFrame, trajectoryTimeProvider,
            initialDesiredChestOrientation, finalDesiredChestOrientation, registry);

      VariableChangedListener heelOffVariableChangedListener = new ToeOffYoVariableChangedListener();
      for (RobotSide robotSide : RobotSide.values)
      {
         doToeOff.get(robotSide).addVariableChangedListener(heelOffVariableChangedListener);
      }

      LoadBearingVariableChangedListener loadBearingVariableChangedListener = new LoadBearingVariableChangedListener();

      requestedPelvisLoadBearing.addVariableChangedListener(loadBearingVariableChangedListener);
      requestedPelvisBackLoadBearing.addVariableChangedListener(loadBearingVariableChangedListener);
      for (RobotSide robotSide : RobotSide.values)
      {
         requestedThighLoadBearing.get(robotSide).addVariableChangedListener(loadBearingVariableChangedListener);
      }
   }

   private VariableChangedListener createPelvisGainsChangedListener()
   {
      VariableChangedListener ret = new VariableChangedListener()
      {
         public void variableChanged(YoVariable<?> v)
         {
            double kPelvisPosition = carIngressPelvisPositionKp.getDoubleValue();
            double dPelvisPosition = GainCalculator.computeDerivativeGain(kPelvisPosition, carIngressPelvisPositionZeta.getDoubleValue());
            pelvisController.setProportionalGains(kPelvisPosition, kPelvisPosition, kPelvisPosition);
            pelvisController.setDerivativeGains(dPelvisPosition, dPelvisPosition, dPelvisPosition);
         }
      };

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
         public void variableChanged(YoVariable<?> v)
         {
            double chestKp = carIngressChestOrientationKp.getDoubleValue();
            double chestZeta = carIngressChestOrientationZeta.getDoubleValue();
            double chestKd = GainCalculator.computeDerivativeGain(chestKp, chestZeta);
            chestOrientationManager.setControlGains(chestKp, chestKd);
         }
      };

      carIngressChestOrientationKp.addVariableChangedListener(ret);
      carIngressChestOrientationZeta.addVariableChangedListener(ret);

      return ret;
   }

   private RigidBody baseForChestOrientationControl;
   private int jacobianForChestOrientationControlId;

   private void setupManagers(VariousWalkingManagers variousWalkingManagers)
   {
      baseForChestOrientationControl = fullRobotModel.getPelvis();
      ChestOrientationManager chestOrientationManager = variousWalkingManagers.getChestOrientationManager();
      String[] chestOrientationControlJointNames = walkingControllerParameters.getDefaultChestOrientationControlJointNames();
      jacobianForChestOrientationControlId = chestOrientationManager.createJacobian(fullRobotModel, chestOrientationControlJointNames);
   }

   public void initialize()
   {
      super.initialize();

      momentumBasedController.setMomentumControlModuleToUse(MOMENTUM_CONTROL_MODULE_TO_USE);

      chestOrientationManager.setUp(baseForChestOrientationControl, jacobianForChestOrientationControlId);
      carIngressChestOrientationKp.set(100.0);
      carIngressChestOrientationZeta.set(1.0);
      VariableChangedListener chestGainsChangedListener = createChestGainsChangedListener();
      chestGainsChangedListener.variableChanged(null);

      initializeContacts();

      desiredPelvisWaypointPosition.setToZero(pelvisPositionControlFrame);
      desiredPelvisWaypointPosition.changeFrame(worldFrame);
      desiredPelvisWaypointLinearVelocity.setToZero(worldFrame);

      pelvisTrajectoryStartTime.set(yoTime.getDoubleValue());
      pelvisPositionTrajectoryGenerator.clear();
      //  pelvisPositionTrajectoryGenerator.appendWayPoint(0.0, desiredPelvisWaypointPosition, desiredPelvisWaypointLinearVelocity);
      pelvisPositionTrajectoryGenerator.initialize();

      FrameOrientation initialDesiredChestOrientation = new FrameOrientation(chestPositionControlFrame);
      initialDesiredChestOrientation.changeFrame(this.initialDesiredChestOrientation.getReferenceFrame());
      this.initialDesiredChestOrientation.setOrientation(initialDesiredChestOrientation);
      finalDesiredChestOrientation.setOrientation(initialDesiredChestOrientation);

      chestTrajectoryStartTime = yoTime.getDoubleValue();
      chestOrientationTrajectoryGenerator.initialize();

      pelvisOrientationManager.setToHoldCurrentDesiredInWorldFrame();
   }

   private void initializeContacts()
   {
      requestedPelvisLoadBearing.set(isContactablePlaneBodyInContact(contactablePelvis));
      requestedPelvisLoadBearing.notifyVariableChangedListeners();
      requestedPelvisBackLoadBearing.set(false); // Set to false there is no button in the GUI to change it anymore
      requestedPelvisBackLoadBearing.notifyVariableChangedListeners();

      for (RobotSide robotSide : RobotSide.values)
      {
         if (isContactablePlaneBodyInContact(feet.get(robotSide)))
            feetManager.setFlatFootContactState(robotSide);
         else
         {
            double defaultSwingTime = this.walkingControllerParameters.getDefaultSwingTime();
            feetManager.requestMoveStraight(robotSide, new FramePose(feet.get(robotSide).getFrameAfterParentJoint()), defaultSwingTime);
         }
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
      doChestControl();
      doCoMControl();
      doPelvisControl();
      //doJointPositionControl();

      setTorqueControlJointsToZeroDersiredAcceleration();

      momentumBasedController.doSecondaryControl();
   }

   protected void doPelvisControl()
   {
      super.doPelvisControl();

      if (pelvisPoseProvider != null)
      {
         if (pelvisPoseProvider.checkForNewPosition())
         {
            double time = yoTime.getDoubleValue() - pelvisTrajectoryStartTime.getDoubleValue();
            pelvisPositionTrajectoryGenerator.compute(time);

            pelvisPositionTrajectoryGenerator.get(desiredPelvisWaypointPosition);
            desiredPelvisWaypointLinearVelocity.setToZero(worldFrame);
            pelvisPositionTrajectoryGenerator.clear();
            pelvisPositionTrajectoryGenerator.appendWaypoint(0.0, desiredPelvisWaypointPosition, desiredPelvisWaypointLinearVelocity);
            desiredPelvisWaypointPosition.setIncludingFrame(pelvisPoseProvider.getDesiredPelvisPosition(worldFrame));
            desiredPelvisWaypointPosition.changeFrame(worldFrame);
            double dt = pelvisPoseProvider.getTrajectoryTime();
            pelvisPositionTrajectoryGenerator.appendWaypoint(dt, desiredPelvisWaypointPosition, desiredPelvisWaypointLinearVelocity);
            pelvisTrajectoryStartTime.set(yoTime.getDoubleValue());

            pelvisPositionTrajectoryGenerator.initialize();
         }
      }

      double time = yoTime.getDoubleValue() - pelvisTrajectoryStartTime.getDoubleValue();
      pelvisPositionTrajectoryGenerator.compute(time);

      FramePoint desiredPosition = new FramePoint(pelvisPositionControlFrame);
      FrameVector desiredVelocity = new FrameVector(pelvisPositionControlFrame);
      FrameVector desiredPelvisAcceleration = new FrameVector(pelvisPositionControlFrame);

      pelvisPositionTrajectoryGenerator.packLinearData(desiredPosition, desiredVelocity, desiredPelvisAcceleration);

      FrameVector pelvisLinearAcceleration = new FrameVector();
      pelvisController.doPositionControl(pelvisLinearAcceleration, desiredPosition, desiredVelocity, desiredPelvisAcceleration, fullRobotModel.getElevator());

      yoPelvisLinearAcceleration.setAndMatchFrame(pelvisLinearAcceleration);

      pelvisLinearAcceleration.changeFrame(pelvis.getBodyFixedFrame());

      if (!requestedPelvisLoadBearing.getBooleanValue())
      {
         pelvisTaskspaceConstraintData.set(elevator, pelvis);
         pelvisTaskspaceConstraintData.setLinearAcceleration(pelvis.getBodyFixedFrame(), elevator.getBodyFixedFrame(), pelvisLinearAcceleration);
      }
      else
      {
         DenseMatrix64F selectionMatrix = new DenseMatrix64F(2, SpatialMotionVector.SIZE);
         selectionMatrix.set(0, 3, 1.0);
         selectionMatrix.set(0, 4, 1.0);
         pelvisTaskspaceConstraintData.setLinearAcceleration(pelvis.getBodyFixedFrame(), elevator.getBodyFixedFrame(), pelvisLinearAcceleration,
               selectionMatrix);
      }

      momentumBasedController.setDesiredSpatialAcceleration(pelvisJacobianId, pelvisTaskspaceConstraintData);
   }

   protected void doChestControl()
   {
      if (chestOrientationProvider != null)
      {
         if (chestOrientationProvider.checkForNewChestOrientation())
         {
            chestOrientationTrajectoryGenerator.compute(yoTime.getDoubleValue() - chestTrajectoryStartTime);
            FrameOrientation previousDesiredChestOrientation = new FrameOrientation(chestPositionControlFrame);
            chestOrientationTrajectoryGenerator.get(previousDesiredChestOrientation);
            initialDesiredChestOrientation.setOrientation(previousDesiredChestOrientation);

            FrameOrientation desiredChestOrientation = chestOrientationProvider.getDesiredChestOrientation();
            desiredChestOrientation.changeFrame(finalDesiredChestOrientation.getReferenceFrame());
            finalDesiredChestOrientation.setOrientation(desiredChestOrientation);
            chestTrajectoryStartTime = yoTime.getDoubleValue();

            chestOrientationTrajectoryGenerator.initialize();
         }
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
         handleFootPose(robotSide);

      super.doFootControl();
   }

   private void doContactPointControl()
   {
      for (ContactablePlaneBody body : bodiesInContact)
      {
         int jacobianId = contactJacobians.get(body);
         ReferenceFrame baseFrame = momentumBasedController.getJacobian(jacobianId).getBaseFrame();
         FrameVector desiredAcceleration = new FrameVector(baseFrame, 0.0, 0.0, 0.0);
         DenseMatrix64F selectionMatrix = new DenseMatrix64F(1, 3);
         selectionMatrix.set(0, 2, 1.0);
         for (FramePoint contactPoint : body.getContactPointsCopy())
         {
            momentumBasedController.setDesiredPointAcceleration(jacobianId, contactPoint, desiredAcceleration, selectionMatrix);
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
      contactJacobians.put(contactablePlaneBody, momentumBasedController.getOrCreateGeometricJacobian(elevator, rigidBody, elevator.getBodyFixedFrame()));
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
         // If the foot is already in load bearing state, do nothing:
         if (footLoadBearingProvider != null && footLoadBearingProvider.checkForNewLoadBearingRequest(robotSide))
            feetManager.setFlatFootContactState(robotSide);

         if (thighLoadBearingProvider != null && thighLoadBearingProvider.checkForNewLoadBearingState(robotSide))
            requestedThighLoadBearing.get(robotSide).set(thighLoadBearingProvider.getDesiredThighLoadBearingState(robotSide));
      }

      if (pelvisLoadBearingProvider != null && pelvisLoadBearingProvider.checkForNewLoadBearingState())
         requestedPelvisLoadBearing.set(desiredPelvisLoadBearingState);
   }

   public boolean isContactablePlaneBodyInContact(ContactablePlaneBody contactablePlaneBody)
   {
      if (momentumBasedController.getContactState(contactablePlaneBody) == null)
         return false;
      else
         return momentumBasedController.getContactState(contactablePlaneBody).inContact();
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
      if (contactableThighs == null)
         return;

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

   private class LoadBearingVariableChangedListener implements VariableChangedListener
   {
      public void variableChanged(YoVariable<?> v)
      {
         if (!(v instanceof BooleanYoVariable))
            return;

         if (v.equals(requestedPelvisLoadBearing))
            setPelvisInContact(requestedPelvisLoadBearing.getBooleanValue());

         if (v.equals(requestedPelvisBackLoadBearing))
            setPelvisBackInContact(requestedPelvisBackLoadBearing.getBooleanValue());

         for (RobotSide robotSide : RobotSide.values)
         {
            if (v.equals(requestedThighLoadBearing.get(robotSide)))
               setThighInContact(robotSide, requestedThighLoadBearing.get(robotSide).getBooleanValue());
         }
      }
   }

   private class ToeOffYoVariableChangedListener implements VariableChangedListener
   {
      public void variableChanged(YoVariable<?> v)
      {
         if (!(v instanceof BooleanYoVariable))
            return;

         for (RobotSide robotSide : RobotSide.values)
         {
            if (v.equals(doToeOff.get(robotSide)))
            {
               if (doToeOff.get(robotSide).getBooleanValue())
               {
                  feetManager.setOnToesContactState(robotSide);
               }
               else
               {
                  feetManager.setFlatFootContactState(robotSide);
               }
            }
         }
      }
   }
}
