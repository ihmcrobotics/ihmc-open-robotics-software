package us.ihmc.commonWalkingControlModules.controlModules.upperBody;

import java.util.ArrayList;
import java.util.EnumMap;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import quickhull3d.Point3d;

import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.ArmControlModule;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.SpineLungingControlModule;
import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.UpperBodySubController;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.NeckJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.NeckTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.SpineJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.UpperBodyTorques;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.utilities.containers.ContainerTools;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoAppearance;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.PIDController;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicVector;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector2d;
import com.yobotics.simulationconstructionset.util.statemachines.State;
import com.yobotics.simulationconstructionset.util.statemachines.StateMachine;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransition;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransitionCondition;

public class BalancingUpperBodySubController implements UpperBodySubController
{
   private final CouplingRegistry couplingRegistry;
   private final ProcessedSensorsInterface processedSensors;
   private final CommonWalkingReferenceFrames referenceFrames;
   private final YoVariableRegistry registry = new YoVariableRegistry("BalancingUpperBodySubController");

   private final ArmControlModule armControlModule;
   private final SpineLungingControlModule spineControlModule;

   private final EnumMap<NeckJointName, PIDController> neckControllers = ContainerTools.createEnumMap(NeckJointName.class);
   private final EnumMap<NeckJointName, DoubleYoVariable> desiredNeckPositions = ContainerTools.createEnumMap(NeckJointName.class);

   private final double controlDT;
   private NeckTorques neckTorques = new NeckTorques();

   private final StateMachine stateMachine;
   private final String name = "BalancingUpperBodySubController";

   private final YoFrameVector2d lungeAxis = new YoFrameVector2d("lungeAxis", "", ReferenceFrame.getWorldFrame(), registry);
   //   public Wrench wrenchOnChest;
   private final RigidBody chest;
   private final RigidBody pelvis;
   private final double robotMass;
   private final double gravity;

   private final DoubleYoVariable stopLungingICPPosition = new DoubleYoVariable("stopLungingRadius", registry);
   private final DoubleYoVariable maxAngle = new DoubleYoVariable("maxAngle", registry);
   private final DoubleYoVariable maxHipTorque = new DoubleYoVariable("maxHipTorque", registry);

   private final YoFrameVector wrenchOnPelvisLinear = new YoFrameVector("wrenchOnPelvisLinear", "", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector wrenchOnPelvisAngular = new YoFrameVector("wrenchOnPelvisAngular", "", ReferenceFrame.getWorldFrame(), registry);

   private final YoFramePoint centerOfMassPosition = new YoFramePoint("comGraphic", "", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector lungeAxisGraphic = new YoFrameVector("lungeAxisGraphic", "", ReferenceFrame.getWorldFrame(), registry);

   private final DoubleYoVariable chestAngularVelocity = new DoubleYoVariable("chestAngularVelocity", registry);
   private final DoubleYoVariable chestAngularAcceleration = new DoubleYoVariable("chestAngularAcceleration", registry);
   
   private final BooleanYoVariable forceControllerIntoState = new BooleanYoVariable("force" + name + "IntoState", registry);
   private final EnumYoVariable<BalancingUpperBodySubControllerState> forcedControllerState = new EnumYoVariable<BalancingUpperBodySubControllerState>("forced"
         + name + "State", registry, BalancingUpperBodySubControllerState.class);

   public BalancingUpperBodySubController(CouplingRegistry couplingRegistry, ProcessedSensorsInterface processedSensors, CommonWalkingReferenceFrames referenceFrames, 
         double controlDT, RigidBody chest, double maxHipTorque, ArmControlModule armControlModule, SpineLungingControlModule spineControlModule, YoVariableRegistry parentRegistry,
         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      this.couplingRegistry = couplingRegistry;
      this.processedSensors = processedSensors;
      this.referenceFrames = referenceFrames;
      this.controlDT = controlDT;
      this.armControlModule = armControlModule;
      this.spineControlModule = spineControlModule;
      this.chest = chest;
      this.maxHipTorque.set(maxHipTorque);
      this.robotMass = processedSensors.getTotalMass();
      this.gravity = processedSensors.getGravityInWorldFrame().getZ();
      this.pelvis = processedSensors.getFullRobotModel().getPelvis();
      this.stateMachine = new StateMachine(name + "State", name + "SwitchTime", BalancingUpperBodySubControllerState.class, processedSensors.getYoTime(),
            registry);

      populateDynamicsGraphicObjects(dynamicGraphicObjectsListRegistry);
      populateYoVariables();

      populateControllers();
      parentRegistry.addChild(registry);

      setGains();
      setParameters();
      displayWarningsForBooleans();

      setUpStateMachine();
   }

   private void populateDynamicsGraphicObjects(DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      DynamicGraphicVector lungeAxisVisual = new DynamicGraphicVector("lungeAxisVisual", centerOfMassPosition, lungeAxisGraphic, 1.0, YoAppearance.DarkRed());
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject(name, lungeAxisVisual);
   }

   public void doUpperBodyControl(UpperBodyTorques upperBodyTorquesToPack)
   {
      stateMachine.doAction();
      if (!forceControllerIntoState.getBooleanValue())
      {
         stateMachine.checkTransitionConditions();
      } else
      {
         stateMachine.setCurrentState(forcedControllerState.getEnumValue());
      }

      armControlModule.doArmControl(upperBodyTorquesToPack.getArmTorques());
      this.doNeckControl();

      // set torques
      upperBodyTorquesToPack.setNeckTorques(neckTorques);
      spineControlModule.getSpineTorques(upperBodyTorquesToPack.getSpineTorques());

      updateDynamicsGraphicObjects();
   }

   private void updateDynamicsGraphicObjects()
   {
      centerOfMassPosition.set(processedSensors.getCenterOfMassPositionInFrame(ReferenceFrame.getWorldFrame()));
      lungeAxisGraphic.setXY(lungeAxis.getFrameVector2dCopy());
   }

   private void setUpStateMachine()
   {
      State base = new BaseState();
      State icpRecoverAccelerateState = new ICPRecoverAccelerateState();
      State icpRecoverDecelerateState = new ICPRecoverDecelerateState();

      StateTransitionCondition isICPOutsideLungeRadius = new IsICPOutsideLungeRadiusCondition();
      StateTransitionCondition doWeNeedToDecelerate = new DoWeNeedToDecelerate();
      StateTransitionCondition isBodyAngularVelocityZero = new IsBodyAngularVelocityZeroCondition(1e-2);

      StateTransition toICPRecoverAccelerate = new StateTransition(icpRecoverAccelerateState.getStateEnum(), isICPOutsideLungeRadius);
      StateTransition toICPRecoverDecelerate = new StateTransition(icpRecoverDecelerateState.getStateEnum(), doWeNeedToDecelerate);
      StateTransition toBase = new StateTransition(base.getStateEnum(), isBodyAngularVelocityZero); //TODO now going back to base

      base.addStateTransition(toICPRecoverAccelerate);
      icpRecoverAccelerateState.addStateTransition(toICPRecoverDecelerate);
      icpRecoverDecelerateState.addStateTransition(toBase); //TODO now going back to base

      stateMachine.addState(base);
      stateMachine.addState(icpRecoverAccelerateState);
      stateMachine.addState(icpRecoverDecelerateState);

      if (forceControllerIntoState.getBooleanValue())
      {
         stateMachine.setCurrentState(forcedControllerState.getEnumValue());
      } else
      {
         stateMachine.setCurrentState(base.getStateEnum());
      }

   }

   private enum BalancingUpperBodySubControllerState
   {
      BASE, ICP_REC_ACC, ICP_REC_DEC;
   }

   private class BaseState extends State
   {
      public BaseState()
      {
         super(BalancingUpperBodySubControllerState.BASE);
      }

      public void doAction()
      {
         spineControlModule.doMaintainDesiredChestOrientation();
      }
      
      public void doTransitionIntoAction()
      {
         couplingRegistry.setIsLunging(false);
      }
      
      public void doTransitionOutOfAction()
      {
         
      }
   }

   private class ICPRecoverAccelerateState extends State
   {
      Wrench wrenchOnPelvis;
      ReferenceFrame pelvisFrame;
      Vector3d wrenchOnPelvisLinearPart;
      Vector3d wrenchOnPelvisAngularPart;

      public ICPRecoverAccelerateState()
      {
         super(BalancingUpperBodySubControllerState.ICP_REC_ACC);
      }

      public void doAction()
      {
         // Wrench works best since this minimizes the unwanted rotations (yaw) of the upper body

//         wrenchOnPelvis = new Wrench(pelvisFrame, ReferenceFrame.getWorldFrame(), wrenchOnPelvisLinearPart, wrenchOnPelvisAngularPart);
//         wrenchOnPelvis.changeFrame(pelvisFrame);
         
         
         wrenchOnPelvis = new Wrench(pelvisFrame, pelvisFrame, wrenchOnPelvisLinearPart, wrenchOnPelvisAngularPart);
         
         storeWrenchCopyAsYoVariable(wrenchOnPelvis);

         spineControlModule.setWrench(wrenchOnPelvis);
         spineControlModule.doMaintainDesiredChestOrientation();

      }

      private void storeWrenchCopyAsYoVariable(Wrench wrench)
      {
         wrenchOnPelvisAngular.set(wrench.getAngularPartCopy());
         wrenchOnPelvisLinear.set(wrench.getLinearPartCopy());
      }

      public void doTransitionIntoAction()
      {
         couplingRegistry.setIsLunging(true);
         
         setLungeDirectionBasedOnIcp();
         // scaling
         spineControlModule.scaleGainsBasedOnLungeAxis(lungeAxis.getFrameVector2dCopy().getVector());

         // setting to zero
//         ArrayList<SpineJointName> spineJointsWithZeroGain = new ArrayList<SpineJointName>();
//         spineJointsWithZeroGain.add(SpineJointName.SPINE_ROLL);
//         spineJointsWithZeroGain.add(SpineJointName.SPINE_PITCH);
//         spineControlModule.setGainsToZero(spineJointsWithZeroGain);
         
         pelvisFrame = pelvis.getBodyFixedFrame();

         wrenchOnPelvisAngularPart = new Vector3d(lungeAxis.getX(), lungeAxis.getY(), 0.0);
         wrenchOnPelvisAngularPart.scale(maxHipTorque.getDoubleValue());
         wrenchOnPelvisLinearPart = new Vector3d();

      }

      public void doTransitionOutOfAction()
      {
         setWrenchOnChestToZero(wrenchOnPelvis);
         storeWrenchCopyAsYoVariable(wrenchOnPelvis);
         spineControlModule.setWrench(wrenchOnPelvis);
         spineControlModule.setGains(); // may not be needed
      }

      private void setLungeDirectionBasedOnIcp()
      {
         // TODO taking the capturePoint in the world frame will not work if the robot walks
         FramePoint2d capturePointInBodyAttachedZUP = couplingRegistry.getCapturePointInFrame(ReferenceFrame.getWorldFrame()).toFramePoint2d();

         lungeAxis.set(-capturePointInBodyAttachedZUP.getY(), capturePointInBodyAttachedZUP.getX());
         lungeAxis.normalize();
      }
   }

   private class ICPRecoverDecelerateState extends NoTransitionActionsState
   {
      public ICPRecoverDecelerateState()
      {
         super(BalancingUpperBodySubControllerState.ICP_REC_DEC);
      }

      public void doAction()
      {
         // TODO Auto-generated method stub
      }
   }

   private class IsICPOutsideLungeRadiusCondition implements StateTransitionCondition
   {
      public boolean checkCondition()
      {
         return !isCapturePointInsideSupportPolygon();
         // can be altered to lunge radius if needed.
      }
   }

   private class DoWeNeedToDecelerate implements StateTransitionCondition
   {
   
      public boolean checkCondition()
      {
         updateAngularVelocityAndAcceleration();
         boolean doWeNeedToSlowDownBecauseOfAngleLimit = doWeNeedToSlowDownBecauseOfAngleLimit();
         boolean willICPEndUpInsideStopLungingRadius = willICPEndUpFarEnoughBack();

         return doWeNeedToSlowDownBecauseOfAngleLimit || willICPEndUpInsideStopLungingRadius;
      }

      private void updateAngularVelocityAndAcceleration()
      {
         // TODO check if this is okay.
         FrameVector chestAngularVelocityVector = processedSensors.getChestAngularVelocityInChestFrame();
         chestAngularVelocityVector.changeFrame(ReferenceFrame.getWorldFrame());
         chestAngularVelocity.set(chestAngularVelocityVector.length());
         
         // TODO fix this bullshit.
         SpatialAccelerationVector chestAccelerationVector = processedSensors.getAccelerationOfPelvisWithRespectToWorld();
         chestAngularAcceleration.set(chestAccelerationVector.getAngularPartCopy().length());
      }

      private boolean doWeNeedToSlowDownBecauseOfAngleLimit()
      {
         //TODO implement
         return false;
      }

      private boolean willICPEndUpFarEnoughBack()
      {
         ReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
         
         //Compute Current Location of CMP based on the applied spine and lower-body torques
         FramePoint2d cmpCurrent = processedSensors.getCentroidalMomentPivotInFrame(midFeetZUpFrame).toFramePoint2d();
//         FramePoint2d cmpCurrent = new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.0, 0.0);  //TODO: Need to compute/get actual CMP
         
         //Compute the time required to bring the upper-body rotation to zero, in response to a constant torque (MaxHipTorque) for an assumed Moment of Inertia (Projected about lunge axis)
         double elapsedTimeToStop = chestAngularVelocity.getDoubleValue() / chestAngularAcceleration.getDoubleValue();
         
         //Predict where the ICP will wind be at t = timeCurrent + elapsedTimeToStop, in response to a constant CMP location.
         //(i.e., where will the ICP be located after the time for which it takes to slow down the upper-body?)
         double gravity = processedSensors.getGravityInWorldFrame().length();
         double CoMHeight = processedSensors.getCenterOfMassPositionInFrame(ReferenceFrame.getWorldFrame()).getZ();
         double omega0 = Math.sqrt( gravity / CoMHeight );  // Natural Frequency of LIPM
         
         //Predict ICP location: icp(timeCurrent + elapsedTimeToStop) = cmp(timeCurrent) + [ icp(timeCurrent) - cmp(timeCurrent) ] * exp(omega0 * elapsedTimeToStop)
         double deltaTPrime = omega0 * elapsedTimeToStop;
         
         //Get Current ICP Location
         FramePoint2d icpCurrent = couplingRegistry.getCapturePointInFrame(midFeetZUpFrame).toFramePoint2d();
         FramePoint2d icpAfterElapsedTimeToStop = icpCurrent;
         icpAfterElapsedTimeToStop.sub(cmpCurrent);
         icpAfterElapsedTimeToStop.scale(Math.exp(deltaTPrime));
         icpAfterElapsedTimeToStop.add(cmpCurrent);
         
         Vector2d vectorFromDesiredToPredictedICPAfterElapsedTimeToStop = new Vector2d(icpAfterElapsedTimeToStop.getX(), icpAfterElapsedTimeToStop.getY()); //TODO: The desired ICP is not necessarily at (0,0)!
         
         //Get unit vector along initial lunge direction (LungeAxis is already normalized)
         FrameVector2d initialICPDirection = new FrameVector2d(lungeAxis.getFrameVector2dCopy());
         
         // Transition to IcpRecoverDecelerate when the component of predicted ICP along the initial lunge direction crosses zero
         // Use an inequality check to eliminate the need for an epsilon
         boolean ret = vectorFromDesiredToPredictedICPAfterElapsedTimeToStop.dot(initialICPDirection.getVectorCopy()) < stopLungingICPPosition.getDoubleValue();
         
//         return ret;
           return false;
      }
   }

   private class IsBodyAngularVelocityZeroCondition implements StateTransitionCondition
   {
      private final double epsilon;

      public IsBodyAngularVelocityZeroCondition(double epsilon)
      {
         this.epsilon = epsilon;
      }

      public boolean checkCondition()
      {
         // TODO implement
         return false;
      }
   }

   public boolean isCapturePointInsideSupportPolygon()
   {
      FrameConvexPolygon2d supportPolygon = couplingRegistry.getBipedSupportPolygons().getSupportPolygonInMidFeetZUp();
      FramePoint2d capturePoint = couplingRegistry.getCapturePointInFrame(supportPolygon.getReferenceFrame()).toFramePoint2d();

      return supportPolygon.isPointInside(capturePoint);
   }

   private void doNeckControl()
   {
      for (NeckJointName neckJointName : NeckJointName.values())
      {
         PIDController pidController = neckControllers.get(neckJointName);
         double desiredPosition = desiredNeckPositions.get(neckJointName).getDoubleValue();
         double desiredVelocity = 0.0;

         double actualPosition = processedSensors.getNeckJointPosition(neckJointName);
         double actualVelcoity = processedSensors.getNeckJointVelocity(neckJointName);

         double torque = pidController.compute(actualPosition, desiredPosition, actualVelcoity, desiredVelocity, controlDT);
         neckTorques.setTorque(neckJointName, torque);
      }
   }

   private void populateYoVariables()
   {
      for (NeckJointName neckJointName : NeckJointName.values())
      {
         String varName = "desired" + neckJointName.getCamelCaseNameForMiddleOfExpression();
         DoubleYoVariable variable = new DoubleYoVariable(varName, registry);

         desiredNeckPositions.put(neckJointName, variable);
      }
   }

   private void populateControllers()
   {
      for (NeckJointName neckJointName : NeckJointName.values())
      {
         neckControllers.put(neckJointName, new PIDController(neckJointName.getCamelCaseNameForStartOfExpression(), registry));
      }
   }

   private void setGains()
   {
      neckControllers.get(NeckJointName.LOWER_NECK_PITCH).setProportionalGain(100.0);
      neckControllers.get(NeckJointName.NECK_YAW).setProportionalGain(100.0);
      neckControllers.get(NeckJointName.UPPER_NECK_PITCH).setProportionalGain(100.0);

      neckControllers.get(NeckJointName.LOWER_NECK_PITCH).setDerivativeGain(5.0);
      neckControllers.get(NeckJointName.NECK_YAW).setDerivativeGain(5.0);
      neckControllers.get(NeckJointName.UPPER_NECK_PITCH).setDerivativeGain(5.0);
   }

   private void setWrenchOnChestToZero(Wrench wrench)
   {
      Vector3d zeroVector = new Vector3d();
      wrench.setAngularPart(zeroVector);
      wrench.setLinearPart(zeroVector);
   }

   public abstract class NoTransitionActionsState extends State
   {
      public NoTransitionActionsState(Enum<?> stateEnum)
      {
         super(stateEnum);
      }

      public void doTransitionIntoAction()
      {
      }

      public void doTransitionOutOfAction()
      {
      }
   }

   private void setParameters()
   {
      stopLungingICPPosition.set(0.0);
      maxAngle.set(Math.PI / 2.0);
      forcedControllerState.set(BalancingUpperBodySubControllerState.ICP_REC_ACC);
   }

   private void displayWarningsForBooleans()
   {
      if (forceControllerIntoState.getBooleanValue())
      {
         System.out.println("Warning! Controller " + this.name + " is forced to remain in the " + forcedControllerState.toString() + " state!");
      }
   }

}
