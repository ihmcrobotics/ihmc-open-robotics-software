package us.ihmc.commonWalkingControlModules.controlModules.upperBody;

import java.util.ArrayList;
import java.util.EnumMap;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

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
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicVector;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint2d;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector2d;
import com.yobotics.simulationconstructionset.util.statemachines.State;
import com.yobotics.simulationconstructionset.util.statemachines.StateMachine;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransition;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransitionCondition;

public class BalancingUpperBodySubController implements UpperBodySubController
{
   public static final boolean USE_SCALING_INSTEAD_OF_SETTING_TO_ZERO = true;
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
   
   private final YoFramePoint2d icpDesiredLunging = new YoFramePoint2d("icpDesiredLunging", "", ReferenceFrame.getWorldFrame(), registry);

   private final DoubleYoVariable maxChestAngle = new DoubleYoVariable("maxChestAngle", registry);
   private final DoubleYoVariable stopLungingICPRadius = new DoubleYoVariable("stopLungingRadius", registry);
   private final DoubleYoVariable maxHipTorque = new DoubleYoVariable("maxHipTorque", registry);
   private DoubleYoVariable bosRadiusAlongLungeAxis = new DoubleYoVariable("bosRadius", registry);

   private final YoFrameVector wrenchOnPelvisLinear = new YoFrameVector("wrenchOnPelvisByUpperBodyLinear", "", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector wrenchOnPelvisAngular = new YoFrameVector("wrenchOnPelvisByUpperBodyAngular", "", ReferenceFrame.getWorldFrame(), registry);

   private final YoFramePoint centerOfMassPosition = new YoFramePoint("comGraphic", "", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector lungeAxisGraphic = new YoFrameVector("lungeAxisGraphic", "", ReferenceFrame.getWorldFrame(), registry);
   
   private final YoFramePoint2d desiredICPVisualizer = new YoFramePoint2d("midFeetPoint", "", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint2d predictedICPAtTimeToStopVisualizer = new YoFramePoint2d("icpPredicted", "", ReferenceFrame.getWorldFrame(), registry);

   private final DoubleYoVariable chestAngle = new DoubleYoVariable("chestAngle", registry);
   private final DoubleYoVariable chestAngularVelocity = new DoubleYoVariable("chestAngularVelocity", registry);
   private final DoubleYoVariable chestAngularAcceleration = new DoubleYoVariable("chestAngularAcceleration", registry);
   
   private final DoubleYoVariable kTimeToSlowSlowDown = new DoubleYoVariable("kTimeToSlowDown", registry);
   private YoFramePoint2d lungeDesiredCoP;
   
   private EnumMap<BalancingUpperBodySubControllerState, Double> timeOfStateStart = new EnumMap<BalancingUpperBodySubController.BalancingUpperBodySubControllerState, Double>(BalancingUpperBodySubControllerState.class);
   private EnumMap<BalancingUpperBodySubControllerState, Double> chestAngleOnStateStart = new EnumMap<BalancingUpperBodySubController.BalancingUpperBodySubControllerState, Double>(BalancingUpperBodySubControllerState.class);
   
   private final BooleanYoVariable forceControllerIntoState = new BooleanYoVariable("force" + name + "IntoState", registry);
   private final EnumYoVariable<BalancingUpperBodySubControllerState> forcedControllerState = new EnumYoVariable<BalancingUpperBodySubControllerState>("forced"
         + name + "State", registry, BalancingUpperBodySubControllerState.class);

   public BalancingUpperBodySubController(CouplingRegistry couplingRegistry, ProcessedSensorsInterface processedSensors,
         CommonWalkingReferenceFrames referenceFrames, double controlDT, RigidBody chest, double maxHipTorque, ArmControlModule armControlModule,
         SpineLungingControlModule spineControlModule, YoVariableRegistry parentRegistry,
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
      
      DynamicGraphicPosition icpPredictedVisual = new DynamicGraphicPosition("icpPredictedVisual", predictedICPAtTimeToStopVisualizer, 0.05, YoAppearance.Purple());
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject(name, icpPredictedVisual);

      DynamicGraphicPosition midFeetPoint = new DynamicGraphicPosition("midFeetPoint", this.desiredICPVisualizer, 0.05, YoAppearance.Blue());
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject(name, midFeetPoint);

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

      updateDynamicsGraphicObjects();
      updateVariables();
      
      wrenchTest();
      
      // set torques
      upperBodyTorquesToPack.setNeckTorques(neckTorques);
      spineControlModule.getSpineTorques(upperBodyTorquesToPack.getSpineTorques());
      
   }

   private void wrenchTest()
   {
//      Wrench upperBodyWrench = new Wrench();
//      spineControlModule.getWrenchByUpperBody(upperBodyWrench);
//      upperBodyWrench.scale(-1.0);
//      couplingRegistry.setUpperBodyWrench(upperBodyWrench);
//      
//      storeWrenchCopyAsYoVariable(upperBodyWrench);
//      
////      Wrench upperBodyWrench = couplingRegistry.getUpperBodyWrench();
//      if (upperBodyWrench != null)
//      {
//         System.out.println(upperBodyWrench.toString());   
//      }
   }

   private void updateVariables()
   {
      chestAngle.set(getChestAngleToWorld());
      
      // TODO check if this is okay.
      FrameVector chestAngularVelocityVector = processedSensors.getChestAngularVelocityInChestFrame();
      chestAngularVelocityVector.changeFrame(ReferenceFrame.getWorldFrame());
      chestAngularVelocity.set(chestAngularVelocityVector.length());
   }

   private void updateDynamicsGraphicObjects()
   {
      centerOfMassPosition.set(processedSensors.getCenterOfMassPositionInFrame(ReferenceFrame.getWorldFrame()));
      lungeAxisGraphic.setXY(lungeAxis.getFrameVector2dCopy());
   }
   
   /**
    * The initial ICP direction is set in the DoTransitionIntoAction() method of the IcpRecoverAccelerateState
    * @param IcpDirectionToPack
    */
   private FramePoint2d getInitialIcpDirection(ReferenceFrame expressedInFrame)
   {
      FramePoint2d icpDirection = new FramePoint2d(lungeAxis.getReferenceFrame(), lungeAxis.getY(), -lungeAxis.getX());  // Lunge axis and ICP direction are just orthogonal in the x-y plane
      icpDirection.changeFrame(expressedInFrame);
      return icpDirection;
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
         timeOfStateStart.put((BalancingUpperBodySubControllerState)this.getStateEnum(), processedSensors.getTime());
      }

      public void doTransitionOutOfAction()
      {
         // TODO Auto-generated method stub
         
      }
   }
   
   private void storeWrenchCopyAsYoVariable(Wrench wrench)
   {
      wrenchOnPelvisAngular.set(wrench.getAngularPartCopy());
      wrenchOnPelvisLinear.set(wrench.getLinearPartCopy());
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

      public void doTransitionIntoAction()
      {
         setLungeDirectionBasedOnIcp();
         couplingRegistry.setLungeAxis(lungeAxis.getFrameVector2dCopy());

         if (USE_SCALING_INSTEAD_OF_SETTING_TO_ZERO)
         {
            // scaling
            spineControlModule.scaleGainsBasedOnLungeAxis(lungeAxis.getFrameVector2dCopy().getVector());            
         }
         else
         {
            // setting to zero
          ArrayList<SpineJointName> spineJointsWithZeroGain = new ArrayList<SpineJointName>();
          spineJointsWithZeroGain.add(SpineJointName.SPINE_ROLL);
          spineJointsWithZeroGain.add(SpineJointName.SPINE_PITCH);
          spineControlModule.setGainsToZero(spineJointsWithZeroGain);            
         }
         pelvisFrame = referenceFrames.getPelvisFrame();//pelvis.getBodyFixedFrame();
         this.wrenchOnPelvisAngularPart = getDesiredLungingTorqueicpRecoverAccelerateState();
         this.wrenchOnPelvisLinearPart = new Vector3d();

         timeOfStateStart.put((BalancingUpperBodySubControllerState)this.getStateEnum(), processedSensors.getTime());
         chestAngleOnStateStart.put((BalancingUpperBodySubControllerState)this.getStateEnum(), getChestAngleToWorld());
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

   private class ICPRecoverDecelerateState extends State
   {
      Wrench wrenchOnPelvis;
      ReferenceFrame pelvisFrame;
      Vector3d wrenchOnPelvisLinearPart;
      Vector3d wrenchOnPelvisAngularPart;
      
      public ICPRecoverDecelerateState()
      {
         super(BalancingUpperBodySubControllerState.ICP_REC_DEC);
      }

      public void doAction()
      {
         wrenchOnPelvis = new Wrench(pelvisFrame, pelvisFrame, wrenchOnPelvisLinearPart, wrenchOnPelvisAngularPart);
         
         storeWrenchCopyAsYoVariable(wrenchOnPelvis);

         spineControlModule.setWrench(wrenchOnPelvis);
         spineControlModule.doMaintainDesiredChestOrientation();
      }
      
      public void doTransitionIntoAction()
      {
         FrameVector2d tempLungeAxis = lungeAxis.getFrameVector2dCopy();
         tempLungeAxis.negate();
         lungeAxis.set(tempLungeAxis);
         
         pelvisFrame = pelvis.getBodyFixedFrame();
         this.wrenchOnPelvisAngularPart = desiredLungingTorqeicpRecoverDecelerateState();
         this.wrenchOnPelvisLinearPart = new Vector3d();
      }
      
      public void doTransitionOutOfAction()
      {
         setWrenchOnChestToZero(wrenchOnPelvis);
         storeWrenchCopyAsYoVariable(wrenchOnPelvis);
         spineControlModule.setWrench(wrenchOnPelvis);
         spineControlModule.setGains(); // may not be needed
      }
      
      private void storeWrenchCopyAsYoVariable(Wrench wrench)
      {
         wrenchOnPelvisAngular.set(wrench.getAngularPartCopy());
         wrenchOnPelvisLinear.set(wrench.getLinearPartCopy());
         
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
         
         if (doWeNeedToSlowDownBecauseOfAngleLimit) System.out.println("Transition to slow down due to Angle Limit.");
         if (willICPEndUpInsideStopLungingRadius) System.out.println("Transition to slow down due favorable predicted ICP location after slow down.");

//         return doWeNeedToSlowDownBecauseOfAngleLimit || willICPEndUpInsideStopLungingRadius;
         return false;
      }

      private void updateAngularVelocityAndAcceleration()
      {
         // TODO fix this bullshit.
         SpatialAccelerationVector chestAccelerationVector = processedSensors.getAccelerationOfPelvisWithRespectToWorld();
         chestAngularAcceleration.set(chestAccelerationVector.getAngularPartCopy().length());
      }

      private boolean doWeNeedToSlowDownBecauseOfAngleLimit()
      {
         
         double deltaTimeInState = processedSensors.getTime() - timeOfStateStart.get((BalancingUpperBodySubControllerState) stateMachine.getCurrentStateEnum());
         double deltaChestAngleInState = chestAngle.getDoubleValue() - 0.0;
         
         double averageChestVelocityInState = deltaChestAngleInState / deltaTimeInState;
         
         return (maxChestAngle.getDoubleValue() <= chestAngle.getDoubleValue() + deltaTimeInState * averageChestVelocityInState * kTimeToSlowSlowDown.getDoubleValue());  
      }

      /**
       * Predict where the ICP will be located when the chest comes to rest if we begin to decelerate the chest now.
       * Assume that the CoP, net torque on the chest, and resulting CmP are constant during deceleration.
       * First, predict the time it takes to decelerate the chest based upon the constant torque and a known chest moment of inertia.
       * Using the first-order ICP dynamics, solve for the change in ICP position over this time period, given the anticipated *constant* CmP location.
       * @return
       */
      private boolean willICPEndUpFarEnoughBack()
      {  
         ReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
         
         Vector3d predictedSpineTorque = desiredLungingTorqeicpRecoverDecelerateState();

         FramePoint2d deltaCMPDueToSpineTorque = new FramePoint2d(midFeetZUpFrame, deltaCMPDueToSpineTorque(predictedSpineTorque).getX(), deltaCMPDueToSpineTorque(predictedSpineTorque).getY()); //TODO: Should just express everything in FrameVectors to begin with

         FramePoint2d predictedCmpLocation = desiredCoPduringICPRecoverDecelerateState(midFeetZUpFrame);
         predictedCmpLocation.add(deltaCMPDueToSpineTorque);
         
         predictedCmpLocation.scale(2.0);  //TODO: Use this parameter to scale the desired IcP overshoot

         //FramePoint2d Copy of Current ICP Location
         FramePoint2d icpCurrent = couplingRegistry.getCapturePointInFrame(midFeetZUpFrame).toFramePoint2d();
         
         //Predicted ICP Location at timeToStop if we decelerate now
         FramePoint2d predictedICPWhenChestStops = new FramePoint2d(midFeetZUpFrame);
         predictedICPWhenChestStops = predictedICPWhenChestStopsIfWeSlowDownNow(icpCurrent, predictedCmpLocation, predictedTimeToStopChest());
         predictedICPAtTimeToStopVisualizer.set(predictedICPWhenChestStops.changeFrameCopy(ReferenceFrame.getWorldFrame()));
         
         //Vector from desired to predicted ICP (if this is zero, then we should start to decelerate now)
         FrameVector2d vectorFromDesiredToPredictedICP = new FrameVector2d(icpDesiredLunging.getFramePoint2dCopy().changeFrameCopy(midFeetZUpFrame), predictedICPWhenChestStops);

         double lengthOfVectorFromDesiredToPredictedICPProjectedAlongInitialICPDirection = vectorFromDesiredToPredictedICP.dot(getUnitVectorNormalToLungeAxis(midFeetZUpFrame));
       
         // Transition to IcpRecoverDecelerate when the component of predicted ICP along the initial lunge direction crosses zero
         // Use an inequality check to eliminate the need for an epsilon
         return lengthOfVectorFromDesiredToPredictedICPProjectedAlongInitialICPDirection < stopLungingICPRadius.getDoubleValue();
      }
   }
   
   private double predictedTimeToStopChest()
   {
      //    double projectedMOI = 1.0;
      //    double torque = 1.0;
      //    double predictedChestAngularDecel = torque / projectedMOI;

      double predictedChestAngularDecel = 10.0 * Math.signum(chestAngularVelocity.getDoubleValue()); //TODO: This should be computed using: predictedChestAngularDecel = torque / projectedMOI

      //Compute the elapsed time required to bring the chest rotation to zero, if we start to decelerate NOW.
      //Assume that chest undergoes a constant angular acceleration = maxHipTorque / projectedMomentOfInertia <- this is computed in updateAngularVelocityAndAcceleration
      double estimatedTimeToStopChest = chestAngularVelocity.getDoubleValue() / predictedChestAngularDecel;

      if (estimatedTimeToStopChest <= 0.0 ) throw new RuntimeException("elapsedTimeToStop must be greater than 0!");
      return estimatedTimeToStopChest;
   }
   
   private FramePoint2d predictedICPWhenChestStopsIfWeSlowDownNow(FramePoint2d icpCurrent, FramePoint2d cmpDuringChestDecelerate, double elapsedTimeToStop)
   {
      //Predict where the ICP will be at the time the chest comes to rest, if we start to decelerate NOW.
      //Assume a constant CmP location proportional to the maxHipTorque.
      double gravity = Math.abs(this.gravity);
      double CoMHeight = processedSensors.getCenterOfMassPositionInFrame(ReferenceFrame.getWorldFrame()).getZ();
      double omega0 = Math.sqrt( gravity / CoMHeight );  // Natural Frequency of LIPM
      
      //Predict ICP location: icp(timeCurrent + elapsedTimeToStop) = cmp(timeCurrent) + [ icp(timeCurrent) - cmp(timeCurrent) ] * exp(omega0 * elapsedTimeToStop)
      double deltaTPrime = omega0 * elapsedTimeToStop;
      
      
      FramePoint2d icpPredictedAtTimeToStop = new FramePoint2d(icpCurrent.getReferenceFrame(), icpCurrent.getPointCopy());
      icpPredictedAtTimeToStop.sub(cmpDuringChestDecelerate);
      icpPredictedAtTimeToStop.scale(Math.exp(deltaTPrime));
      icpPredictedAtTimeToStop.add(cmpDuringChestDecelerate);

      return icpPredictedAtTimeToStop;
   }
   
   
   private Vector2d deltaCMPDueToSpineTorque (Vector3d netSpineTorque)
   {
      double mass = this.robotMass;
      double gravity = this.gravity;
      
      Vector2d deltaCMP = new Vector2d();
      deltaCMP.set(-netSpineTorque.getY(), netSpineTorque.getX());
      deltaCMP.scale( 1 / (mass*gravity) );
      return deltaCMP;
   }
   
   private Vector3d SpineTorqueForDesiredDeltaCMP (Vector2d deltaCMP)
   {
      double mass = this.robotMass;
      double gravity = this.gravity;
      
      Vector3d netSpineTorque = new Vector3d();
      netSpineTorque.set(-deltaCMP.getY(), deltaCMP.getX(), 0.0);
      netSpineTorque.scale(mass*gravity);
      return netSpineTorque;
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
      
      icpDesiredLunging.set(new FramePoint2d(referenceFrames.getMidFeetZUpFrame()).changeFrameCopy(ReferenceFrame.getWorldFrame()));
      desiredICPVisualizer.set(icpDesiredLunging);
      
      lungeDesiredCoP = new YoFramePoint2d("lungeDesiredCoP", "", referenceFrames.getMidFeetZUpFrame(), registry);
      bosRadiusAlongLungeAxis.set(0.0);  //TODO: This should be more representative of the actual base of support, which varies depending on single/double support and stance width in double support 
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
      maxChestAngle.set(Math.PI / 2.0);
      kTimeToSlowSlowDown.set(1.4);
      // forcing controller into state
      forceControllerIntoState.set(false);
      stopLungingICPRadius.set(0.0);
      forcedControllerState.set(BalancingUpperBodySubControllerState.ICP_REC_ACC);
   }
   
 
   private FrameVector2d getUnitVectorNormalToLungeAxis(ReferenceFrame desiredFrame)
   {
      // Lunge axis and ICP direction are just orthogonal in the x-y plane
      FrameVector2d icpDirection = new FrameVector2d(lungeAxis.getReferenceFrame(), lungeAxis.getY(), -lungeAxis.getX());
      icpDirection.changeFrame(desiredFrame);
      icpDirection.normalize();
      return icpDirection;
   }
   
   /**
    * The desired *constant* spine torque to apply during ICPRecoverAccelerateState().
    * This method may be used in order to predict future IcP locations, and should therefore also be referenced by the actual state, in order to ensure consistency.
    * @param wrenchAngularPartToPack
    */
   private Vector3d getDesiredLungingTorqueicpRecoverAccelerateState()
   {
      Vector3d wrenchAngularPart  = new Vector3d(lungeAxis.getX(), lungeAxis.getY(), 0.0);
      wrenchAngularPart.scale(maxHipTorque.getDoubleValue());
      return wrenchAngularPart;
   }
   
   /**
    * The desired *constant* spine torque to apply during ICPRecoverDecelerateState().
    * This method may be used in order to predict future IcP locations, and should therefore also be referenced by the actual state, in order to ensure consistency.
    * @param wrenchAngularPartToPack
    */
   private Vector3d desiredLungingTorqeicpRecoverDecelerateState()
   {
      Vector3d wrenchAngularPart  = new Vector3d(lungeAxis.getX(), lungeAxis.getY(), 0.0);
      wrenchAngularPart.scale(-maxHipTorque.getDoubleValue());
      return wrenchAngularPart;
   }
   
   /**
    * The desired *constant* CoP location during ICPRecoverAccelerateState().
    * This method may be used in order to predict future IcP locations, and should therefore also be referenced by the actual state, in order to ensure consistency.
    */
   private FramePoint2d desiredCoPduringICPRecoverAccelerateState(ReferenceFrame referenceFrame)
   {
      FramePoint2d desiredCoP = new FramePoint2d(referenceFrame);

      FrameVector2d vectorFromMidFeetToCoP = lungeAxis.getFrameVector2dCopy();
      vectorFromMidFeetToCoP.normalize();
      vectorFromMidFeetToCoP.scale(bosRadiusAlongLungeAxis.getDoubleValue());
      vectorFromMidFeetToCoP.changeFrame(referenceFrame);
      
      desiredCoP.set(vectorFromMidFeetToCoP);
      return desiredCoP;
   }
   
   
   /**
    * The desired *constant* CoP location during ICPRecoverDecelerateState().
    * This method may be used in order to predict future IcP locations, and should therefore also be referenced by the actual state, in order to ensure consistency.
    */
   private FramePoint2d desiredCoPduringICPRecoverDecelerateState(ReferenceFrame referenceFrame)
   {
      FramePoint2d desiredCoP = new FramePoint2d(referenceFrame);

      FrameVector2d vectorFromMidFeetToCoP = lungeAxis.getFrameVector2dCopy();
      vectorFromMidFeetToCoP.normalize();
      vectorFromMidFeetToCoP.scale(bosRadiusAlongLungeAxis.getDoubleValue());
      vectorFromMidFeetToCoP.changeFrame(referenceFrame);
      
      desiredCoP.set(vectorFromMidFeetToCoP);
      return desiredCoP;
   }
   
   private void setIcpDesiredLunging(double newX, double newY)
   {
      icpDesiredLunging.set(newX, newY);
      desiredICPVisualizer.set(icpDesiredLunging);
   }

   private void displayWarningsForBooleans()
   {
      if (forceControllerIntoState.getBooleanValue())
      {
         System.out.println("Warning! Controller " + this.name + " is forced to remain in the " + forcedControllerState.toString() + " state!");
      }
   }

   private double getChestAngleToWorld()
   {
      Quat4d chestToWorld = processedSensors.getChestOrientationInFrame(ReferenceFrame.getWorldFrame()).getQuaternion();
      AxisAngle4d chestToWorldAxisAngle = new AxisAngle4d();
      chestToWorldAxisAngle.set(chestToWorld);

      double bodyAngleToWorld = chestToWorldAxisAngle.getAngle();
      return bodyAngleToWorld;
   }

}
