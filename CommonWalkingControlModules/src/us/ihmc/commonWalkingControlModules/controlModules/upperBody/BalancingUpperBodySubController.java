package us.ihmc.commonWalkingControlModules.controlModules.upperBody;

import java.util.ArrayList;
import java.util.EnumMap;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
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
   private YoFramePoint2d desiredCMP;
   
   private EnumMap<BalancingUpperBodySubControllerState, Double> timeOfStateStart = new EnumMap<BalancingUpperBodySubController.BalancingUpperBodySubControllerState, Double>(BalancingUpperBodySubControllerState.class);
   private EnumMap<BalancingUpperBodySubControllerState, Double> chestAngleOnStateStart = new EnumMap<BalancingUpperBodySubController.BalancingUpperBodySubControllerState, Double>(BalancingUpperBodySubControllerState.class);
   
   private final BooleanYoVariable forceControllerIntoState = new BooleanYoVariable("force" + name + "IntoState", registry);
   private final EnumYoVariable<BalancingUpperBodySubControllerState> forcedControllerState = new EnumYoVariable<BalancingUpperBodySubControllerState>("forced"
         + name + "State", registry, BalancingUpperBodySubControllerState.class);
   
   
   private final boolean USE_INVERSE_DYNAMICS_FOR_LUNGING = false;
   

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
   }
   


   private void setUpStateMachine()
   {
      State base = new BaseState();
      State icpRecoverAccelerateState = new ICPRecoverAccelerateState();
      State icpRecoverDecelerateState = new ICPRecoverDecelerateState();
      State orientationRecoverAccelerateState = new OrientationRecoverAccelerateState();
      State orientationRecoverDecelerateState = new OrientationRecoverDecelerateState();

      StateTransitionCondition isICPOutsideLungeRadius = new IsICPOutsideLungeRadiusCondition();
      StateTransitionCondition doWeNeedToDecelerate = new DoWeNeedToDecelerate();
      StateTransitionCondition isBodyAngularVelocityZeroISH = new HasBodyAngularVelocityCrossedZeroCondition(1e-2);
      StateTransitionCondition doWeNeedToDecelerateAgain = new DoWeNeedToDecelerateAgainCondition();
      

      StateTransition toICPRecoverAccelerate = new StateTransition(icpRecoverAccelerateState.getStateEnum(), isICPOutsideLungeRadius);
      StateTransition toICPRecoverDecelerate = new StateTransition(icpRecoverDecelerateState.getStateEnum(), doWeNeedToDecelerate);
      StateTransition toOrientationRecoverAccelerate = new StateTransition(orientationRecoverAccelerateState.getStateEnum(), isBodyAngularVelocityZeroISH);
      StateTransition toOrientationRecoverDecelerate = new StateTransition(orientationRecoverDecelerateState.getStateEnum(), doWeNeedToDecelerateAgain);
      StateTransition toBase = new StateTransition(base.getStateEnum(), isBodyAngularVelocityZeroISH);   //TODO: This should be based on angular position, not velocity

      base.addStateTransition(toICPRecoverAccelerate);
      icpRecoverAccelerateState.addStateTransition(toICPRecoverDecelerate);
      icpRecoverDecelerateState.addStateTransition(toBase);   //TODO now going back to base
//      icpRecoverDecelerateState.addStateTransition(toOrientationRecoverAccelerate);
//      orientationRecoverAccelerateState.addStateTransition(toOrientationRecoverDecelerate);
//      orientationRecoverDecelerateState.addStateTransition(toBase);

      stateMachine.addState(base);
      stateMachine.addState(icpRecoverAccelerateState);
      stateMachine.addState(icpRecoverDecelerateState);
      stateMachine.addState(orientationRecoverAccelerateState);
      stateMachine.addState(orientationRecoverDecelerateState);

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
      BASE, ICP_REC_ACC, ICP_REC_DEC, ORIENT_REC_ACC, ORIENT_REC_DEC;
   }

   private class BaseState extends State
   {
      public BaseState()
      {
         super(BalancingUpperBodySubControllerState.BASE);
      }

      public void doTransitionIntoAction()
      {
         setLungeAxisInWorldFrame(0.0, 0.0);
         timeOfStateStart.put((BalancingUpperBodySubControllerState)this.getStateEnum(), processedSensors.getTime());
      }
      
      public void doAction()
      {
         spineControlModule.doMaintainDesiredChestOrientation();
      }

      public void doTransitionOutOfAction()
      {
         System.out.println("Exit Base State");
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
      private static final boolean USE_CONSTANT_WRENCH_AROUND_LUNGEAXIS = true;
      private static final boolean USE_CONSTANT_TORQUE_AROUND_LUNGEAXIS = false;
      private static final boolean USE_CMP_CONTROL = false;
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
         
         if (USE_INVERSE_DYNAMICS_FOR_LUNGING)
         {
            if (USE_CONSTANT_WRENCH_AROUND_LUNGEAXIS)
            {
               wrenchOnPelvis = new Wrench(pelvisFrame, pelvisFrame, wrenchOnPelvisLinearPart, wrenchOnPelvisAngularPart);
               
               storeWrenchCopyAsYoVariable(wrenchOnPelvis);

               spineControlModule.setWrench(wrenchOnPelvis);
               spineControlModule.doMaintainDesiredChestOrientation();
            }
            
            if (USE_CONSTANT_TORQUE_AROUND_LUNGEAXIS)
            {
               spineControlModule.doConstantTorqueAroundLungeAxis(couplingRegistry.getLungeAxisInFrame(ReferenceFrame.getWorldFrame()), maxHipTorque.getDoubleValue());
               spineControlModule.doMaintainDesiredChestOrientation();
            }
            else if (USE_CMP_CONTROL)
            {
               spineControlModule.doCoPToCMPDistanceControl(desiredCMP.getFramePoint2dCopy());
               spineControlModule.doMaintainDesiredChestOrientation();
            }

         }
         else
         {
            Vector3d hipTorque = desiredLungingTorqueDuring(BalancingUpperBodySubControllerState.ICP_REC_ACC);
            hipTorque.negate();
            spineControlModule.setHipXYTorque(hipTorque);
         }
      }

      public void doTransitionIntoAction()
      {
         setLungeAxisBasedOnIcp();

         if (USE_SCALING_INSTEAD_OF_SETTING_TO_ZERO)
         {
            // scaling
            spineControlModule.scaleGainsBasedOnLungeAxis(couplingRegistry.getLungeAxisInFrame(ReferenceFrame.getWorldFrame()).getVectorCopy());
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
         this.wrenchOnPelvisAngularPart = desiredLungingTorqueDuring(BalancingUpperBodySubControllerState.ICP_REC_ACC);
         this.wrenchOnPelvisLinearPart = new Vector3d();

         timeOfStateStart.put((BalancingUpperBodySubControllerState)this.getStateEnum(), processedSensors.getTime());
         chestAngleOnStateStart.put((BalancingUpperBodySubControllerState)this.getStateEnum(), getChestAngleToWorld());
         
         if (USE_CMP_CONTROL)
         {
            desiredCMP.set(getInitialIcpDirection(ReferenceFrame.getWorldFrame()));
         }
       }

      public void doTransitionOutOfAction()
      {
         if (USE_INVERSE_DYNAMICS_FOR_LUNGING)
         {
            if (USE_CONSTANT_WRENCH_AROUND_LUNGEAXIS)
            {
               setWrenchOnChestToZero(wrenchOnPelvis);
               storeWrenchCopyAsYoVariable(wrenchOnPelvis);
               spineControlModule.setWrench(wrenchOnPelvis);
               spineControlModule.setGains(); // may not be needed               
            }
         }
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
         if (USE_INVERSE_DYNAMICS_FOR_LUNGING)
         {
            wrenchOnPelvis = new Wrench(pelvisFrame, pelvisFrame, wrenchOnPelvisLinearPart, wrenchOnPelvisAngularPart);
            
            storeWrenchCopyAsYoVariable(wrenchOnPelvis);

            spineControlModule.setWrench(wrenchOnPelvis);
            spineControlModule.doMaintainDesiredChestOrientation(); 
         }
         else
         {
            Vector3d hipTorque = desiredLungingTorqueDuring(BalancingUpperBodySubControllerState.ICP_REC_DEC);
            hipTorque.negate();
            spineControlModule.setHipXYTorque(hipTorque);
         }
      }

      public void doTransitionIntoAction()
      {
         timeOfStateStart.put((BalancingUpperBodySubControllerState)this.getStateEnum(), processedSensors.getTime());

         flipVisualizedLungeAxisGraphic();

         pelvisFrame = pelvis.getBodyFixedFrame();
         this.wrenchOnPelvisAngularPart = desiredLungingTorqueDuring(BalancingUpperBodySubControllerState.ICP_REC_DEC);
         this.wrenchOnPelvisLinearPart = new Vector3d();
      }
      
      public void doTransitionOutOfAction()
      {
         if (USE_INVERSE_DYNAMICS_FOR_LUNGING)
         {
            setWrenchOnChestToZero(wrenchOnPelvis);
            storeWrenchCopyAsYoVariable(wrenchOnPelvis);
            spineControlModule.setWrench(wrenchOnPelvis);
            spineControlModule.setGains(); // may not be needed
         }
      }
      
      private void storeWrenchCopyAsYoVariable(Wrench wrench)
      {
         wrenchOnPelvisAngular.set(wrench.getAngularPartCopy());
         wrenchOnPelvisLinear.set(wrench.getLinearPartCopy());
         
      }
   }
   
   private class OrientationRecoverAccelerateState extends State
   {
      Wrench wrenchOnPelvis;
      ReferenceFrame pelvisFrame;
      Vector3d wrenchOnPelvisLinearPart;
      Vector3d wrenchOnPelvisAngularPart;
      
      public OrientationRecoverAccelerateState()
      {
         super(BalancingUpperBodySubControllerState.ORIENT_REC_ACC);
      }

      public void doAction()
      {
         if (USE_INVERSE_DYNAMICS_FOR_LUNGING)
         {
            wrenchOnPelvis = new Wrench(pelvisFrame, pelvisFrame, wrenchOnPelvisLinearPart, wrenchOnPelvisAngularPart);
            
            storeWrenchCopyAsYoVariable(wrenchOnPelvis);

            spineControlModule.setWrench(wrenchOnPelvis);
            spineControlModule.doMaintainDesiredChestOrientation(); 
         }
         else
         {
            Vector3d hipTorque = desiredLungingTorqueDuring(BalancingUpperBodySubControllerState.ORIENT_REC_ACC);
            hipTorque.negate();
            spineControlModule.setHipXYTorque(hipTorque);
         }
         
      }

      public void doTransitionIntoAction()
      {
         timeOfStateStart.put((BalancingUpperBodySubControllerState)this.getStateEnum(), processedSensors.getTime());

         pelvisFrame = pelvis.getBodyFixedFrame();
         this.wrenchOnPelvisAngularPart = desiredLungingTorqueDuring(BalancingUpperBodySubControllerState.ORIENT_REC_ACC);
         this.wrenchOnPelvisLinearPart = new Vector3d();
      }

      public void doTransitionOutOfAction()
      {
         if (USE_INVERSE_DYNAMICS_FOR_LUNGING)
         {
            setWrenchOnChestToZero(wrenchOnPelvis);
            storeWrenchCopyAsYoVariable(wrenchOnPelvis);
            spineControlModule.setWrench(wrenchOnPelvis);
            spineControlModule.setGains(); // may not be needed
         }         
      }
   }
   
   private class OrientationRecoverDecelerateState extends State
   {
      Wrench wrenchOnPelvis;
      ReferenceFrame pelvisFrame;
      Vector3d wrenchOnPelvisLinearPart;
      Vector3d wrenchOnPelvisAngularPart;
      
      public OrientationRecoverDecelerateState()
      {
         super(BalancingUpperBodySubControllerState.ORIENT_REC_DEC);
      }

      public void doAction()
      {
         if (USE_INVERSE_DYNAMICS_FOR_LUNGING)
         {
            wrenchOnPelvis = new Wrench(pelvisFrame, pelvisFrame, wrenchOnPelvisLinearPart, wrenchOnPelvisAngularPart);
            
            storeWrenchCopyAsYoVariable(wrenchOnPelvis);

            spineControlModule.setWrench(wrenchOnPelvis);
            spineControlModule.doMaintainDesiredChestOrientation(); 
         }
         else
         {
            Vector3d hipTorque = desiredLungingTorqueDuring(BalancingUpperBodySubControllerState.ORIENT_REC_DEC);
            hipTorque.negate();
            spineControlModule.setHipXYTorque(hipTorque);
         }
      }

      public void doTransitionIntoAction()
      {
         timeOfStateStart.put((BalancingUpperBodySubControllerState)this.getStateEnum(), processedSensors.getTime());

         flipVisualizedLungeAxisGraphic();
         
         pelvisFrame = pelvis.getBodyFixedFrame();
         this.wrenchOnPelvisAngularPart = desiredLungingTorqueDuring(BalancingUpperBodySubControllerState.ORIENT_REC_DEC);
         this.wrenchOnPelvisLinearPart = new Vector3d();
      }
      
      public void doTransitionOutOfAction()
      {
         if (USE_INVERSE_DYNAMICS_FOR_LUNGING)
         {
            setWrenchOnChestToZero(wrenchOnPelvis);
            storeWrenchCopyAsYoVariable(wrenchOnPelvis);
            spineControlModule.setWrench(wrenchOnPelvis);
            spineControlModule.setGains(); // may not be needed
         }         
      }
   }
   
   private void setLungeAxisBasedOnIcp()
   {
      // TODO taking the capturePoint in the world frame will not work if the robot walks
      FramePoint2d capturePointInBodyAttachedZUP = new FramePoint2d(ReferenceFrame.getWorldFrame());
      capturePointInBodyAttachedZUP.set(couplingRegistry.getCapturePointInFrame(ReferenceFrame.getWorldFrame()).toFramePoint2d());

      setLungeAxisInWorldFrame(-capturePointInBodyAttachedZUP.getY(), capturePointInBodyAttachedZUP.getX());
      
//      lungeAxis.set(-capturePointInBodyAttachedZUP.getY(), capturePointInBodyAttachedZUP.getX());
//      lungeAxis.normalize();
//      lungeAxisGraphic.setXY(lungeAxis.getVectorCopy());
//      couplingRegistry.setLungeAxis(lungeAxis.getFrameVector2dCopy());
   }
   
   
   private void flipVisualizedLungeAxisGraphic()
   {
      Vector2d temp = couplingRegistry.getLungeAxisInFrame(ReferenceFrame.getWorldFrame()).getVectorCopy();
      temp.negate();
      lungeAxisGraphic.setX(temp.getX()); lungeAxisGraphic.setY(temp.getY());
   }
   
   private void setLungeAxisInWorldFrame(double newX, double newY)
   {
      FrameVector2d tempLungeAxis = new FrameVector2d(ReferenceFrame.getWorldFrame(), newX, newY);
      if (tempLungeAxis.length() != 0.0)
      {
         tempLungeAxis.normalize();
      }

      lungeAxisGraphic.set(tempLungeAxis.getX(), tempLungeAxis.getY(), 0.0);
      couplingRegistry.setLungeAxis(tempLungeAxis);
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

         return doWeNeedToSlowDownBecauseOfAngleLimit || willICPEndUpInsideStopLungingRadius;
//         return false;
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
         BalancingUpperBodySubControllerState currentState = BalancingUpperBodySubControllerState.ICP_REC_DEC;
         ReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
         
         Vector3d predictedSpineTorque = desiredLungingTorqueDuring(currentState);
//         Vector3d predictedSpineTorque = desiredLungingTorqeicpRecoverDecelerate();

         FramePoint2d deltaCMPDueToSpineTorque = new FramePoint2d(midFeetZUpFrame, deltaCMPDueToSpineTorque(predictedSpineTorque).getX(), deltaCMPDueToSpineTorque(predictedSpineTorque).getY()); //TODO: Should just express everything in FrameVectors to begin with

         FramePoint2d predictedCmpLocation = desiredCoPduring(currentState, midFeetZUpFrame);
         predictedCmpLocation.add(deltaCMPDueToSpineTorque);
         
         predictedCmpLocation.scale(0.05);  //TODO: Use this parameter to scale the desired IcP overshoot (0.05)

         //FramePoint2d Copy of Current ICP Location
         FramePoint2d icpCurrent = couplingRegistry.getCapturePointInFrame(midFeetZUpFrame).toFramePoint2d();
         
         //Predicted ICP Location at timeToStop if we decelerate now
         FramePoint2d predictedICPWhenChestStops = new FramePoint2d(midFeetZUpFrame);
         predictedICPWhenChestStops = predictedICPWhenChestStopsIfWeSlowDownNow(icpCurrent, predictedCmpLocation, predictedTimeToStopChestDuring(currentState));
         predictedICPAtTimeToStopVisualizer.set(predictedICPWhenChestStops.changeFrameCopy(ReferenceFrame.getWorldFrame()));
         
         //Vector from desired to predicted ICP (if this is zero, then we should start to decelerate now)
         FrameVector2d vectorFromDesiredToPredictedICP = new FrameVector2d(icpDesiredLunging.getFramePoint2dCopy().changeFrameCopy(midFeetZUpFrame), predictedICPWhenChestStops);

         double lengthOfVectorFromDesiredToPredictedICPProjectedAlongInitialICPDirection = vectorFromDesiredToPredictedICP.dot(getUnitVectorNormalToLungeAxis(midFeetZUpFrame));
       
         // Transition to IcpRecoverDecelerate when the component of predicted ICP along the initial lunge direction crosses zero
         // Use an inequality check to eliminate the need for an epsilon
         return lengthOfVectorFromDesiredToPredictedICPProjectedAlongInitialICPDirection < stopLungingICPRadius.getDoubleValue();
      }
   }
   
   private double predictedChestAccelerationDuring(BalancingUpperBodySubControllerState state)
   {
      //    double projectedMOI = 1.0;
//          double torque = desiredLungingTorqueDuring(state).length();
      //    double predictedChestAngularDecel = torque / projectedMOI;

      double predictedChestAngularDecel = 10.0 * Math.signum(chestAngularVelocity.getDoubleValue()); //TODO: This should be computed using: predictedChestAngularDecel = torque / projectedMOI
      return predictedChestAngularDecel;
   }
   
   private double predictedTimeToStopChestDuring(BalancingUpperBodySubControllerState state)
   {
      //Compute the elapsed time required to bring the chest rotation to zero, if we start to decelerate NOW.
      //Assume that chest undergoes a constant angular acceleration = maxHipTorque / projectedMomentOfInertia <- this is computed in updateAngularVelocityAndAcceleration
      double predictedTimeToStopChest = chestAngularVelocity.getDoubleValue() / predictedChestAccelerationDuring(state);

      if (predictedTimeToStopChest <= 0.0 ) throw new RuntimeException("predictedTimeToStopChest must be greater than 0!");
      return predictedTimeToStopChest;
   }
   
   private double predictedDistanceToStopChestDuring(BalancingUpperBodySubControllerState state)
   {
      double deltaT = predictedTimeToStopChestDuring(state);
      double stopDistance = chestAngularVelocity.getDoubleValue() * deltaT - 0.5 * predictedChestAccelerationDuring(state) * deltaT * deltaT;
      
      return stopDistance;
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
   
   private Vector3d SpineTorqueForDesiredDeltaCMP (Vector2d deltaCMP, ReferenceFrame referenceFrame)
   {
      double mass = this.robotMass;
      double gravity = this.gravity;
      
      Vector3d netSpineTorque = new Vector3d();
      netSpineTorque.set(-deltaCMP.getY(), deltaCMP.getX(), 0.0);
      netSpineTorque.scale(mass*gravity);
      return netSpineTorque;
   }

   private class HasBodyAngularVelocityCrossedZeroCondition implements StateTransitionCondition
   {
      private final double epsilon;

      public HasBodyAngularVelocityCrossedZeroCondition(double epsilon)
      {
         this.epsilon = epsilon;
      }

      public boolean checkCondition()
      {
         ReferenceFrame lungeAxisExpressedInFrame = ReferenceFrame.getWorldFrame();
         FrameVector2d spineXYAngularVelocity = new FrameVector2d(lungeAxisExpressedInFrame);
         spineXYAngularVelocity.set(processedSensors.getSpineJointVelocity(SpineJointName.SPINE_ROLL), processedSensors.getSpineJointVelocity(SpineJointName.SPINE_PITCH));
         
         double angularVelocityProjectedAlongLungeAxis = Math.abs(spineXYAngularVelocity.dot(couplingRegistry.getLungeAxisInFrame(lungeAxisExpressedInFrame)));
         
//         System.out.println("Projected Spine Angular Velocity: " + angularVelocityProjectedAlongLungeAxis);
         
         boolean ret = angularVelocityProjectedAlongLungeAxis < epsilon;
         return ret;
      }
   }
   
   private class DoWeNeedToDecelerateAgainCondition implements StateTransitionCondition
   {
      public boolean checkCondition()
      {
         return processedSensors.getSpineJointPosition(SpineJointName.SPINE_PITCH) - predictedDistanceToStopChestDuring(BalancingUpperBodySubControllerState.ORIENT_REC_ACC) <= 0.0;
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
      
      desiredCMP = new YoFramePoint2d("desiredCMP", "", ReferenceFrame.getWorldFrame(), registry);
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
      maxChestAngle.set(Math.PI);
      kTimeToSlowSlowDown.set(1.4);
      // forcing controller into state
      forceControllerIntoState.set(false);
      stopLungingICPRadius.set(0.0);
      forcedControllerState.set(BalancingUpperBodySubControllerState.ICP_REC_ACC);
      setLungeAxisInWorldFrame(0.0, 0.0);
   }
   
 
   private FrameVector2d getUnitVectorNormalToLungeAxis(ReferenceFrame desiredFrame)
   {
      // Lunge axis and ICP direction are just orthogonal in the x-y plane
      FrameVector2d lungeAxis = couplingRegistry.getLungeAxisInFrame(ReferenceFrame.getWorldFrame());
      FrameVector2d icpDirection = new FrameVector2d(lungeAxis.getReferenceFrame(), lungeAxis.getY(), -lungeAxis.getX());
      icpDirection.changeFrame(desiredFrame);
      icpDirection.normalize();
      return icpDirection;
   }
   
 /**
 * The desired *constant* spine torque to apply during the corresponding BalancingUpperBodySubControllerState, based on the current lungeAxis and maxHipTorque.
 * This method may be used in order to predict future IcP locations, and should therefore also be referenced by the actual state, in order to ensure consistency.
 * @param wrenchAngularPartToPack
 */
   private Vector3d desiredLungingTorqueDuring(BalancingUpperBodySubControllerState state)
   {
      Vector3d lungingTorque  = new Vector3d();  //TODO: In which frame is torque expressed?
      FrameVector2d lungeAxis = couplingRegistry.getLungeAxisInFrame(ReferenceFrame.getWorldFrame());
      lungingTorque.set(lungeAxis.getX(), lungeAxis.getY(), 0.0);
      switch (state)
      {
      case BASE:
         lungingTorque.scale(0.0);
         break;
      case ICP_REC_ACC:
         lungingTorque.scale(maxHipTorque.getDoubleValue());
         break;
      case ICP_REC_DEC:
         lungingTorque.scale(-1.5*maxHipTorque.getDoubleValue());
         break;
      case ORIENT_REC_ACC:
//         FramePoint2d desiredCoP = desiredCoPduring(BalancingUpperBodySubControllerState.ORIENT_REC_ACC, referenceFrames.getMidFeetZUpFrame());
//         SpineTorqueForDesiredDeltaCMP(desiredCoP, referenceFrames.getMidFeetZUpFrame());
//         lungingTorque.scale();
         
         lungingTorque.scale(-1.0*maxHipTorque.getDoubleValue());
         break;
      case ORIENT_REC_DEC:
         lungingTorque.scale(0.5*maxHipTorque.getDoubleValue());

      default:
         break;
      }

      return lungingTorque;
   }
   
   
   /**
    * The desired *constant* CoP location during each BalancingUpperBodySubControllerState.
    * This method may be used in order to predict future IcP locations, and should therefore also be referenced by the actual state, in order to ensure consistency.
    */
   private FramePoint2d desiredCoPduring(BalancingUpperBodySubControllerState state, ReferenceFrame referenceFrame)
   {
      FramePoint2d desiredCoP = new FramePoint2d(referenceFrame);
      FrameVector2d lungeAxis = couplingRegistry.getLungeAxisInFrame(ReferenceFrame.getWorldFrame());
      
      FrameVector2d vectorFromMidFeetToCoP = new FrameVector2d(lungeAxis.getReferenceFrame(), lungeAxis.getX(), lungeAxis.getY());
      vectorFromMidFeetToCoP.normalize();
      vectorFromMidFeetToCoP.scale(bosRadiusAlongLungeAxis.getDoubleValue());
      vectorFromMidFeetToCoP.changeFrame(referenceFrame);
      
      switch (state)
      {
      case BASE:
         desiredCoP.set(vectorFromMidFeetToCoP);
         break;
      case ICP_REC_ACC:
         desiredCoP.set(vectorFromMidFeetToCoP);
         break;
      case ICP_REC_DEC:
         desiredCoP.set(vectorFromMidFeetToCoP);
         break;
      case ORIENT_REC_ACC:
         desiredCoP.set(vectorFromMidFeetToCoP);
         break;
      case ORIENT_REC_DEC:
         desiredCoP.set(vectorFromMidFeetToCoP);

      default:
         break;
      }

      return desiredCoP;
   }
   
   
   private void setIcpDesiredLunging(double newX, double newY)  //TODO: THIS SHOULD BE SET AS THE CENTER OF THE BOS WHEN NOT WALKING
   {
      icpDesiredLunging.set(newX, newY);
      desiredICPVisualizer.set(icpDesiredLunging);
   }
   
   /**
    * The initial ICP direction is set in the DoTransitionIntoAction() method of the IcpRecoverAccelerateState
    * @param IcpDirectionToPack
    */
   private FramePoint2d getInitialIcpDirection(ReferenceFrame expressedInFrame)
   {
      FrameVector2d lungeAxis = couplingRegistry.getLungeAxisInFrame(ReferenceFrame.getWorldFrame());
      FramePoint2d icpDirection = new FramePoint2d(lungeAxis.getReferenceFrame(), lungeAxis.getY(), -lungeAxis.getX());  // Lunge axis and ICP direction are just orthogonal in the x-y plane
      icpDirection.changeFrame(expressedInFrame);
      return icpDirection;
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
