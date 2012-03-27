package us.ihmc.commonWalkingControlModules.controlModules.upperBody;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.ArmControlModule;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.SpineLungingControlModule;
import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.UpperBodySubController;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.ArmJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.ArmTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.NeckJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.NeckTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.SpineJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.SpineTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.UpperBodyTorques;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.containers.ContainerTools;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CompositeRigidBodyInertia;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.Wrench;

import com.mathworks.jama.Matrix;
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
import com.yobotics.simulationconstructionset.util.statemachines.State;
import com.yobotics.simulationconstructionset.util.statemachines.StateMachine;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransition;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransitionCondition;

public class BalancingUpperBodySubController implements UpperBodySubController
{
   private final boolean USE_ARMS_FOR_LUNGING = false;
   
   private final boolean LATCH_LUNGE_AXIS_TO_PITCH_OR_ROLL = true;
   private final boolean USE_INVERSE_DYNAMICS_FOR_LUNGING = false;
   public static final boolean SCALE_PITCH_ROLL_FEEDBACK_GAINS_PROPORTIONAL_TO_LUNGE_AXIS_PITCH_ROLL_COMPONENTS = false;  //Otherwise just set feedback to zero when lunging
   

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

   private final RigidBody chest;
   private final RigidBody pelvis;
   private final ReferenceFrame pelvisFrame;
   private final double robotMass;
   private final double gravity;
   
   private ArrayList<InverseDynamicsJoint> allIDJointsAbovePelvis;
   private InverseDynamicsJoint spineRollIDjoint;
   private InverseDynamicsJoint spineYawIDjoint;
   private InverseDynamicsJoint spinePitchIDjoint;
   private CompositeRigidBodyInertia upperBodyMoI;
   private double upperBodyMoIProjectedOntoLungeAxis = 0.0;

   private final DoubleYoVariable maxChestAngleMagnitude = new DoubleYoVariable("maxChestAngle", registry);
   private final DoubleYoVariable startLungingICPRadius = new DoubleYoVariable("startLungingRadius", registry);
   private final DoubleYoVariable stopLungingICPRadius = new DoubleYoVariable("stopLungingRadius", registry);
   private final DoubleYoVariable maxHipTorque = new DoubleYoVariable("maxHipTorque", registry);
   private final DoubleYoVariable bosRadiusPerpToLungeAxis = new DoubleYoVariable("bosRadius", registry);
   private final DoubleYoVariable predictedCMPscaling = new DoubleYoVariable("predictedCMPscaling", registry);

   
   private Wrench desiredWrenchOnPelvis;
   private Wrench actualWrenchOnPelvis;

   private final YoFramePoint robotCoMPosition = new YoFramePoint("comGraphic", "", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint chestCoMPosition = new YoFramePoint("chestCoMGraphic", "", ReferenceFrame.getWorldFrame(), registry);
   
   private final YoFrameVector lungeAxisGraphic = new YoFrameVector("lungeAxisGraphic", "", ReferenceFrame.getWorldFrame(), registry);
   
   private final YoFramePoint2d icpDesiredLunging = new YoFramePoint2d("icpDesiredLunging", "", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint2d icpActual = new YoFramePoint2d("icpActual", "", ReferenceFrame.getWorldFrame(), registry);

   private final YoFramePoint2d desiredCMP = new YoFramePoint2d("desiredCMP", "", ReferenceFrame.getWorldFrame(), registry);
   

   private final DoubleYoVariable chestAngle = new DoubleYoVariable("chestAngle", registry);
   private final DoubleYoVariable chestAngularVelocityMagnitude = new DoubleYoVariable("chestAngularVelocity", registry);
   
   private final DoubleYoVariable predictedTimeToDecelerateChest = new DoubleYoVariable("predictedTimeToDecelerateChest", registry);
   private final DoubleYoVariable actualTimeToAccelerateChest = new DoubleYoVariable("actualTimeToAccelerateChest", registry);
   private final DoubleYoVariable actualTimeToDecelerateChest = new DoubleYoVariable("actualTimeToDecelerateChest", registry);
   
   private final YoFramePoint2d actualICPAtChestStop = new YoFramePoint2d("actualICPAtChestStop", "", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint2d predictedICPAtChestStop = new YoFramePoint2d("predictedICPAtChestStop", "", ReferenceFrame.getWorldFrame(), registry);

   private final YoFramePoint2d lungeAxisPerpIntersectionWithBoS = new YoFramePoint2d("lungeAxisBoSIntersection", "", ReferenceFrame.getWorldFrame(), registry);

   private final DoubleYoVariable kTimeToSlowSlowDown = new DoubleYoVariable("kTimeToSlowDown", registry);
   private final DoubleYoVariable kCMP = new DoubleYoVariable("kCMP", registry);
      
   private EnumMap<BalancingUpperBodySubControllerState, Double> timeOfStateStart = new EnumMap<BalancingUpperBodySubController.BalancingUpperBodySubControllerState, Double>(BalancingUpperBodySubControllerState.class);
   private EnumMap<BalancingUpperBodySubControllerState, Double> minimumElapsedTimeInState = new EnumMap<BalancingUpperBodySubControllerState, Double>(BalancingUpperBodySubControllerState.class);
   private EnumMap<BalancingUpperBodySubControllerState, Double> chestAngleOnStateStart = new EnumMap<BalancingUpperBodySubController.BalancingUpperBodySubControllerState, Double>(BalancingUpperBodySubControllerState.class);
   private EnumMap<BalancingUpperBodySubControllerState, Double> chestAngularVelMagnitudeAboutLungeAxisOnStateStart = new EnumMap<BalancingUpperBodySubController.BalancingUpperBodySubControllerState, Double>(BalancingUpperBodySubControllerState.class);
   
   private EnumMap<BalancingUpperBodySubControllerState, Double> desiredSpineLungingTorqueMagnitudes = new EnumMap<BalancingUpperBodySubControllerState, Double>(BalancingUpperBodySubControllerState.class);
   private EnumMap<BalancingUpperBodySubControllerState, Double> desiredArmLungingTorqueScaling = new EnumMap<BalancingUpperBodySubControllerState, Double>(BalancingUpperBodySubControllerState.class);
   
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
      this.pelvisFrame = pelvis.getBodyFixedFrame();
      this.upperBodyMoI = new CompositeRigidBodyInertia();
      this.stateMachine = new StateMachine(name + "State", name + "SwitchTime", BalancingUpperBodySubControllerState.class, processedSensors.getYoTime(),
            registry);
      this.desiredWrenchOnPelvis = new Wrench(pelvisFrame, pelvisFrame);

      populateAllIDjointsAbovePelvis();

      populateDynamicsGraphicObjects(dynamicGraphicObjectsListRegistry);
      populateYoVariables();

      populateControllers();
      parentRegistry.addChild(registry);

      setGains();
      setParameters();
      displayWarningsForBooleans();
      
      if (USE_ARMS_FOR_LUNGING)
      {
         setUpStateMachineSimpleWithArmsLunging();
      }
      else
      {
         setUpStateMachineSimple();
//         setUpStateMachineBalanceBeam();
      }
      
   }

   public void doUpperBodyControl(UpperBodyTorques upperBodyTorquesToPack)
{
      updateVariables();
      updateDynamicsGraphicObjects();
      
      stateMachine.doAction();
      if (!forceControllerIntoState.getBooleanValue())
      {
         stateMachine.checkTransitionConditions();
      } else
      {
         stateMachine.setCurrentState(forcedControllerState.getEnumValue());
      }
 
      if (USE_ARMS_FOR_LUNGING)
      {
         setArmTorquesDirectlyForLunging(upperBodyTorquesToPack);
      }
      else
      {
         armControlModule.doArmControl(upperBodyTorquesToPack.getArmTorques());
      }

      this.doNeckControl();
      upperBodyTorquesToPack.setNeckTorques(neckTorques);

      spineControlModule.getSpineTorques(upperBodyTorquesToPack.getSpineTorques());
      
      setActualWrenchExertedOnPelvis(upperBodyTorquesToPack);
      
      if (currentStateElapsedTime() < 0.0)  //TODO: This prevents problems when rewinding, but is inefficient.
      {
         timeOfStateStart.put((BalancingUpperBodySubControllerState) stateMachine.getCurrentStateEnum(), processedSensors.getTime());
      }
      
   }
   
   private void setUpStateMachineSimple()
   {
      State base = new BaseState();
      State icpRecoverAccelerateState = new ICPRecoverAccelerateState();
      State icpRecoverDecelerateState = new ICPRecoverDecelerateSpineState();

      StateTransitionCondition isICPOutsideLungeRadius = new DoWeNeedToStartLungingCondition();
      StateTransitionCondition doWeNeedToDecelerate = new DoWeNeedToDecelerateNow();
      StateTransitionCondition isBodyAngularVelocityZero = new HasChestAngularVelocityCrossedZeroCondition();
      
      StateTransition toICPRecoverAccelerate = new StateTransition(icpRecoverAccelerateState.getStateEnum(), isICPOutsideLungeRadius);
      StateTransition toICPRecoverDecelerate = new StateTransition(icpRecoverDecelerateState.getStateEnum(), doWeNeedToDecelerate);
      StateTransition toBase = new StateTransition(base.getStateEnum(), isBodyAngularVelocityZero);

      base.addStateTransition(toICPRecoverAccelerate);
      icpRecoverAccelerateState.addStateTransition(toICPRecoverDecelerate);
      icpRecoverDecelerateState.addStateTransition(toBase);   //TODO now going back to base

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
      
      minimumElapsedTimeInState.put(BalancingUpperBodySubControllerState.BASE, 100.0*controlDT);
      minimumElapsedTimeInState.put(BalancingUpperBodySubControllerState.ICP_REC_ACC, 10.0*controlDT);
      minimumElapsedTimeInState.put(BalancingUpperBodySubControllerState.ICP_REC_DEC, 10.0*controlDT);
   }
   
   private void setUpStateMachineBalanceBeam()
   {
      State base = new BaseState();
      State icpRecoverAccelerateState = new ICPRecoverAccelerateState();
      State icpRecoverDecelerateState = new ICPRecoverDecelerateSpineState();
      State orientationRecoverAccelerateState = new OrientationRecoverAccelerateState();
      State orientationRecoverDecelerateState = new OrientationRecoverDecelerateState();

      StateTransitionCondition isICPOutsideLungeRadius = new DoWeNeedToStartLungingCondition();
      StateTransitionCondition doWeNeedToDecelerate = new DoWeNeedToDecelerateNow();
      StateTransitionCondition isBodyAngularVelocityZero = new HasChestAngularVelocityCrossedZeroCondition();
      StateTransitionCondition doWeNeedToDecelerateAgain = new DecelerateNowToStopUpperBodyAtUprightCondition();
      StateTransitionCondition isChestUpright = new isChestFrameAlignedWithGravityCondition();
      
      StateTransition toICPRecoverAccelerate = new StateTransition(icpRecoverAccelerateState.getStateEnum(), isICPOutsideLungeRadius);
      StateTransition toICPRecoverDecelerate = new StateTransition(icpRecoverDecelerateState.getStateEnum(), doWeNeedToDecelerate);
      StateTransition toOrientationRecoverAccelerate = new StateTransition(orientationRecoverAccelerateState.getStateEnum(), isBodyAngularVelocityZero);
      StateTransition toOrientationRecoverDecelerate = new StateTransition(orientationRecoverDecelerateState.getStateEnum(), doWeNeedToDecelerateAgain);
      StateTransition toBase = new StateTransition(base.getStateEnum(), isChestUpright);

      base.addStateTransition(toICPRecoverAccelerate);
      icpRecoverAccelerateState.addStateTransition(toICPRecoverDecelerate);
      icpRecoverDecelerateState.addStateTransition(toOrientationRecoverAccelerate);
      orientationRecoverAccelerateState.addStateTransition(toOrientationRecoverDecelerate);
      orientationRecoverDecelerateState.addStateTransition(toBase);

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
      
      minimumElapsedTimeInState.put(BalancingUpperBodySubControllerState.BASE, 10.0*controlDT);
      minimumElapsedTimeInState.put(BalancingUpperBodySubControllerState.ICP_REC_ACC, 10.0*controlDT);
      minimumElapsedTimeInState.put(BalancingUpperBodySubControllerState.ICP_REC_DEC, 10.0*controlDT);
      minimumElapsedTimeInState.put(BalancingUpperBodySubControllerState.OR_REC_ACC, 1000.0*controlDT);
      minimumElapsedTimeInState.put(BalancingUpperBodySubControllerState.OR_REC_DEC, 10.0*controlDT);
   }
   

   private void setUpStateMachineSimpleWithArmsLunging()
   {
      State base = new BaseState();
      State icpRecoverAccelerateState = new ICPRecoverAccelerateState();
      State icpRecoverDecelerateSpineState = new ICPRecoverDecelerateSpineState();
      State icpRecoverDecelerateArmsState = new ICPRecoverDecelerateArmsState();

      StateTransitionCondition isICPOutsideLungeRadius = new DoWeNeedToStartLungingCondition();
      StateTransitionCondition doWeNeedToDecelerate = new DoWeNeedToDecelerateNow();
      StateTransitionCondition isBodyAngularVelocityZero = new HasChestAngularVelocityCrossedZeroCondition();
      StateTransitionCondition isArmAngularVelocityZero = new HasArmAngularVelocityCrossedZeroCondition();
      
      StateTransition toICPRecoverAccelerate = new StateTransition(icpRecoverAccelerateState.getStateEnum(), isICPOutsideLungeRadius);
      StateTransition toICPRecoverDecelerateSpine = new StateTransition(icpRecoverDecelerateSpineState.getStateEnum(), doWeNeedToDecelerate);
      StateTransition toICPRecoverDecelerateArms = new StateTransition(icpRecoverDecelerateArmsState.getStateEnum(), isBodyAngularVelocityZero);
      StateTransition toBase = new StateTransition(base.getStateEnum(), isArmAngularVelocityZero);   //TODO: This should be based on angular position, not velocity

      base.addStateTransition(toICPRecoverAccelerate);
      icpRecoverAccelerateState.addStateTransition(toICPRecoverDecelerateSpine);
      icpRecoverDecelerateSpineState.addStateTransition(toICPRecoverDecelerateArms);
      icpRecoverDecelerateArmsState.addStateTransition(toBase);
            
      stateMachine.addState(base);
      stateMachine.addState(icpRecoverAccelerateState);
      stateMachine.addState(icpRecoverDecelerateSpineState);
      stateMachine.addState(icpRecoverDecelerateArmsState);

      if (forceControllerIntoState.getBooleanValue())
      {
         stateMachine.setCurrentState(forcedControllerState.getEnumValue());
      } else
      {
         stateMachine.setCurrentState(base.getStateEnum());
      }
      
      minimumElapsedTimeInState.put(BalancingUpperBodySubControllerState.BASE, 1000.0*controlDT);
      minimumElapsedTimeInState.put(BalancingUpperBodySubControllerState.ICP_REC_ACC, 5.0*controlDT);
      minimumElapsedTimeInState.put(BalancingUpperBodySubControllerState.ICP_REC_DEC, 5.0*controlDT);
      minimumElapsedTimeInState.put(BalancingUpperBodySubControllerState.ICP_REC_DEC2, 5.0*controlDT);

   }

   private enum BalancingUpperBodySubControllerState
   {
      BASE, ICP_REC_ACC, ICP_REC_DEC, ICP_REC_DEC2, OR_REC_ACC, OR_REC_DEC;
   }

   
   private class BaseState extends State
   {
      private BalancingUpperBodySubControllerState currentState = BalancingUpperBodySubControllerState.BASE;

      public BaseState()
      {
         super(BalancingUpperBodySubControllerState.BASE);
      }

      public void doTransitionIntoAction()
      {
         timeOfStateStart.put(currentState, processedSensors.getTime());
         chestAngleOnStateStart.put(currentState, getChestAngleToWorld());
         setLungeAxisInWorldFrame(0.0, 0.0);
      }
      
      public void doAction()
      {
         spineControlModule.doSpineControlUsingIDwithPDfeedback();

//         UpperBodyTorques upperBodyTorquesToPack = new UpperBodyTorques();
//         spineControlModule.doSpineControl(upperBodyTorquesToPack.getSpineTorques());

      }

      public void doTransitionOutOfAction()
      {
         System.out.println("\n Exit Base State");
         SpineTorques torques = new SpineTorques();
         spineControlModule.getSpineTorques(torques);
      }
   }
   

   private class ICPRecoverAccelerateState extends State
   {
      private BalancingUpperBodySubControllerState currentState = BalancingUpperBodySubControllerState.ICP_REC_ACC;
     

      public ICPRecoverAccelerateState()
      {
         super(BalancingUpperBodySubControllerState.ICP_REC_ACC);
      }

      public void doAction()
      {
         if (USE_INVERSE_DYNAMICS_FOR_LUNGING)
         {
            lungeWithSpineUsingID();
         }
         else
         {
            lungeWithSpineUsingFixedPitchRollTorque();
         }
      }

      public void doTransitionIntoAction()
      {
         timeOfStateStart.put(currentState, processedSensors.getTime());
         
         setLungeAxisInCouplingRegistryBasedOnIcp();
         
         chestAngleOnStateStart.put(currentState, getChestAngleToWorld());
         setChestAngularVelocityMagnitudeAlongLungeAxisOnStateStart(currentState);
                  
         setDesiredWrenchOnPelvisAccordingToCurrentState(currentState);
                  
         if (USE_INVERSE_DYNAMICS_FOR_LUNGING)
         {
            if (SCALE_PITCH_ROLL_FEEDBACK_GAINS_PROPORTIONAL_TO_LUNGE_AXIS_PITCH_ROLL_COMPONENTS)
            {
               spineControlModule.scaleGainsBasedOnLungeAxis(couplingRegistry.getLungeAxisInFrame(ReferenceFrame.getWorldFrame()).getVectorCopy());
            }
            else
            {
               spineControlModule.scaleGainsToZero();            
            }
         }
       }

      public void doTransitionOutOfAction()
      {
         setDesiredWrenchOnPelvisToZero();
      }
   }
    

   private class ICPRecoverDecelerateSpineState extends State
   {
      private BalancingUpperBodySubControllerState currentState = BalancingUpperBodySubControllerState.ICP_REC_DEC;
      
      public ICPRecoverDecelerateSpineState()
      {
         super(BalancingUpperBodySubControllerState.ICP_REC_DEC);
      }

      public void doAction()
      {
         if (USE_INVERSE_DYNAMICS_FOR_LUNGING)
         {
            lungeWithSpineUsingID();
         }
         else
         {
            lungeWithSpineUsingFixedPitchRollTorque();
         }
      }

      public void doTransitionIntoAction()
      {
         timeOfStateStart.put(currentState, processedSensors.getTime());
         setChestAngularVelocityMagnitudeAlongLungeAxisOnStateStart(currentState);
         flipVisualizedLungeAxisGraphic();

         setDesiredWrenchOnPelvisAccordingToCurrentState(currentState);
      }
      
      public void doTransitionOutOfAction()
      {
         actualTimeToDecelerateChest.set(currentStateElapsedTime());
         actualICPAtChestStop.set(couplingRegistry.getCapturePointInFrame(ReferenceFrame.getWorldFrame()).toFramePoint2d());
         
         setDesiredWrenchOnPelvisToZero();
         
         System.out.println("Predicted ICP at Chest Stop: " + predictedICPAtChestStop);
         System.out.println("Actual ICP at Chest Stop: " + actualICPAtChestStop);
         
         System.out.println("Predicted Time to Stop Chest: " + predictedTimeToStopChestDuring(currentState));
         System.out.println("Actual Time to Stop Chest: " + actualTimeToDecelerateChest.getDoubleValue());
         
         if (USE_INVERSE_DYNAMICS_FOR_LUNGING)
         {
            spineControlModule.setGains(); // may not be needed
         }
      }
   }
   
   private class ICPRecoverDecelerateArmsState extends State
   {
      private BalancingUpperBodySubControllerState currentState = BalancingUpperBodySubControllerState.ICP_REC_DEC2;


      public ICPRecoverDecelerateArmsState()
      {
         super(BalancingUpperBodySubControllerState.ICP_REC_DEC2);
      }

      public void doAction()
      {
         if (USE_INVERSE_DYNAMICS_FOR_LUNGING)
         {
            lungeWithSpineUsingID();
         }
         else
         {
            lungeWithSpineUsingFixedPitchRollTorque();
         }         
      }

      public void doTransitionIntoAction()
      {
         timeOfStateStart.put(BalancingUpperBodySubControllerState.ICP_REC_DEC2, processedSensors.getTime()); 
         setDesiredWrenchOnPelvisAccordingToCurrentState(currentState);
      }

      public void doTransitionOutOfAction()
      {
         
      }
      
   }
   
   private class OrientationRecoverAccelerateState extends State
   {
      BalancingUpperBodySubControllerState currentState = BalancingUpperBodySubControllerState.OR_REC_ACC;

      
      public OrientationRecoverAccelerateState()
      {
         super(BalancingUpperBodySubControllerState.OR_REC_ACC);
      }

      public void doAction()
      {
         if (USE_INVERSE_DYNAMICS_FOR_LUNGING)
         {
            lungeWithSpineUsingID();
         }
         else
         {
            lungeWithSpineUsingFixedPitchRollTorque();
         }
      }

      public void doTransitionIntoAction()
      {
         timeOfStateStart.put(currentState, processedSensors.getTime());

         flipVisualizedLungeAxisGraphic();
         
         
         FrameVector2d vectorFromMidZFeetUpTodesiredCoP = new FrameVector2d(desiredCoPduring(currentState, referenceFrames.getMidFeetZUpFrame()));
         
         FrameVector bosLimitedLungingTorque = desiredLungingTorqueDuring(currentState, pelvisFrame);
         bosLimitedLungingTorque.normalize();
         bosLimitedLungingTorque.scale(vectorFromMidZFeetUpTodesiredCoP.length()*robotMass*gravity);
      
         desiredWrenchOnPelvis.setAngularPart(bosLimitedLungingTorque.getVectorCopy());
      }
      
      public void doTransitionOutOfAction()
      {
         setDesiredWrenchOnPelvisToZero();
         
         if (USE_INVERSE_DYNAMICS_FOR_LUNGING)
         {
            spineControlModule.setGains(); // may not be needed
         }
      }
   }
   
   private class OrientationRecoverDecelerateState extends State
   {
      public OrientationRecoverDecelerateState()
      {
         super(BalancingUpperBodySubControllerState.OR_REC_DEC);
      }

      public void doAction()
      {
         if (USE_INVERSE_DYNAMICS_FOR_LUNGING)
         {
            lungeWithSpineUsingID();
         }
         else
         {
            lungeWithSpineUsingFixedPitchRollTorque();
         }
      }

      public void doTransitionIntoAction()
      {
         timeOfStateStart.put(BalancingUpperBodySubControllerState.OR_REC_DEC, processedSensors.getTime());

         flipVisualizedLungeAxisGraphic();

         desiredWrenchOnPelvis.setAngularPart(desiredLungingTorqueDuring(BalancingUpperBodySubControllerState.OR_REC_DEC, desiredWrenchOnPelvis.getExpressedInFrame()).getVector());

      }
      
      public void doTransitionOutOfAction()
      {
         setDesiredWrenchOnPelvisToZero();
         if (USE_INVERSE_DYNAMICS_FOR_LUNGING)
         {
            spineControlModule.setGains(); // may not be needed
         }
      }
   }
   
   
   
   private void setLungeAxisInCouplingRegistryBasedOnIcp()
   {
      //TODO: When walking, the lunge axis should ONLY contain components in directions in which the robot IS allowed to take a corrective step.  For example, when walking on a balance beam, the robot should only lunge about x-axis
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      
      FramePoint2d capturePointInWorldFrame = new FramePoint2d(worldFrame);
      capturePointInWorldFrame.set(couplingRegistry.getCapturePointInFrame(capturePointInWorldFrame.getReferenceFrame()).toFramePoint2d());
      
      FrameVector2d vectorFromDesiredToActualICP = new FrameVector2d(ReferenceFrame.getWorldFrame());
      vectorFromDesiredToActualICP.sub(capturePointInWorldFrame, couplingRegistry.getDesiredCapturePointInFrame(capturePointInWorldFrame.getReferenceFrame()));
      
      setLungeAxisInWorldFrame(-vectorFromDesiredToActualICP.getY(), vectorFromDesiredToActualICP.getX());
//      setLungeAxisInWorldFrame(-capturePointInWorldFrame.getY(), capturePointInWorldFrame.getX());
      
   }
   
   
   private void setLungeAxisInWorldFrame(double newX, double newY)
   {
      FrameVector2d tempLungeAxis = new FrameVector2d(ReferenceFrame.getWorldFrame(), newX, newY);

      if (tempLungeAxis.length() != 0.0)
      {
         tempLungeAxis.normalize();
         if (LATCH_LUNGE_AXIS_TO_PITCH_OR_ROLL)
         {
            tempLungeAxis.set(Math.round(tempLungeAxis.getX()),Math.round(tempLungeAxis.getY()));
         }
      }

      lungeAxisGraphic.set(tempLungeAxis.getX(), tempLungeAxis.getY(), 0.0);
      couplingRegistry.setLungeAxis(tempLungeAxis);
      
      setTotalUpperBodyMomentOfInertiaProjectedAlongLungeAxis();
   }
   
 
   
   private void flipVisualizedLungeAxisGraphic()
   {
      Vector2d temp = couplingRegistry.getLungeAxisInFrame(ReferenceFrame.getWorldFrame()).getVectorCopy();
      temp.negate();
      lungeAxisGraphic.setX(temp.getX()); lungeAxisGraphic.setY(temp.getY());
   }
   

   
   private FramePoint2d getIntersectionOfLungeAxisNormalWithSupportPolygon(ReferenceFrame desiredFrame)
   {
      ReferenceFrame midZFeetUpFrame = referenceFrames.getMidFeetZUpFrame();
      FrameVector2d lungeAxisInMidZFeetUp = couplingRegistry.getLungeAxisInFrame(midZFeetUpFrame);
      
      FramePoint2d desiredCoP = new FramePoint2d(lungeAxisInMidZFeetUp.getReferenceFrame(), lungeAxisInMidZFeetUp.getY(), -lungeAxisInMidZFeetUp.getX());
      
      FrameConvexPolygon2d supportPolygonInMidFeetZUp = couplingRegistry.getBipedSupportPolygons().getSupportPolygonInMidFeetZUp();
      
      FrameLineSegment2d segmentFromPolygonCentroidToDesiredCoP = 
         new FrameLineSegment2d(supportPolygonInMidFeetZUp.getCentroidCopy(), desiredCoP);
      
      FramePoint2d[] intersectionPoints = segmentFromPolygonCentroidToDesiredCoP.intersectionWith(supportPolygonInMidFeetZUp);
      FramePoint2d ret = intersectionPoints[0];
      
      ret.changeFrame(desiredFrame);
      
      // no intersections
      if (ret == null)
      {
         ret = supportPolygonInMidFeetZUp.getCentroidCopy();
      }

      return ret;
   }
   
   private void lungeWithSpineUsingFixedPitchRollTorque()
   {
      Vector2d hipTorque = new Vector2d(desiredWrenchOnPelvis.getAngularPartCopy().getX(), desiredWrenchOnPelvis.getAngularPartCopy().getY());
      
      Vector2d deltaHipTorque = new Vector2d(computeTorqueDueToMassOffset(spineRollIDjoint), computeTorqueDueToMassOffset(spinePitchIDjoint));
      hipTorque.sub(deltaHipTorque);

      spineControlModule.setSpineXYTorque(hipTorque);
   }
   
   private void lungeWithSpineUsingID()
   {
      boolean USE_CONSTANT_WRENCH_AROUND_LUNGEAXIS = true;
      boolean USE_CONSTANT_TORQUE_AROUND_LUNGEAXIS = false;
      boolean USE_CMP_CONTROL = false;

      if (USE_CONSTANT_WRENCH_AROUND_LUNGEAXIS)
      {
         spineControlModule.doSpineControlUsingIDwithPDfeedback();
      }
      
      if (USE_CONSTANT_TORQUE_AROUND_LUNGEAXIS)
      {
         spineControlModule.doConstantTorqueAroundLungeAxis(couplingRegistry.getLungeAxisInFrame(ReferenceFrame.getWorldFrame()), maxHipTorque.getDoubleValue());
         spineControlModule.doSpineControlUsingIDwithPDfeedback();
      }
      else if (USE_CMP_CONTROL)
      {
         FramePoint2d desiredCMP = getLungeAxisPerpUnitVector(referenceFrames.getMidFeetZUpFrame());
         desiredCMP.scale( kCMP.getDoubleValue());
         couplingRegistry.setDesiredCMP(desiredCMP);
         
         spineControlModule.doCMPControl(desiredCMP, couplingRegistry.getLungeAxisInFrame(ReferenceFrame.getWorldFrame()));
      }
   }
   
   private class DoWeNeedToStartLungingCondition implements StateTransitionCondition
   {
      public boolean checkCondition()
      {         
//         FrameVector2d vectorFromActualToDesiredICP = new FrameVector2d(icpActual.getFramePoint2dCopy(), icpDesired.getFramePoint2dCopy());
//         return vectorFromActualToDesiredICP.length() > startLungingICPRadius.getDoubleValue();
         
         return !isCapturePointInsideSupportPolygon() && isElapsedTimeInCurrentStateAboveMinimumThreshold();
      }
   }
   
   private double currentStateElapsedTime()
   {
      double deltaTimeInState = processedSensors.getTime() - timeOfStateStart.get((BalancingUpperBodySubControllerState) stateMachine.getCurrentStateEnum());
      
      return deltaTimeInState;
   }
   
   private boolean isElapsedTimeInCurrentStateAboveMinimumThreshold()
   {
      boolean minimumElapsedTimeInCurrentState = currentStateElapsedTime() > minimumElapsedTimeInState.get((BalancingUpperBodySubControllerState) stateMachine.getCurrentStateEnum());
      return minimumElapsedTimeInCurrentState;
   }

   private class DoWeNeedToDecelerateNow implements StateTransitionCondition
   {
      BalancingUpperBodySubControllerState currentState;
       
      public boolean checkCondition()
      {
    	 currentState = (BalancingUpperBodySubControllerState) stateMachine.getCurrentStateEnum();
    	 
    	 boolean doWeNeedToDecelerateNow;
         
         double predictedChestAngleMagnitudeIfWeDecelerateNow = computeChestAngleMagnitudeAboutLungeAxis() + predictedDistanceToStopChestDuring(currentState);
         boolean doWeNeedToSlowDownBecauseOfAngleLimit = predictedChestAngleMagnitudeIfWeDecelerateNow > maxChestAngleMagnitude.getDoubleValue();  //TODO: FIX THIS
         
         if (doWeNeedToSlowDownBecauseOfAngleLimit)
         {
        	 doWeNeedToDecelerateNow = doWeNeedToSlowDownBecauseOfAngleLimit && isElapsedTimeInCurrentStateAboveMinimumThreshold();
        	 System.out.println("Transition to slow down due to Angle Limit.");
         }
         else
         {
        	 doWeNeedToDecelerateNow = willICPEndUpAtDesiredPositionIfWeDecelerateNow() && isElapsedTimeInCurrentStateAboveMinimumThreshold();
        	 if (doWeNeedToDecelerateNow) System.out.println("Transition to slow down due favorable predicted ICP location after slow down.");
         }

            
         return doWeNeedToDecelerateNow;
      }


      /**
       * Predict where the ICP will be located when the chest comes to rest if we begin to decelerate the chest now.
       * Assume that the CoP, net torque on the chest, and resulting CmP are constant during deceleration.
       * First, predict the time it takes to decelerate the chest based upon the constant torque and a known chest moment of inertia.
       * Using the first-order ICP dynamics, solve for the change in ICP position over this time period, given the anticipated *constant* CmP location.
       * @return
       */
      private boolean willICPEndUpAtDesiredPositionIfWeDecelerateNow()
      {  
         ReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
         
         FramePoint2d icpActual = couplingRegistry.getCapturePointInFrame(midFeetZUpFrame).toFramePoint2d();
         FramePoint2d predictedCMPDuringChestSlowDown = cmpDueToDesiredCoPandDesiredSpineTorque(currentState, midFeetZUpFrame);
         FramePoint2d desiredICPWhenChestStops = icpDesiredLunging.getFramePoint2dCopy().changeFrameCopy(midFeetZUpFrame);
         
         FramePoint2d predictedICPWhenChestStops = new FramePoint2d(midFeetZUpFrame);
         predictedICPWhenChestStops = predictICPWhenChestStopsIfWeSlowDownNow(icpActual, predictedCMPDuringChestSlowDown, predictedTimeToStopChestDuring(currentState));
         predictedICPAtChestStop.set(predictedICPWhenChestStops.changeFrameCopy(ReferenceFrame.getWorldFrame()));

         FrameVector2d vectorFromDesiredToPredictedICP = new FrameVector2d(desiredICPWhenChestStops, predictedICPWhenChestStops);

         boolean hasVectorFromDesiredToPredictedIcpFlippedDirection = vectorFromDesiredToPredictedICP.dot(getUnitVectorNormalToLungeAxis(midFeetZUpFrame)) < stopLungingICPRadius.getDoubleValue();
         
         return hasVectorFromDesiredToPredictedIcpFlippedDirection;
      }
   }
   
   
   private void setChestAngularVelocityMagnitudeAlongLungeAxisOnStateStart(BalancingUpperBodySubControllerState currentState)
   {
      if (currentStateElapsedTime() > 0.0) throw new RuntimeException("Can only set this value at the transition into a state.");

      chestAngularVelMagnitudeAboutLungeAxisOnStateStart.put(currentState, getCurrentChestAngularVelMagnitudeAboutLungeAxis());
   }
   
   private double getCurrentChestAngularVelMagnitudeAboutLungeAxis()
   {
      ReferenceFrame chestFrame = chest.getBodyFixedFrame();
      FrameVector2d chestAngularVelocityInChest = new FrameVector2d(chestFrame, processedSensors.getChestAngularVelocityInChestFrame().getX(), processedSensors.getChestAngularVelocityInChestFrame().getY());
      return  chestAngularVelocityInChest.dot(couplingRegistry.getLungeAxisInFrame(chestAngularVelocityInChest.getReferenceFrame()));
   }
   
   private double getChestAngleToWorld()
   {
      Quat4d chestToWorld = processedSensors.getChestOrientationInFrame(ReferenceFrame.getWorldFrame()).getQuaternion();
      AxisAngle4d chestToWorldAxisAngle = new AxisAngle4d();
      chestToWorldAxisAngle.set(chestToWorld);
   
      double bodyAngleToWorld = chestToWorldAxisAngle.getAngle();
      return bodyAngleToWorld;
   }
   

   private double computeChestAngleMagnitudeAboutLungeAxis()
   {
      ReferenceFrame expressedInFrame = ReferenceFrame.getWorldFrame();
      FrameVector2d lungeAxis = couplingRegistry.getLungeAxisInFrame(expressedInFrame);
      
      double scaledAngle = 0.0;
      if (lungeAxis != null)
      {
         Transform3D chestTransformToWorld = chest.getBodyFixedFrame().getTransformToDesiredFrame(expressedInFrame);
         Quat4d chestToWorldQuat = new Quat4d();
         chestTransformToWorld.get(chestToWorldQuat);
         AxisAngle4d chestToWorldAxisAndAngle = new AxisAngle4d();
         chestToWorldAxisAndAngle.set(chestToWorldQuat);
         
         FrameVector2d axis = new FrameVector2d(expressedInFrame, chestToWorldAxisAndAngle.getX(), chestToWorldAxisAndAngle.getY());
         double angle = chestToWorldAxisAndAngle.angle;
         
         scaledAngle = angle * axis.dot(lungeAxis);
      }

      return scaledAngle;
   }
   
    

   private ArrayList<InverseDynamicsJoint> populateAllIDjointsAbovePelvis()
   {
      this.spineRollIDjoint =  pelvis.getChildrenJoints().get(0);
      this.spineYawIDjoint = this.spineRollIDjoint.getSuccessor().getChildrenJoints().get(0);
      this.spinePitchIDjoint = this.spineYawIDjoint.getSuccessor().getChildrenJoints().get(0);

      checkSpineIDJointNames();
      
      ArrayList<InverseDynamicsJoint> allJoints = new ArrayList<InverseDynamicsJoint>();
      List<InverseDynamicsJoint> jointsToIgnore = new ArrayList<InverseDynamicsJoint>();
      ArrayList<RigidBody> morgue = new ArrayList<RigidBody>();

      morgue.add(pelvis);
      
      InverseDynamicsJoint leftHipPitch = pelvis.getChildrenJoints().get(1);
      InverseDynamicsJoint rightHipPitch = pelvis.getChildrenJoints().get(2);
      
      jointsToIgnore.add(leftHipPitch);
      jointsToIgnore.add(rightHipPitch);
      

      while (!morgue.isEmpty())
      {
         RigidBody currentBody = morgue.get(0);

         if (currentBody.hasChildrenJoints())
         {
            List<InverseDynamicsJoint> childrenJoints = currentBody.getChildrenJoints();
            for (InverseDynamicsJoint joint : childrenJoints)
            {
               if (!jointsToIgnore.contains(joint))
               {
                  RigidBody successor = joint.getSuccessor();
                  if (successor != null)
                  {
                     allJoints.add(joint);
                     morgue.add(successor);
                  }
               }
            }
         }

         morgue.remove(currentBody);
      }
      
      this.allIDJointsAbovePelvis = allJoints;
      
      computeTotalUpperBodyMomentOfInertia();
      
      return allJoints;
   }
   

   private void computeTotalUpperBodyMomentOfInertia()
   {  
      CompositeRigidBodyInertia totalCompositeRigidBodyInertia = new CompositeRigidBodyInertia(pelvis.getBodyFixedFrame(), new Matrix3d(), 0.0);
      
      for(InverseDynamicsJoint childJoint : allIDJointsAbovePelvis)
      {
         RigidBody rigidBody = childJoint.getSuccessor();
         CompositeRigidBodyInertia inertia = new CompositeRigidBodyInertia(rigidBody.getBodyFixedFrame(), rigidBody.getInertia().getMassMomentOfInertiaPartCopy(), rigidBody.getInertia().getMass());
         
         inertia.changeFrame(totalCompositeRigidBodyInertia.getExpressedInFrame());
         totalCompositeRigidBodyInertia.add(inertia);
      }
      
      setTotalUpperBodyCompositeRigidBodyInertia(totalCompositeRigidBodyInertia);
   }

   private CompositeRigidBodyInertia getTotalUpperBodyCompositeRigidBodyInertiaCopy(ReferenceFrame expressedInFrame)
   {
      CompositeRigidBodyInertia MoI = new CompositeRigidBodyInertia();
      MoI.set(this.upperBodyMoI);
      
      MoI.changeFrame(expressedInFrame);
      
      return MoI;
   }

   
   private void setTotalUpperBodyMomentOfInertiaProjectedAlongLungeAxis()
   {
      computeTotalUpperBodyMomentOfInertia();   //TODO: May not be necessary to re-compute, depending on whether the arms affected the total MoI by changing orientation
      
      FrameVector2d lungeAxis = couplingRegistry.getLungeAxisInFrame(pelvisFrame);
      double projectedMOI;
      if (lungeAxis != null)
      {
         double moiFeltBySpinePitch = getTotalUpperBodyCompositeRigidBodyInertiaCopy(spinePitchIDjoint.getFrameAfterJoint()).getMassMomentOfInertiaPartCopy().m11;
         double moiFeltBySpineRoll = getTotalUpperBodyCompositeRigidBodyInertiaCopy(spineRollIDjoint.getFrameBeforeJoint()).getMassMomentOfInertiaPartCopy().m00;
         
         double pitchComponentSquared = lungeAxis.getY()*lungeAxis.getY();
         double rollComponentSquared = lungeAxis.getX()*lungeAxis.getX();
         projectedMOI =  moiFeltBySpinePitch * pitchComponentSquared +  moiFeltBySpineRoll * rollComponentSquared;
      }
      else
      {
         projectedMOI = 0.0;
      }

      System.out.println("projected MOI: " + projectedMOI);
      
      upperBodyMoIProjectedOntoLungeAxis = projectedMOI;
   }
   
   private double predictedChestAccelerationDuring(BalancingUpperBodySubControllerState predictedState)
   {
      if (upperBodyMoIProjectedOntoLungeAxis == 0.0 )
      {
         setTotalUpperBodyMomentOfInertiaProjectedAlongLungeAxis();
      }
      
      double predictedChestAngularDecel = desiredSpineLungingTorqueMagnitudes.get(predictedState) / (upperBodyMoIProjectedOntoLungeAxis);
      
      return predictedChestAngularDecel;
   }
   
   private double computeTorqueDueToMassOffset(InverseDynamicsJoint jointName)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      
      FrameVector forceVectorDueToGravity = new FrameVector(worldFrame, 0.0, 0.0, gravity * upperBodyMoI.getMass());
      FrameVector vectorFromJointOriginToUpperBodyCoM = new FrameVector(worldFrame);
      
      FramePoint jointOrigin = new FramePoint(jointName.getFrameBeforeJoint());
      jointOrigin.changeFrame(worldFrame);
      
      FramePoint upperBodyCoM = upperBodyMoI.getCenterOfMassOffset();
      upperBodyCoM.changeFrame(worldFrame);
      vectorFromJointOriginToUpperBodyCoM.sub(upperBodyCoM, jointOrigin);
      
      FrameVector torqueVectorDueToGravity = new FrameVector(worldFrame);
      torqueVectorDueToGravity.cross(vectorFromJointOriginToUpperBodyCoM, forceVectorDueToGravity);
      
      double torqueAppliedToJoint = torqueVectorDueToGravity.dot(getIDRevoluteJointAxis(jointName, worldFrame));

      return torqueAppliedToJoint;
   }

   private FrameVector getIDRevoluteJointAxis(InverseDynamicsJoint jointName, ReferenceFrame expressedInFrame)
   {
      Matrix jointVelocity = new Matrix(1, 1);
      jointVelocity.set(0, 0, 1.0);
      GeometricJacobian jointJacobian = jointName.getMotionSubspace();
      Twist twistToPack = jointJacobian.getTwist(jointVelocity);
      
      
      Vector3d axis = twistToPack.getAngularPartCopy();
      axis.normalize();
      FrameVector jointAxis = new FrameVector(jointName.getFrameAfterJoint(), axis); //shouldn't matter whether using getFrameBeforeJoint() or getFrameAfterJoint()
      jointAxis.changeFrame(expressedInFrame);
      return jointAxis;
   }

   private double predictedTimeToStopChestDuring(BalancingUpperBodySubControllerState predictedState)
   {
      double predictedTimeToStopChest = Math.abs(chestAngularVelocityMagnitude.getDoubleValue() / predictedChestAccelerationDuring(predictedState));

      predictedTimeToDecelerateChest.set(predictedTimeToStopChest);
      return predictedTimeToStopChest;
   }
      
   private double predictedDistanceToStopChestDuring(BalancingUpperBodySubControllerState state)
   {
      double deltaT = predictedTimeToStopChestDuring(state);
      double predictedStopDistance = chestAngularVelocityMagnitude.getDoubleValue() * deltaT - 0.5 * predictedChestAccelerationDuring(state) * deltaT * deltaT;
      
      return predictedStopDistance;
   }
   
   
   private FramePoint2d cmpDueToDesiredCoPandDesiredSpineTorque(BalancingUpperBodySubControllerState stateToReferenceForDesireds, ReferenceFrame referenceFrame)
   {
      Vector3d desiredSpineTorque = desiredLungingTorqueDuring(stateToReferenceForDesireds, referenceFrame).getVectorCopy();
      
      FramePoint2d deltaCMPDueToSpineTorque = 
         new FramePoint2d(referenceFrame, deltaCMPDueToSpineTorque(desiredSpineTorque).getX(), deltaCMPDueToSpineTorque(desiredSpineTorque).getY()); 

      FramePoint2d predictedCMP = desiredCoPduring(stateToReferenceForDesireds, referenceFrame);
      predictedCMP.add(deltaCMPDueToSpineTorque);
      
      predictedCMP.scale(predictedCMPscaling.getDoubleValue());  //Tune the desired overshoot for which to transition to chest deceleration
      return predictedCMP;
   }
   
   private FramePoint2d predictICPWhenChestStopsIfWeSlowDownNow(FramePoint2d icpCurrent, FramePoint2d cmpDuringChestDecelerate, double elapsedTimeToStop)
   {
      //Predict where the ICP will be at the time the chest comes to rest, if we start to decelerate NOW.
      //Assume a constant CmP location proportional to the maxHipTorque.
      double gravity = Math.abs(this.gravity);
      double CoMHeight = processedSensors.getCenterOfMassPositionInFrame(ReferenceFrame.getWorldFrame()).getZ();
      double omega0 = Math.sqrt( gravity / CoMHeight );
      
      // icp(timeCurrent + elapsedTimeToStop) = cmp(timeCurrent) + [ icp(timeCurrent) - cmp(timeCurrent) ] * exp(omega0 * elapsedTimeToStop)
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
   
   private Vector3d computeSpineTorqueForDesiredDeltaCMP (Vector2d deltaCMP, ReferenceFrame referenceFrame)
   {
      double mass = this.robotMass;
      double gravity = this.gravity;
      
      Vector3d netSpineTorque = new Vector3d();
      netSpineTorque.set(-deltaCMP.getY(), deltaCMP.getX(), 0.0);
      netSpineTorque.scale(mass*gravity);
      return netSpineTorque;
   }

   private class HasChestAngularVelocityCrossedZeroCondition implements StateTransitionCondition
   {

      public HasChestAngularVelocityCrossedZeroCondition()
      {
      }

      public boolean checkCondition()
      {
         ReferenceFrame lungeAxisExpressedInFrame = ReferenceFrame.getWorldFrame();
         FrameVector2d spineXYAngularVelocity = new FrameVector2d(lungeAxisExpressedInFrame);
         spineXYAngularVelocity.set(processedSensors.getSpineJointVelocity(SpineJointName.SPINE_ROLL), processedSensors.getSpineJointVelocity(SpineJointName.SPINE_PITCH));
         
         double angularVelocityProjectedAlongLungeAxis = spineXYAngularVelocity.dot(couplingRegistry.getLungeAxisInFrame(lungeAxisExpressedInFrame));
         
//         System.out.println("Projected Spine Angular Velocity: " + angularVelocityProjectedAlongLungeAxis);
         
         boolean ret = angularVelocityProjectedAlongLungeAxis < 0.0 &&  isElapsedTimeInCurrentStateAboveMinimumThreshold();
         return ret;
      }
   }
   

   private class HasArmAngularVelocityCrossedZeroCondition implements StateTransitionCondition
   {
      
      public HasArmAngularVelocityCrossedZeroCondition()
      {
      }
      
      public boolean checkCondition()
      {
         BalancingUpperBodySubControllerState currentState = (BalancingUpperBodySubControllerState) stateMachine.getCurrentStateEnum();
         double pitchAngularVelocity = 0.0;
         for (RobotSide robotSide : RobotSide.values())
         {
            pitchAngularVelocity =+ processedSensors.getArmJointVelocity(robotSide, ArmJointName.SHOULDER_PITCH);
         }
         
         double LungeAxisPitchComponent = couplingRegistry.getLungeAxisInFrame(ReferenceFrame.getWorldFrame()).getY();
         
         boolean ret;
         
         if (desiredArmLungingTorqueScaling.get(currentState) < 0.0)
         {
            ret = true; // Return true if shoulder torque is in fact increasing the shoulder velocity, (for the case when arm joints swing in opposite direction of hip joint)
         }
         else
         {
            ret = pitchAngularVelocity > LungeAxisPitchComponent;
         }
         if (ret)
         {
            System.out.println("Arm Angular Velocity Zero");
         }
         
         return ret && isElapsedTimeInCurrentStateAboveMinimumThreshold(); 
      }  
   }
   
   
   private class DecelerateNowToStopUpperBodyAtUprightCondition implements StateTransitionCondition
   {
      public boolean checkCondition()
      {
//         double angularDisplacementAboutLungeAxis = processedSensors.getSpineJointPosition(SpineJointName.SPINE_PITCH);         //TODO: This only works about the pitch axis!
         double angularDisplacmentAboutLungeAxis = computeChestAngleMagnitudeAboutLungeAxis();
         return angularDisplacmentAboutLungeAxis - predictedDistanceToStopChestDuring(BalancingUpperBodySubControllerState.OR_REC_ACC) <= 0.0
         && currentStateElapsedTime() > minimumElapsedTimeInState.get((BalancingUpperBodySubControllerState) stateMachine.getCurrentStateEnum());
      }
   }
   
   private class isChestFrameAlignedWithGravityCondition implements StateTransitionCondition
   {
      public boolean checkCondition()
      {
         return chestAngle.getDoubleValue() < 1e-1 && isElapsedTimeInCurrentStateAboveMinimumThreshold();
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

   private void populateDynamicsGraphicObjects(DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      DynamicGraphicVector lungeAxisVisual = new DynamicGraphicVector("lungeAxisVisual", robotCoMPosition, lungeAxisGraphic, 1.0, YoAppearance.DarkRed());
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject(name, lungeAxisVisual);
   
      DynamicGraphicPosition icpDesiredVisual = new DynamicGraphicPosition("icpDesiredVisual", icpDesiredLunging, 0.05, YoAppearance.Yellow());
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject(name, icpDesiredVisual);
      
      DynamicGraphicPosition icpActualVisual = new DynamicGraphicPosition("icpActualVisual", icpActual, 0.05, YoAppearance.Blue());
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject(name, icpActualVisual);
      
      DynamicGraphicPosition icpPredictedVisual = new DynamicGraphicPosition("icpPredictedVisual", predictedICPAtChestStop, 0.05, YoAppearance.Purple());
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject(name, icpPredictedVisual);
      
      DynamicGraphicPosition lungeAxisPerpIntersectionWithBoSVisual = new DynamicGraphicPosition("lungeAxisPerpIntersectionWithBoSVisual", lungeAxisPerpIntersectionWithBoS, 0.02, YoAppearance.Pink());
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject(name, lungeAxisPerpIntersectionWithBoSVisual);
   
      DynamicGraphicVector chestAxisAngleVisual = new DynamicGraphicVector("chestAxisVisual", chestCoMPosition, lungeAxisGraphic, 2.0, YoAppearance.DarkRed());
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject(name, chestAxisAngleVisual);

   }


   private void updateVariables()
   {
      chestAngle.set(getChestAngleToWorld());
      
      FrameVector chestAngularVelocityVector = processedSensors.getChestAngularVelocityInChestFrame();
      chestAngularVelocityVector.changeFrame(ReferenceFrame.getWorldFrame());
      chestAngularVelocityMagnitude.set(chestAngularVelocityVector.length());
      
      FramePoint2d icpDesiredFromCouplingRegistry = couplingRegistry.getDesiredCapturePointInFrame(ReferenceFrame.getWorldFrame());
      setIcpDesired(icpDesiredFromCouplingRegistry.getX(), icpDesiredFromCouplingRegistry.getY());
      
      FramePoint icpActualFromCouplingRegistry = couplingRegistry.getCapturePointInFrame(ReferenceFrame.getWorldFrame());
      icpActual.set(icpActualFromCouplingRegistry.getX(), icpActualFromCouplingRegistry.getY());    
   }


   private void updateDynamicsGraphicObjects()
   {
      robotCoMPosition.set(processedSensors.getCenterOfMassPositionInFrame(ReferenceFrame.getWorldFrame()));
      
      FramePoint chestCoM = chestCoMPosition.getFramePointCopy();
      this.chest.packCoMOffset(chestCoM);
      chestCoM.changeFrame(chestCoMPosition.getReferenceFrame());
      chestCoMPosition.set(chestCoM);
      
      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
      if (couplingRegistry.getLungeAxisInFrame(referenceFrame) != null) lungeAxisPerpIntersectionWithBoS.set(getIntersectionOfLungeAxisNormalWithSupportPolygon(referenceFrame));
      
   }


   private void setParameters()
   {
      maxChestAngleMagnitude.set(Math.PI);
      startLungingICPRadius.set(0.25);
      stopLungingICPRadius.set(0.0);
      kTimeToSlowSlowDown.set(1.4);
      

      forceControllerIntoState.set(false);
      forcedControllerState.set(BalancingUpperBodySubControllerState.BASE);


      setLungeAxisInWorldFrame(0.0, 0.0);
      
      
      if (USE_ARMS_FOR_LUNGING)
      {
//         setShoulderLungingParameters0();
         setShoulderLungingParameters1();  // positive arm torque
//         setShoulderLungingParameters2();
//         setShoulderLungingParameters3();  // negative arm torque
      
      }
      else
      {
         predictedCMPscaling.set(0.05);
         
         desiredSpineLungingTorqueMagnitudes.put(BalancingUpperBodySubControllerState.BASE, 0.0);
         desiredSpineLungingTorqueMagnitudes.put(BalancingUpperBodySubControllerState.ICP_REC_ACC, 1.0*maxHipTorque.getDoubleValue());
         desiredSpineLungingTorqueMagnitudes.put(BalancingUpperBodySubControllerState.ICP_REC_DEC, -1.5*maxHipTorque.getDoubleValue());
         desiredSpineLungingTorqueMagnitudes.put(BalancingUpperBodySubControllerState.OR_REC_ACC, -0.8*maxHipTorque.getDoubleValue());
         desiredSpineLungingTorqueMagnitudes.put(BalancingUpperBodySubControllerState.OR_REC_DEC, 0.85*maxHipTorque.getDoubleValue());
      }
      
      // Use default values for now
      icpDesiredLunging.set(new FramePoint2d(referenceFrames.getMidFeetZUpFrame()).changeFrameCopy(ReferenceFrame.getWorldFrame()));
      bosRadiusPerpToLungeAxis.set(0.0);
      
      // Proportional feedback gain to track desired CMP (used when USE_CMP_CONTROL = true)
      kCMP.set(0.25);
   }
      
   private void setShoulderLungingParameters0()
   {
      predictedCMPscaling.set(0.75);

      desiredSpineLungingTorqueMagnitudes.put(BalancingUpperBodySubControllerState.BASE, 0.0);
      desiredSpineLungingTorqueMagnitudes.put(BalancingUpperBodySubControllerState.ICP_REC_ACC, 1.0*maxHipTorque.getDoubleValue());
      desiredSpineLungingTorqueMagnitudes.put(BalancingUpperBodySubControllerState.ICP_REC_DEC, -1.5*maxHipTorque.getDoubleValue());
      desiredSpineLungingTorqueMagnitudes.put(BalancingUpperBodySubControllerState.ICP_REC_DEC2, -1.5*maxHipTorque.getDoubleValue());
      desiredSpineLungingTorqueMagnitudes.put(BalancingUpperBodySubControllerState.OR_REC_ACC, -0.5*maxHipTorque.getDoubleValue());
      desiredSpineLungingTorqueMagnitudes.put(BalancingUpperBodySubControllerState.OR_REC_DEC, 0.85*maxHipTorque.getDoubleValue());

      //////////////////////    LUNGE WITH SHOULDER TORQUE   /////////////////////////////////////////////
      desiredArmLungingTorqueScaling.put(BalancingUpperBodySubControllerState.BASE, 0.0);  //Not Used
      desiredArmLungingTorqueScaling.put(BalancingUpperBodySubControllerState.ICP_REC_ACC, 0.5);  //[0.5] or [0.2]
      desiredArmLungingTorqueScaling.put(BalancingUpperBodySubControllerState.ICP_REC_DEC, 0.1); //[-0.1]
      desiredArmLungingTorqueScaling.put(BalancingUpperBodySubControllerState.ICP_REC_DEC2, desiredArmLungingTorqueScaling.get(BalancingUpperBodySubControllerState.ICP_REC_DEC)); //[0.1]
   }
      
   private void setShoulderLungingParameters1()
   //Positive hip and shoulder torque   
   {
      predictedCMPscaling.set(1.00);
      
      desiredSpineLungingTorqueMagnitudes.put(BalancingUpperBodySubControllerState.BASE, 0.0);
      desiredSpineLungingTorqueMagnitudes.put(BalancingUpperBodySubControllerState.ICP_REC_ACC, 1.0*maxHipTorque.getDoubleValue());
      desiredSpineLungingTorqueMagnitudes.put(BalancingUpperBodySubControllerState.ICP_REC_DEC, -1.5*maxHipTorque.getDoubleValue());
      desiredSpineLungingTorqueMagnitudes.put(BalancingUpperBodySubControllerState.ICP_REC_DEC2, -1.5*maxHipTorque.getDoubleValue());
      desiredSpineLungingTorqueMagnitudes.put(BalancingUpperBodySubControllerState.OR_REC_ACC, -0.5*maxHipTorque.getDoubleValue());
      desiredSpineLungingTorqueMagnitudes.put(BalancingUpperBodySubControllerState.OR_REC_DEC, 0.85*maxHipTorque.getDoubleValue());

      //////////////////////    LUNGE WITH SHOULDER TORQUE   /////////////////////////////////////////////
      desiredArmLungingTorqueScaling.put(BalancingUpperBodySubControllerState.BASE, 0.0);  //Not Used
      desiredArmLungingTorqueScaling.put(BalancingUpperBodySubControllerState.ICP_REC_ACC, 1.35);  //[0.5] or [0.2]
      desiredArmLungingTorqueScaling.put(BalancingUpperBodySubControllerState.ICP_REC_DEC, 0.25*0.1); //[-0.1]
      desiredArmLungingTorqueScaling.put(BalancingUpperBodySubControllerState.ICP_REC_DEC2, desiredArmLungingTorqueScaling.get(BalancingUpperBodySubControllerState.ICP_REC_DEC)); //[0.1]
   }
   
   private void setShoulderLungingParameters2()
   //Positive hip torque negative shoulder torque
   {
      predictedCMPscaling.set(1.00);
      
      desiredSpineLungingTorqueMagnitudes.put(BalancingUpperBodySubControllerState.BASE, 0.0);
      desiredSpineLungingTorqueMagnitudes.put(BalancingUpperBodySubControllerState.ICP_REC_ACC, 1.0*maxHipTorque.getDoubleValue());
      desiredSpineLungingTorqueMagnitudes.put(BalancingUpperBodySubControllerState.ICP_REC_DEC, -1.5*maxHipTorque.getDoubleValue());
      desiredSpineLungingTorqueMagnitudes.put(BalancingUpperBodySubControllerState.ICP_REC_DEC2, -1.5*maxHipTorque.getDoubleValue());
      desiredSpineLungingTorqueMagnitudes.put(BalancingUpperBodySubControllerState.OR_REC_ACC, -0.5*maxHipTorque.getDoubleValue());
      desiredSpineLungingTorqueMagnitudes.put(BalancingUpperBodySubControllerState.OR_REC_DEC, 0.85*maxHipTorque.getDoubleValue());

      //////////////////////    LUNGE WITH SHOULDER TORQUE   /////////////////////////////////////////////
      desiredArmLungingTorqueScaling.put(BalancingUpperBodySubControllerState.BASE, 0.0);  //Not Used
      desiredArmLungingTorqueScaling.put(BalancingUpperBodySubControllerState.ICP_REC_ACC, 1.35);
      desiredArmLungingTorqueScaling.put(BalancingUpperBodySubControllerState.ICP_REC_DEC, 0.1); 
      desiredArmLungingTorqueScaling.put(BalancingUpperBodySubControllerState.ICP_REC_DEC2, desiredArmLungingTorqueScaling.get(BalancingUpperBodySubControllerState.ICP_REC_DEC)); //[0.1]
   }
   
   private void setShoulderLungingParameters3()
   {
      predictedCMPscaling.set(0.05);

      desiredSpineLungingTorqueMagnitudes.put(BalancingUpperBodySubControllerState.BASE, 0.0);
      desiredSpineLungingTorqueMagnitudes.put(BalancingUpperBodySubControllerState.ICP_REC_ACC, 1.0*maxHipTorque.getDoubleValue());
      desiredSpineLungingTorqueMagnitudes.put(BalancingUpperBodySubControllerState.ICP_REC_DEC, -1.5*maxHipTorque.getDoubleValue());
      desiredSpineLungingTorqueMagnitudes.put(BalancingUpperBodySubControllerState.ICP_REC_DEC2, -1.5*maxHipTorque.getDoubleValue());
      desiredSpineLungingTorqueMagnitudes.put(BalancingUpperBodySubControllerState.OR_REC_ACC, -0.5*maxHipTorque.getDoubleValue());
      desiredSpineLungingTorqueMagnitudes.put(BalancingUpperBodySubControllerState.OR_REC_DEC, 0.85*maxHipTorque.getDoubleValue());

      //////////////////////    LUNGE WITH SHOULDER TORQUE   /////////////////////////////////////////////
      desiredArmLungingTorqueScaling.put(BalancingUpperBodySubControllerState.BASE, 0.0);  //Not Used
      desiredArmLungingTorqueScaling.put(BalancingUpperBodySubControllerState.ICP_REC_ACC, -0.1);
      desiredArmLungingTorqueScaling.put(BalancingUpperBodySubControllerState.ICP_REC_DEC, -0.1);
      desiredArmLungingTorqueScaling.put(BalancingUpperBodySubControllerState.ICP_REC_DEC2, desiredArmLungingTorqueScaling.get(BalancingUpperBodySubControllerState.ICP_REC_DEC)); //[0.1]
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


   private void setIcpDesired(double newX, double newY)
   {
      icpDesiredLunging.set(newX, newY);
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

   private FrameVector2d getUnitVectorNormalToLungeAxis(ReferenceFrame desiredFrame)
   {
      // Lunge axis and ICP direction are just orthogonal in the x-y plane
      FrameVector2d lungeAxis = couplingRegistry.getLungeAxisInFrame(ReferenceFrame.getWorldFrame());
      FrameVector2d icpDirection = new FrameVector2d(lungeAxis.getReferenceFrame(), lungeAxis.getY(), -lungeAxis.getX());
      icpDirection.changeFrame(desiredFrame);
      icpDirection.normalize();
      return icpDirection;
   }
 

   private FramePoint2d getLungeAxisPerpUnitVector(ReferenceFrame expressedInFrame)
   {
      FrameVector2d lungeAxis = couplingRegistry.getLungeAxisInFrame(ReferenceFrame.getWorldFrame());
      FramePoint2d lungeAxisPerp = new FramePoint2d(lungeAxis.getReferenceFrame(), lungeAxis.getY(), -lungeAxis.getX());  // Lunge axis and ICP direction are just orthogonal in the x-y plane
      lungeAxisPerp.changeFrame(expressedInFrame);
      return lungeAxisPerp;
   }


   private FrameVector desiredLungingTorqueDuring(BalancingUpperBodySubControllerState state, ReferenceFrame referenceFrame)
   {
      FrameVector lungingTorque  = new FrameVector(referenceFrame);
      FrameVector2d lungeAxis = couplingRegistry.getLungeAxisInFrame(referenceFrame);
      if (lungeAxis != null)
      {
         lungingTorque.set(lungeAxis.getX(), lungeAxis.getY(), 0.0);
      }

      lungingTorque.scale( desiredSpineLungingTorqueMagnitudes.get(state) );
      
      if (state.equals(BalancingUpperBodySubControllerState.OR_REC_ACC))
      {
         double minSpinePitchTorque = computeTorqueDueToMassOffset(spinePitchIDjoint);
         double minSpineRollTorque = computeTorqueDueToMassOffset(spinePitchIDjoint);
      }


      return lungingTorque;
   }
   
   private void setDesiredWrenchOnPelvisAccordingToCurrentState(BalancingUpperBodySubControllerState currentState)
   {
      desiredWrenchOnPelvis.setAngularPart(desiredLungingTorqueDuring(currentState, desiredWrenchOnPelvis.getExpressedInFrame()).getVectorCopy());
      spineControlModule.setWrench(desiredWrenchOnPelvis);
   }
   
   private void setDesiredWrenchOnPelvisToZero()
   {
      Vector3d zeroVector = new Vector3d();
      desiredWrenchOnPelvis.setAngularPart(zeroVector);
      desiredWrenchOnPelvis.setLinearPart(zeroVector);
      
      spineControlModule.setWrench(desiredWrenchOnPelvis);
   }
   
   
   private FramePoint2d desiredCoPduring(BalancingUpperBodySubControllerState state, ReferenceFrame referenceFrame)
   {
      FramePoint2d desiredCoP = new FramePoint2d(referenceFrame);
      FrameVector2d lungeAxis = couplingRegistry.getLungeAxisInFrame(ReferenceFrame.getWorldFrame());

      FrameVector2d vectorFromMidFeetToCoP = new FrameVector2d(lungeAxis.getReferenceFrame(), lungeAxis.getX(), lungeAxis.getY());
      vectorFromMidFeetToCoP.normalize();
      vectorFromMidFeetToCoP.scale(bosRadiusPerpToLungeAxis.getDoubleValue());
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
      case OR_REC_ACC:
          desiredCoP.set(getIntersectionOfLungeAxisNormalWithSupportPolygon(referenceFrame));
         break;
      case OR_REC_DEC:
         desiredCoP.set(vectorFromMidFeetToCoP);

      default:
         break;
      }

      return desiredCoP;
   }
   
   private void setArmTorquesDirectlyForLunging(UpperBodyTorques upperBodyTorquesToPack)
   {
      BalancingUpperBodySubControllerState currentState = (BalancingUpperBodySubControllerState) stateMachine.getCurrentStateEnum();
      
      armControlModule.doArmControl(upperBodyTorquesToPack.getArmTorques());
      ArmTorques[] armTorques = upperBodyTorquesToPack.getArmTorques();
      
      if (stateMachine.isCurrentState(BalancingUpperBodySubControllerState.BASE))
      {
         for (RobotSide robotSide : RobotSide.values())
         {
            for (ArmJointName armJointName : ArmJointName.values())
            {
               // Scale PD torques in order to minimize arm acceleration when returning back to base state
               armTorques[robotSide.ordinal()].setTorque(armJointName, 0.5*armTorques[robotSide.ordinal()].getTorque(armJointName));  //[0.2]
            }
         }
      }
      else
      {
         for (RobotSide robotSide : RobotSide.values())
         {
            armTorques[robotSide.ordinal()].setTorque(ArmJointName.SHOULDER_PITCH, desiredArmLungingTorqueScaling.get(currentState)*desiredLungingTorqueDuring(currentState,chest.getBodyFixedFrame()).getY());
         }
      }
   }
   
   private void setActualWrenchExertedOnPelvis(UpperBodyTorques upperBodyTorques)
   {
      ReferenceFrame expressedInFrame = ReferenceFrame.getWorldFrame();
      
      Wrench totalUpperBodyWrench = new Wrench(pelvisFrame, pelvisFrame);
      totalUpperBodyWrench.setAngularPart(this.desiredWrenchOnPelvis.getAngularPartCopy());
      double upperBodyTotalMass = this.upperBodyMoI.getMass();
      FramePoint lungeJointOrigin = new FramePoint(spinePitchIDjoint.getFrameBeforeJoint());
      FramePoint upperBodyCoM = this.upperBodyMoI.getCenterOfMassOffset();
      upperBodyCoM.changeFrame(expressedInFrame);
      
      FrameVector spinePitchAngularAcceleration = computeAngularAccelerationOfUpperBodyRelativeToPelvis(expressedInFrame);
      
      FrameVector comAcceleration = new FrameVector(expressedInFrame);
      comAcceleration.cross(upperBodyCoM, spinePitchAngularAcceleration);

      comAcceleration.scale(upperBodyTotalMass);
      comAcceleration.changeFrame(totalUpperBodyWrench.getExpressedInFrame());
      
      totalUpperBodyWrench.setLinearPart(comAcceleration.getVectorCopy());

      couplingRegistry.setActualUpperBodyLungingWrench(totalUpperBodyWrench);
   }

	private FrameVector computeAngularAccelerationOfUpperBodyRelativeToPelvis(ReferenceFrame expressedInFrame) 
	{
		//FIXME: This only computes pitch axis angular acceleration
		FrameVector spinePitchAngularAcceleration = new FrameVector(expressedInFrame);
	    SpatialAccelerationVector spinePitchAccelToPack = new SpatialAccelerationVector();
	    spinePitchIDjoint.packJointAcceleration(spinePitchAccelToPack);
	    spinePitchAngularAcceleration.set(spinePitchAccelToPack.getExpressedInFrame(), spinePitchAccelToPack.getAngularPartCopy());
	    spinePitchAngularAcceleration.changeFrame(expressedInFrame);
		return spinePitchAngularAcceleration;
	}

   private FrameVector computeAngularAccelerationAcrossSpineJoints()
   {
//      spinePitchIDjoint.packJointAcceleration(totalPelvisAccelRelativeToChest);
      FrameVector totalAngularAccelerationAcrossJoints = new FrameVector(pelvisFrame);
      
      SpatialAccelerationVector spinePitchAccelToPack = new SpatialAccelerationVector();
      spinePitchIDjoint.packJointAcceleration(spinePitchAccelToPack);
      FrameVector spinePitchAngularAccel = new FrameVector(spinePitchAccelToPack.getExpressedInFrame(), spinePitchAccelToPack.getAngularPartCopy());
      spinePitchAngularAccel.changeFrame(pelvisFrame);
      
      SpatialAccelerationVector spineYawAccelToPack = new SpatialAccelerationVector();
      spineYawIDjoint.packJointAcceleration(spineYawAccelToPack);
      FrameVector spineYawAngularAccel = new FrameVector(spineYawAccelToPack.getExpressedInFrame(), spineYawAccelToPack.getAngularPartCopy()); 
      spineYawAngularAccel.changeFrame(pelvisFrame);
      
      SpatialAccelerationVector spineRollAccelToPack = new SpatialAccelerationVector();
      spineRollIDjoint.packJointAcceleration(spineRollAccelToPack);
      FrameVector spineRollAngularAccel = new FrameVector(spineRollAccelToPack.getExpressedInFrame(), spineRollAccelToPack.getAngularPartCopy());
      spineRollAngularAccel.changeFrame(pelvisFrame);
      
      totalAngularAccelerationAcrossJoints.add(spinePitchAngularAccel);
      totalAngularAccelerationAcrossJoints.add(spineYawAngularAccel);
      totalAngularAccelerationAcrossJoints.add(spineRollAngularAccel);

      return totalAngularAccelerationAcrossJoints;
   }
   
   
   private void setTotalUpperBodyCompositeRigidBodyInertia(CompositeRigidBodyInertia totalCompositeRigidBodyInertia)
   {
      upperBodyMoI.set(totalCompositeRigidBodyInertia);
   }
   
   private void checkSpineIDJointNames()
   {
      if (!spinePitchIDjoint.getName().equalsIgnoreCase("spinePitch"))
      {
         throw new RuntimeException("Name of Inverse dynamics joint, " + spinePitchIDjoint.getName() + ", does not match expected name.");
      }
      if (!spineRollIDjoint.getName().equalsIgnoreCase("spineRoll"))
      {
         throw new RuntimeException("Name of Inverse dynamics joint, " + spineRollIDjoint.getName() + ", does not match expected name.");
      }
      if (!spineYawIDjoint.getName().equalsIgnoreCase("spineYaw"))
      {
         throw new RuntimeException("Name of Inverse dynamics joint, " + spineYawIDjoint.getName() + ", does not match expected name.");
      }
   }
   
   private void displayWarningsForBooleans()
   {
      if (forceControllerIntoState.getBooleanValue())
      {
         System.out.println("Warning! Controller " + this.name + " is forced to remain in the " + forcedControllerState.toString() + " state!");
      }
   }


}
