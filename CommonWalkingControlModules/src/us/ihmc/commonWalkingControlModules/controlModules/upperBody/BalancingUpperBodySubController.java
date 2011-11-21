package us.ihmc.commonWalkingControlModules.controlModules.upperBody;

import java.util.EnumMap;

import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.ArmControlModule;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.SpineLungingControlModule;
import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.UpperBodySubController;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.NeckJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.NeckTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.UpperBodyTorques;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.utilities.containers.ContainerTools;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.PIDController;
import com.yobotics.simulationconstructionset.util.statemachines.State;
import com.yobotics.simulationconstructionset.util.statemachines.StateMachine;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransition;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransitionCondition;

public class BalancingUpperBodySubController implements UpperBodySubController
{
   private final CouplingRegistry couplingRegistry;
   private final ProcessedSensorsInterface processedSensors;
   private final YoVariableRegistry registry = new YoVariableRegistry("BalancingUpperBodySubController");

   private final ArmControlModule armControlModule;
   private final SpineLungingControlModule spineControlModule;

   private final EnumMap<NeckJointName, PIDController> neckControllers = ContainerTools.createEnumMap(NeckJointName.class);
   private final EnumMap<NeckJointName, DoubleYoVariable> desiredNeckPositions = ContainerTools.createEnumMap(NeckJointName.class);

   private final double controlDT;
   private NeckTorques neckTorques = new NeckTorques();

   private final StateMachine stateMachine;
   private final String name = "BalancingUpperBodySubController";

   public BalancingUpperBodySubController(CouplingRegistry couplingRegistry, ProcessedSensorsInterface processedSensors, double controlDT,
         ArmControlModule armControlModule, SpineLungingControlModule spineControlModule, YoVariableRegistry parentRegistry)
   {
      this.couplingRegistry = couplingRegistry;
      this.processedSensors = processedSensors;
      this.controlDT = controlDT;
      this.armControlModule = armControlModule;
      this.spineControlModule = spineControlModule;
      this.stateMachine = new StateMachine(name + "State", name + "SwitchTime", BalancingUpperBodySubControllerState.class, processedSensors.getYoTime(),
            registry);

      populateYoVariables();

      populateControllers();
      parentRegistry.addChild(registry);

      setGains();

      setUpStateMachine();
   }

   public void doUpperBodyControl(UpperBodyTorques upperBodyTorquesToPack)
   {
      stateMachine.doAction();
      stateMachine.checkTransitionConditions();
      
      armControlModule.doArmControl(upperBodyTorquesToPack.getArmTorques());
      this.doNeckControl();
      
      // set torques
      upperBodyTorquesToPack.setNeckTorques(neckTorques);
      spineControlModule.getSpineTorques(upperBodyTorquesToPack.getSpineTorques());
   }

   private void setUpStateMachine()
   {
      State base = new BaseState();
      State icpRecoverAccelerateState = new ICPRecoverAccelerateState();
      State icpRecoverDecelerateState = new ICPRecoverDecelerateState();

      StateTransitionCondition isICPOutsideLungeRadius = new IsICPOutsideLungeRadiusCondition();
      StateTransitionCondition doWeNeedToDecelerate = new DoWeNeedToSlowDownDecelerate();
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
      
      stateMachine.setCurrentState(base.getStateEnum());
   }

   private enum BalancingUpperBodySubControllerState
   {
      BASE, ICP_REC_ACC, ICP_REC_DEC;
   }

   private class BaseState extends NoTransitionActionsState
   {
      public BaseState()
      {
         super(BalancingUpperBodySubControllerState.BASE);
      }

      @Override
      public void doAction()
      {
         spineControlModule.doMaintainDesiredChestOrientation();
      }
   }

   private class ICPRecoverAccelerateState extends NoTransitionActionsState
   {
      public ICPRecoverAccelerateState()
      {
         super(BalancingUpperBodySubControllerState.ICP_REC_ACC);
      }

      @Override
      public void doAction()
      {
         // TODO Auto-generated method stub
      }
   }

   private class ICPRecoverDecelerateState extends NoTransitionActionsState
   {
      public ICPRecoverDecelerateState()
      {
         super(BalancingUpperBodySubControllerState.ICP_REC_DEC);
      }

      @Override
      public void doAction()
      {
         // TODO Auto-generated method stub
      }
   }

   private class IsICPOutsideLungeRadiusCondition implements StateTransitionCondition
   {
      public boolean checkCondition()
      {
         //TODO implement
         return false;
      }
   }

   private class DoWeNeedToSlowDownDecelerate implements StateTransitionCondition
   {
      private double angularVelocity;
      private double angularAcceleration;

      public boolean checkCondition()
      {
         updateAngularVelocityAndAcceleration();
         boolean doWeNeedToSlowDownBecauseOfAngleLimit = doWeNeedToSlowDownBecauseOfAngleLimit();
         boolean willICPEndUpInsideStopLungingRadius = willICPEndUpFarEnoughBack();

         return doWeNeedToSlowDownBecauseOfAngleLimit || willICPEndUpInsideStopLungingRadius;
      }

      private void updateAngularVelocityAndAcceleration()
      {
         //TODO implement
      }

      private boolean doWeNeedToSlowDownBecauseOfAngleLimit()
      {
         //TODO implement
         return false;
      }

      private boolean willICPEndUpFarEnoughBack()
      {
         //TODO implement
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

   public abstract class NoTransitionActionsState extends State
   {
      public NoTransitionActionsState(Enum<?> stateEnum)
      {
         super(stateEnum);
      }

      @Override
      public void doTransitionIntoAction()
      {
      }

      @Override
      public void doTransitionOutOfAction()
      {
      }
   }

}
