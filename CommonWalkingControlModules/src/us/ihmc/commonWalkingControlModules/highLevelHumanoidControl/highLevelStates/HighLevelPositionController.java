package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;
import java.util.LinkedHashMap;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumControlModuleBridge.MomentumControlModuleType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.robotics.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.SpatialMotionVector;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.yoUtilities.controllers.GainCalculator;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.math.frames.YoFrameOrientation;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;


/**
 * Similar control to {@link CarIngressEgressController} but without handling external wrenches (no ground contact) nor trying to control any part of the robot with respect to world frame.
 * This is mainly to test our control frame while having the robot hanging in the air.
 * @author Sylvain
 *
 */
public class HighLevelPositionController extends HighLevelBehavior
{
   private final static HighLevelState controllerState = HighLevelState.TASKSPACE_POSITION_CONTROL;
   private final static MomentumControlModuleType MOMENTUM_CONTROL_MODULE_TO_USE = MomentumControlModuleType.OPT_NULLSPACE;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   private final double controlDT;
   
   private final TwistCalculator twistCalculator;
   private final FullRobotModel fullRobotModel;
   private final MomentumBasedController momentumBasedController;
   
   private final RigidBody baseForEveryControl;
   private final RigidBody pelvis;
   private final RigidBody chest;
   private final RigidBody head;
   private final SideDependentList<RigidBody> feet = new SideDependentList<>();
   private final SideDependentList<RigidBody> hands = new SideDependentList<>();
   
   private final ArrayList<RigidBody> endEffectors = new ArrayList<>();
   private final LinkedHashMap<RigidBody, Integer> jacobianIdList = new LinkedHashMap<>();
   private final LinkedHashMap<RigidBody, RigidBodySpatialAccelerationControlModule> spatialAccelerationControlModules = new LinkedHashMap<>();
   private final LinkedHashMap<RigidBody, TaskspaceConstraintData> taskspaceConstraintDataMap = new LinkedHashMap<>();
   
   private final LinkedHashMap<RigidBody, YoFramePoint> yoDesiredPositions = new LinkedHashMap<>();
   private final LinkedHashMap<RigidBody, YoFrameOrientation> yoDesiredOrientations = new LinkedHashMap<>();

   private final LinkedHashMap<RigidBody, DoubleYoVariable> yoPositionGains = new LinkedHashMap<>();
   private final LinkedHashMap<RigidBody, DoubleYoVariable> yoOrientationGains = new LinkedHashMap<>();
   private final LinkedHashMap<RigidBody, DoubleYoVariable> yoZetas = new LinkedHashMap<>();
   
   private final LinkedHashMap<RigidBody, DenseMatrix64F> selectionMatrices = new LinkedHashMap<>();

   private final DoubleYoVariable allControllerGains = new DoubleYoVariable("highLevelPControl_allControllerGains", registry);
   private final DoubleYoVariable allControllerZetas = new DoubleYoVariable("highLevelPControl_allControllerZetas", registry);
   private final DoubleYoVariable gainScaling = new DoubleYoVariable("highLevelPControl_gainScaling", registry);
   private final LinkedHashMap<RigidBody, DoubleYoVariable> individualControllerGainScalingMap = new LinkedHashMap<>();

   public HighLevelPositionController(MomentumBasedController momentumBasedController)
   {
      super(controllerState);
   
      this.controlDT = momentumBasedController.getControlDT();
      this.twistCalculator = momentumBasedController.getTwistCalculator();
      this.fullRobotModel = momentumBasedController.getFullRobotModel();
      this.momentumBasedController = momentumBasedController;
      
      this.pelvis = fullRobotModel.getPelvis();
      this.chest = fullRobotModel.getChest();
      this.head = fullRobotModel.getHead();
      for (RobotSide robotSide : RobotSide.values)
      {
         this.feet.put(robotSide, fullRobotModel.getFoot(robotSide));
         this.hands.put(robotSide, fullRobotModel.getHand(robotSide));
      }
      this.baseForEveryControl = pelvis;
      
      setupJacobians();
      setupSpatialAccelerationControlModules();
      setupDesireds();
      setupGains();
      setupSelectionMatrices();
      
      initializeGains();
      setupGainChangedListeners();
   }

   private void setupJacobians()
   {
      int jacobianId;
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody foot = feet.get(robotSide);
         endEffectors.add(foot);
         jacobianId = momentumBasedController.getOrCreateGeometricJacobian(pelvis, foot, foot.getBodyFixedFrame());
         jacobianIdList.put(foot, jacobianId);
         
         RigidBody hand = hands.get(robotSide);
         endEffectors.add(hand);
         jacobianId = momentumBasedController.getOrCreateGeometricJacobian(chest, hand, hand.getBodyFixedFrame());
         jacobianIdList.put(hand, jacobianId);
      }
      
      endEffectors.add(chest);
      jacobianId = momentumBasedController.getOrCreateGeometricJacobian(pelvis, chest, chest.getBodyFixedFrame());
      jacobianIdList.put(chest, jacobianId);
      
      endEffectors.add(head);
      jacobianId = momentumBasedController.getOrCreateGeometricJacobian(chest, head, head.getBodyFixedFrame());
      jacobianIdList.put(head, jacobianId);
   }
   
   private void setupSpatialAccelerationControlModules()
   {
      for (int i = 0; i < endEffectors.size(); i++)
      {
         RigidBody endEffector = endEffectors.get(i);
         String name = endEffector.getName();
         ReferenceFrame endEffectorControlFrame = endEffector.getBodyFixedFrame();

         RigidBodySpatialAccelerationControlModule spatialAccelerationControlModule = new RigidBodySpatialAccelerationControlModule(name, twistCalculator,
               endEffector, endEffectorControlFrame, controlDT, registry);
         spatialAccelerationControlModules.put(endEffector, spatialAccelerationControlModule);
         
         taskspaceConstraintDataMap.put(endEffector, new TaskspaceConstraintData());
      }
   }
   
   private void setupDesireds()
   {
      for (int i = 0; i < endEffectors.size(); i++)
      {
         RigidBody endEffector = endEffectors.get(i);
         
         String name = endEffector.getName() + "Desired";
         ReferenceFrame baseFrame = baseForEveryControl.getBodyFixedFrame();
         YoFramePoint desiredPosition = new YoFramePoint(name, baseFrame, registry);
         yoDesiredPositions.put(endEffector, desiredPosition);
         
         YoFrameOrientation desiredOrientation = new YoFrameOrientation(name, baseFrame, registry);
         yoDesiredOrientations.put(endEffector, desiredOrientation);
      }
   }
   
   private void setupGains()
   {
      for (int i = 0; i < endEffectors.size(); i++)
      {
         RigidBody endEffector = endEffectors.get(i);
         
         String namePrefixGain = "kp";
         
         String nameSuffixPosition = "_position_" + endEffector.getName();
         String nameSuffixOrientation = "_orientation_" + endEffector.getName();
         
         DoubleYoVariable positionGains = new DoubleYoVariable(namePrefixGain + nameSuffixPosition, registry);
         yoPositionGains.put(endEffector, positionGains);
         
         DoubleYoVariable zeta = new DoubleYoVariable("zeta_" + endEffector.getName(), registry);
         yoZetas.put(endEffector, zeta);
         
         DoubleYoVariable orientationGains = new DoubleYoVariable(namePrefixGain + nameSuffixOrientation, registry);
         yoOrientationGains.put(endEffector, orientationGains);
         
         DoubleYoVariable individualControllerGainScaling = new DoubleYoVariable("highLevelPControl_" + endEffector.getName() + "_gainScaling", registry);
         individualControllerGainScalingMap.put(endEffector, individualControllerGainScaling);
      }
   }

   private void setupSelectionMatrices()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody foot = feet.get(robotSide);
         selectionMatrices.put(foot, CommonOps.identity(SpatialMotionVector.SIZE));

         RigidBody hand = hands.get(robotSide);
         selectionMatrices.put(hand, CommonOps.identity(SpatialMotionVector.SIZE));
      }
      
      DenseMatrix64F orientationOnly = new DenseMatrix64F(3, SpatialMotionVector.SIZE);
      for (int i = 0; i < orientationOnly.getNumRows(); i++)
         orientationOnly.set(i, i, 1.0);

      selectionMatrices.put(chest, new DenseMatrix64F(orientationOnly));
      selectionMatrices.put(head, new DenseMatrix64F(orientationOnly));
   }

   private void initializeGains()
   {
      allControllerGains.set(40.0);
      allControllerZetas.set(0.5);
      gainScaling.set(0.0);
      gainScaling.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            gainScaling.set(MathTools.clipToMinMax(gainScaling.getDoubleValue(), 0.0, 1.0));
         }
      });
      
      double defaultGain = allControllerGains.getDoubleValue();
      double defaultZeta = allControllerZetas.getDoubleValue();
      
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody foot = feet.get(robotSide);
         yoPositionGains.get(foot).set(defaultGain);
         yoOrientationGains.get(foot).set(defaultGain);
         yoZetas.get(foot).set(defaultZeta);

         RigidBody hand = hands.get(robotSide);
         yoPositionGains.get(hand).set(defaultGain);
         yoOrientationGains.get(hand).set(defaultGain);
         yoZetas.get(hand).set(defaultZeta);
      }

      yoOrientationGains.get(chest).set(defaultGain);
      yoZetas.get(chest).set(defaultZeta);

      yoOrientationGains.get(head).set(defaultGain);
      yoZetas.get(head).set(defaultZeta);
   }
   
   private void setupGainChangedListeners()
   {
      VariableChangedListener gainChangedListener = new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            for (int i = 0; i < endEffectors.size(); i++)
            {
               RigidBody endEffector = endEffectors.get(i);

               double scaleFactor = gainScaling.getDoubleValue() * individualControllerGainScalingMap.get(endEffector).getDoubleValue();
               
               double kpPosition = yoPositionGains.get(endEffector).getDoubleValue() * scaleFactor;
               double kdPosition = GainCalculator.computeDerivativeGain(kpPosition, yoZetas.get(endEffector).getDoubleValue()) * scaleFactor;
               double kpOrientation = yoOrientationGains.get(endEffector).getDoubleValue() * scaleFactor;
               double kdOrientation = GainCalculator.computeDerivativeGain(kpOrientation, yoZetas.get(endEffector).getDoubleValue()) * scaleFactor;
               
               spatialAccelerationControlModules.get(endEffector).setPositionProportionalGains(kpPosition, kpPosition, kpPosition);
               spatialAccelerationControlModules.get(endEffector).setPositionDerivativeGains(kdPosition, kdPosition, kdPosition);
               spatialAccelerationControlModules.get(endEffector).setOrientationProportionalGains(kpOrientation, kpOrientation, kpOrientation);
               spatialAccelerationControlModules.get(endEffector).setOrientationDerivativeGains(kdOrientation, kdOrientation, kdOrientation);
            }
         }
      };
      
      for (int i = 0; i < endEffectors.size(); i++)
      {
         RigidBody endEffector = endEffectors.get(i);

         yoPositionGains.get(endEffector).addVariableChangedListener(gainChangedListener);
         yoOrientationGains.get(endEffector).addVariableChangedListener(gainChangedListener);
         yoZetas.get(endEffector).addVariableChangedListener(gainChangedListener);
         
         final DoubleYoVariable individualControllerGainScaling = individualControllerGainScalingMap.get(endEffector);
         individualControllerGainScaling.addVariableChangedListener(gainChangedListener);
         individualControllerGainScaling.addVariableChangedListener(new VariableChangedListener()
         {
            @Override
            public void variableChanged(YoVariable<?> v)
            {
               individualControllerGainScaling.set(MathTools.clipToMinMax(individualControllerGainScaling.getDoubleValue(), 0.0, 1.0));
            }
         });
      }

      allControllerGains.addVariableChangedListener(gainChangedListener);
      allControllerZetas.addVariableChangedListener(gainChangedListener);
      gainScaling.addVariableChangedListener(gainChangedListener);
      
      gainChangedListener.variableChanged(null);
   }
   
   private final FramePoint desiredPosition = new FramePoint();
   private final FrameOrientation desiredOrientation = new FrameOrientation();
   private final FrameVector zeroDesiredLinearVelocity = new FrameVector();
   private final FrameVector zeroDesiredAngularVelocity = new FrameVector();
   private final FrameVector zeroDesiredLinearAcceleration = new FrameVector();
   private final FrameVector zeroDesiredAngularAcceleration = new FrameVector();
   
   private final SpatialAccelerationVector spatialAccelerationOutput = new SpatialAccelerationVector();
   
   private final DenseMatrix64F emptyNullspaceMultipliers = new DenseMatrix64F(0, 1);
   
   @Override
   public void doAction()
   {
      momentumBasedController.doPrioritaryControl();

      for (int i = 0; i < endEffectors.size(); i++)
      {
         RigidBody endEffector = endEffectors.get(i);

         RigidBodySpatialAccelerationControlModule spatialAccelerationControlModule = spatialAccelerationControlModules.get(endEffector);

         ReferenceFrame endEffectorControlFrame = endEffector.getBodyFixedFrame();
         zeroDesiredLinearVelocity.setToZero(endEffectorControlFrame);
         zeroDesiredAngularVelocity.setToZero(endEffectorControlFrame);
         zeroDesiredLinearAcceleration.setToZero(endEffectorControlFrame);
         zeroDesiredAngularAcceleration.setToZero(endEffectorControlFrame);

         yoDesiredPositions.get(endEffector).getFrameTupleIncludingFrame(desiredPosition);
         yoDesiredOrientations.get(endEffector).getFrameOrientationIncludingFrame(desiredOrientation);

         spatialAccelerationControlModule.doPositionControl(desiredPosition, desiredOrientation, zeroDesiredLinearVelocity, zeroDesiredAngularVelocity,
               zeroDesiredLinearAcceleration, zeroDesiredAngularAcceleration, baseForEveryControl);

         spatialAccelerationControlModule.packAcceleration(spatialAccelerationOutput);
         TaskspaceConstraintData taskspaceConstraintData = taskspaceConstraintDataMap.get(endEffector);
         taskspaceConstraintData.set(baseForEveryControl, endEffector);
         taskspaceConstraintData.set(spatialAccelerationOutput, emptyNullspaceMultipliers, selectionMatrices.get(endEffector));
         momentumBasedController.setDesiredSpatialAcceleration(jacobianIdList.get(endEffector), taskspaceConstraintData);
      }
      
      momentumBasedController.doSecondaryControl();
      
      SixDoFJoint rootJoint = fullRobotModel.getRootJoint();
      rootJoint.setDesiredAccelerationToZero();
      momentumBasedController.getInverseDynamicsCalculator().compute();
   }

   @Override
   public void doTransitionIntoAction()
   {
      for (int i = 0; i < endEffectors.size(); i++)
      {
         RigidBody endEffector = endEffectors.get(i);
         ReferenceFrame endEffectorControlFrame = endEffector.getBodyFixedFrame();
         ReferenceFrame baseFrame = baseForEveryControl.getBodyFixedFrame();
         
         desiredPosition.setToZero(endEffectorControlFrame);
         desiredPosition.changeFrame(baseFrame);
         yoDesiredPositions.get(endEffector).set(desiredPosition);
         
         desiredOrientation.setToZero(endEffectorControlFrame);
         desiredOrientation.changeFrame(baseFrame);
         yoDesiredOrientations.get(endEffector).set(desiredOrientation);
      }
      
      momentumBasedController.clearContacts();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      momentumBasedController.setMomentumControlModuleToUse(MOMENTUM_CONTROL_MODULE_TO_USE);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }
}
