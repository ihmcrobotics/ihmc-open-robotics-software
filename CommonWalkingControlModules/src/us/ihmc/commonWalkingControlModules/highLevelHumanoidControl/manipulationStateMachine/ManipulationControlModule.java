package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.HandControllerInterface;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import java.util.Map;

/**
 * @author twan
 *         Date: 5/13/13
 */
public class ManipulationControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final SideDependentList<IndividualHandControlStateMachine> individualHandControlStateMachines =
      new SideDependentList<IndividualHandControlStateMachine>();
   private final SideDependentList<GeometricJacobian> jacobians = new SideDependentList<GeometricJacobian>();

   public ManipulationControlModule(DoubleYoVariable yoTime, FullRobotModel fullRobotModel, TwistCalculator twistCalculator,
                                    WalkingControllerParameters walkingControllerParameters, DesiredHandPoseProvider handPoseProvider,
                                    DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
                                    SideDependentList<HandControllerInterface> handControllers, MomentumBasedController momentumBasedController,
                                    YoVariableRegistry parentRegistry)
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         HandControllerInterface handControllerInterface = null;
         if (handControllers != null)
            handControllerInterface = handControllers.get(robotSide);

         // TODO: create manipulationControlParameters, have current walking parameters class implement it

         RigidBody endEffector = fullRobotModel.getHand(robotSide);
         GeometricJacobian jacobian = new GeometricJacobian(fullRobotModel.getChest(), endEffector, endEffector.getBodyFixedFrame());
         jacobians.put(robotSide, jacobian);

         Map<OneDoFJoint, Double> defaultArmJointPositions = walkingControllerParameters.getDefaultArmJointPositions(fullRobotModel, robotSide);
         Map<OneDoFJoint, Double> minTaskSpacePositions = walkingControllerParameters.getMinTaskspaceArmJointPositions(fullRobotModel, robotSide);
         Map<OneDoFJoint, Double> maxTaskSpacePositions = walkingControllerParameters.getMaxTaskspaceArmJointPositions(fullRobotModel, robotSide);

         individualHandControlStateMachines.put(robotSide,
               new IndividualHandControlStateMachine(yoTime, robotSide, fullRobotModel, twistCalculator, walkingControllerParameters, handPoseProvider,
                     dynamicGraphicObjectsListRegistry, handControllerInterface, momentumBasedController.getGravityZ(), momentumBasedController.getControlDT(),
                     momentumBasedController, jacobian, defaultArmJointPositions, minTaskSpacePositions, maxTaskSpacePositions, registry));
      }

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      for (IndividualHandControlStateMachine stateMachine : individualHandControlStateMachines)
      {
         stateMachine.initialize();
      }
   }

   public void doControl()
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         jacobians.get(robotSide).compute();
         individualHandControlStateMachines.get(robotSide).doControl();
      }
   }
}
