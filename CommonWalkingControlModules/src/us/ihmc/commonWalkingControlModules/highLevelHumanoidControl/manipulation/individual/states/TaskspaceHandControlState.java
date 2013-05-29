package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.statemachines.State;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.states.direct.DirectIndividualHandControlState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.utilities.FormattingTools;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;

public abstract class TaskspaceHandControlState extends State<DirectIndividualHandControlState>
{
   protected final String name;
   protected final YoVariableRegistry registry;
   protected final GeometricJacobian jacobian;

   private final TaskspaceConstraintData taskspaceConstraintData = new TaskspaceConstraintData();
   protected final MomentumBasedController momentumBasedController;


   public TaskspaceHandControlState(DirectIndividualHandControlState stateEnum, MomentumBasedController momentumBasedController, GeometricJacobian jacobian,
                                    YoVariableRegistry parentRegistry)
   {
      super(stateEnum);

      RigidBody endEffector = jacobian.getEndEffector();
      name = endEffector.getName() + FormattingTools.underscoredToCamelCase(this.stateEnum.toString(), true) + "State";
      registry = new YoVariableRegistry(name);

      this.momentumBasedController = momentumBasedController;
      this.jacobian = jacobian;

      parentRegistry.addChild(registry);
   }

   @Override
   public final void doAction()
   {
      SpatialAccelerationVector handAcceleration = computeDesiredSpatialAcceleration();
      taskspaceConstraintData.set(handAcceleration);
      momentumBasedController.setDesiredSpatialAcceleration(jacobian, taskspaceConstraintData);
   }

   protected abstract SpatialAccelerationVector computeDesiredSpatialAcceleration();

   @Override
   public void doTransitionIntoAction()
   {
   }

   @Override
   public void doTransitionOutOfAction()
   {
   }
}
