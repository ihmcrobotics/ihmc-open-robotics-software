package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.IndividualHandControlState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.utilities.FormattingTools;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.statemachines.State;

public abstract class TaskspaceHandControlState extends State<IndividualHandControlState>
{
   protected final String name;
   protected final YoVariableRegistry registry;
   protected final int jacobianId;

   private final TaskspaceConstraintData taskspaceConstraintData = new TaskspaceConstraintData();
   protected final MomentumBasedController momentumBasedController;


   public TaskspaceHandControlState(String namePrefix, IndividualHandControlState stateEnum, MomentumBasedController momentumBasedController, int jacobianId,
                                    YoVariableRegistry parentRegistry)
   {
      super(stateEnum);

      name = namePrefix + FormattingTools.underscoredToCamelCase(this.stateEnum.toString(), true) + "State";
      registry = new YoVariableRegistry(name);

      this.momentumBasedController = momentumBasedController;
      this.jacobianId = jacobianId;

      parentRegistry.addChild(registry);
   }

   @Override
   public void doAction()
   {
      SpatialAccelerationVector handAcceleration = computeDesiredSpatialAcceleration();
      taskspaceConstraintData.set(handAcceleration);
      momentumBasedController.setDesiredSpatialAcceleration(jacobianId, taskspaceConstraintData);
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
