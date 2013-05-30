package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.PDController;
import com.yobotics.simulationconstructionset.util.statemachines.State;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.calculators.GainCalculator;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.IndividualHandControlState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.FormattingTools;
import us.ihmc.utilities.screwTheory.*;

import java.util.LinkedHashMap;
import java.util.Map;


public class JointSpaceHandControlControlState extends State<IndividualHandControlState>
{
   private final OneDoFJoint[] oneDoFJoints;
   private final LinkedHashMap<OneDoFJoint, DoubleTrajectoryGenerator> trajectories;
   private final LinkedHashMap<OneDoFJoint, PDController> pdControllers;

   private final DoubleYoVariable kpAllArmJoints, kdAllArmJoints, zetaAllArmJoints;

   private final DoubleYoVariable moveTimeArmJoint;

   private final YoVariableRegistry registry;
   private final MomentumBasedController momentumBasedController;

   public JointSpaceHandControlControlState(IndividualHandControlState stateEnum, RobotSide robotSide, GeometricJacobian jacobian,
           MomentumBasedController momentumBasedController, YoVariableRegistry parentRegistry, double moveTime)
   {
      super(stateEnum);

      RigidBody endEffector = jacobian.getEndEffector();
      registry = new YoVariableRegistry(endEffector.getName() + FormattingTools.underscoredToCamelCase(this.stateEnum.toString(), true) + "State");

      moveTimeArmJoint = new DoubleYoVariable("moveTimeArmJoint", registry);
      moveTimeArmJoint.set(moveTime);

      kpAllArmJoints = new DoubleYoVariable("kpAllArmJoints" + robotSide, registry);
      kpAllArmJoints.set(120.0);

      zetaAllArmJoints = new DoubleYoVariable("zetaAllArmJoints" + robotSide, registry);
      zetaAllArmJoints.set(1.0);

      kdAllArmJoints = new DoubleYoVariable("kdAllArmJoints" + robotSide, registry);
      updateDerivativeGain();

      trajectories = new LinkedHashMap<OneDoFJoint, DoubleTrajectoryGenerator>();
      pdControllers = new LinkedHashMap<OneDoFJoint, PDController>();

      InverseDynamicsJoint[] joints = ScrewTools.createJointPath(jacobian.getBase(), jacobian.getEndEffector());
      this.oneDoFJoints = ScrewTools.filterJoints(joints, RevoluteJoint.class);

      for (OneDoFJoint joint : oneDoFJoints)
      {
         PDController pdController = new PDController(kpAllArmJoints, kdAllArmJoints, joint.getName() + robotSide.getCamelCaseNameForMiddleOfExpression(),
                                        registry);
         pdControllers.put(joint, pdController);
      }

      this.momentumBasedController = momentumBasedController;

      parentRegistry.addChild(registry);
   }

   private void updateDerivativeGain()
   {
      kdAllArmJoints.set(GainCalculator.computeDerivativeGain(kpAllArmJoints.getDoubleValue(), zetaAllArmJoints.getDoubleValue()));
   }

   private void setDesiredJointAccelerations()
   {
      updateDerivativeGain();

      for (OneDoFJoint joint : oneDoFJoints)
      {
         DoubleTrajectoryGenerator trajectoryGenerator = trajectories.get(joint);
         trajectoryGenerator.compute(getTimeInCurrentState());

         double desiredPosition = trajectoryGenerator.getValue();
         double desiredVelocity = trajectoryGenerator.getVelocity();
         double feedforwardAcceleration = trajectoryGenerator.getAcceleration();

         double currentPosition = joint.getQ();
         double currentVelocity = joint.getQd();

         PDController pdController = pdControllers.get(joint);
         double desiredAcceleration = feedforwardAcceleration + pdController.compute(currentPosition, desiredPosition, currentVelocity, desiredVelocity);

         momentumBasedController.setOneDoFJointAcceleration(joint, desiredAcceleration);
      }
   }

   @Override
   public void doAction()
   {
      setDesiredJointAccelerations();
   }

   @Override
   public void doTransitionIntoAction()
   {
      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         trajectories.get(oneDoFJoint).initialize();
      }
   }

   @Override
   public void doTransitionOutOfAction()
   {
      // empty
   }

   public boolean isDone()
   {
      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         if (!trajectories.get(oneDoFJoint).isDone())
            return false;
      }

      return true;
   }

   public void setTrajectories(Map<OneDoFJoint, ? extends DoubleTrajectoryGenerator> trajectories)
   {
      this.trajectories.clear();
      this.trajectories.putAll(trajectories);
   }
}
