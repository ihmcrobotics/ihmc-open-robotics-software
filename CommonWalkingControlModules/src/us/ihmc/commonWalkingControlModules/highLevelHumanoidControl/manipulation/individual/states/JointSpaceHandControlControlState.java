package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import java.util.LinkedHashMap;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.IndividualHandControlState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.FormattingTools;
import us.ihmc.utilities.maps.ObjectObjectMap;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.VariableChangedListener;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.GainCalculator;
import com.yobotics.simulationconstructionset.util.PDController;
import com.yobotics.simulationconstructionset.util.math.filter.RateLimitedYoVariable;
import com.yobotics.simulationconstructionset.util.statemachines.State;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleTrajectoryGenerator;


public class JointSpaceHandControlControlState extends State<IndividualHandControlState>
{
   private final OneDoFJoint[] oneDoFJoints;
   private final LinkedHashMap<OneDoFJoint, DoubleTrajectoryGenerator> trajectories;
   private final LinkedHashMap<OneDoFJoint, PDController> pdControllers;

   private final ObjectObjectMap<OneDoFJoint, RateLimitedYoVariable> rateLimitedAccelerations;

   
   private final DoubleYoVariable kpAllArmJoints, kdAllArmJoints, zetaAllArmJoints, maxAccelerationArmJoints, maxJerkArmJoints;

   private final DoubleYoVariable moveTimeArmJoint;

   private final YoVariableRegistry registry;
   private final MomentumBasedController momentumBasedController;

   public JointSpaceHandControlControlState(double dt, IndividualHandControlState stateEnum, RobotSide robotSide, GeometricJacobian jacobian,
           MomentumBasedController momentumBasedController, ArmControllerParameters armControllerParameters, YoVariableRegistry parentRegistry, double moveTime)
   {
      super(stateEnum);

      RigidBody endEffector = jacobian.getEndEffector();
      registry = new YoVariableRegistry(endEffector.getName() + FormattingTools.underscoredToCamelCase(this.stateEnum.toString(), true) + "State");

      moveTimeArmJoint = new DoubleYoVariable("moveTimeArmJoint", registry);
      moveTimeArmJoint.set(moveTime);

      kpAllArmJoints = new DoubleYoVariable("kpAllArmJoints" + robotSide, registry);
      kpAllArmJoints.set(armControllerParameters.getKpAllArmJoints());

      zetaAllArmJoints = new DoubleYoVariable("zetaAllArmJoints" + robotSide, registry);
      zetaAllArmJoints.set(armControllerParameters.getZetaAllArmJoints());

      kdAllArmJoints = new DoubleYoVariable("kdAllArmJoints" + robotSide, registry);

      maxAccelerationArmJoints = new DoubleYoVariable("maxAccelerationArmJoints" + robotSide, registry);
      maxJerkArmJoints = new DoubleYoVariable("maxJerkArmJoints" + robotSide, registry);
      
      maxAccelerationArmJoints.set(armControllerParameters.getMaxAccelerationAllArmJoints());
      maxJerkArmJoints.set(armControllerParameters.getMaxJerkAllArmJoints());
      
      setupVariableListener();

      trajectories = new LinkedHashMap<OneDoFJoint, DoubleTrajectoryGenerator>();
      pdControllers = new LinkedHashMap<OneDoFJoint, PDController>();
      rateLimitedAccelerations = new ObjectObjectMap<OneDoFJoint, RateLimitedYoVariable>();
      
      InverseDynamicsJoint[] joints = ScrewTools.createJointPath(jacobian.getBase(), jacobian.getEndEffector());
      this.oneDoFJoints = ScrewTools.filterJoints(joints, RevoluteJoint.class);

      for (OneDoFJoint joint : oneDoFJoints)
      {
         PDController pdController = new PDController(kpAllArmJoints, kdAllArmJoints, joint.getName() + robotSide.getCamelCaseNameForMiddleOfExpression(),
                                        registry);
         pdControllers.put(joint, pdController);
         
         
         RateLimitedYoVariable rateLimitedAcceleration = new RateLimitedYoVariable(joint.getName() + "Acceleration" + robotSide, registry, maxJerkArmJoints, dt);
         rateLimitedAccelerations.add(joint, rateLimitedAcceleration);
      }

      this.momentumBasedController = momentumBasedController;

      parentRegistry.addChild(registry);
      
      setupVariableListener();
   }

   private void setupVariableListener()
   {
      VariableChangedListener listener = new VariableChangedListener()
      {
         public void variableChanged(YoVariable v)
         {
            kdAllArmJoints.set(GainCalculator.computeDerivativeGain(kpAllArmJoints.getDoubleValue(), zetaAllArmJoints.getDoubleValue()));            
         }
      };

      kpAllArmJoints.addVariableChangedListener(listener);
      kdAllArmJoints.addVariableChangedListener(listener);
      
      listener.variableChanged(null);
   }

   private void setDesiredJointAccelerations()
   {
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

         desiredAcceleration = MathTools.clipToMinMax(desiredAcceleration, maxAccelerationArmJoints.getDoubleValue());
         
         RateLimitedYoVariable rateLimitedAcceleration = rateLimitedAccelerations.get(joint);
         rateLimitedAcceleration.update(desiredAcceleration);
         desiredAcceleration = rateLimitedAcceleration.getDoubleValue();
         
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
