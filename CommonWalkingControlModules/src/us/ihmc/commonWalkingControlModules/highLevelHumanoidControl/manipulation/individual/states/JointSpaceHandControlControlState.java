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
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.ScrewTools;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.VariableChangedListener;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.GainCalculator;
import com.yobotics.simulationconstructionset.util.PIDController;
import com.yobotics.simulationconstructionset.util.math.filter.RateLimitedYoVariable;
import com.yobotics.simulationconstructionset.util.statemachines.State;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleTrajectoryGenerator;


public class JointSpaceHandControlControlState extends State<IndividualHandControlState>
{
   private final OneDoFJoint[] oneDoFJoints;
   private final LinkedHashMap<OneDoFJoint, DoubleTrajectoryGenerator> trajectories;
   private final LinkedHashMap<OneDoFJoint, PIDController> pidControllers;

   private final ObjectObjectMap<OneDoFJoint, RateLimitedYoVariable> rateLimitedAccelerations;

   
   private final DoubleYoVariable kpArmJointspace, kdArmJointspace, kiArmJointspace, zetaArmJointspace, maxAccelerationArmJointspace, maxJerkArmJointspace, maxIntegralErrorArmJointspace;

   private final DoubleYoVariable moveTimeArmJoint;

   private final YoVariableRegistry registry;
   private final MomentumBasedController momentumBasedController;
   
   private final double dt;

   public JointSpaceHandControlControlState(String namePrefix, IndividualHandControlState stateEnum, RobotSide robotSide,
                                            InverseDynamicsJoint[] controlledJoints, int jacobianId, MomentumBasedController momentumBasedController,
                                            ArmControllerParameters armControllerParameters, double dt, double moveTime, YoVariableRegistry parentRegistry)
   {
      super(stateEnum);

      this.dt = dt;
      
      registry = new YoVariableRegistry(namePrefix + FormattingTools.underscoredToCamelCase(this.stateEnum.toString(), true) + "State");

      moveTimeArmJoint = new DoubleYoVariable("moveTimeArmJoint", registry);
      moveTimeArmJoint.set(moveTime);

      kpArmJointspace = new DoubleYoVariable("kpArmJointspace" + robotSide, registry);
      kpArmJointspace.set(armControllerParameters.getArmJointspaceKp());

      zetaArmJointspace = new DoubleYoVariable("zetaArmJointspace" + robotSide, registry);
      zetaArmJointspace.set(armControllerParameters.getArmJointspaceZeta());

      kdArmJointspace = new DoubleYoVariable("kdArmJointspace" + robotSide, registry);

      kiArmJointspace = new DoubleYoVariable("kiArmJointspace" + robotSide, registry);
      kiArmJointspace.set(armControllerParameters.getArmJointspaceKi());

      maxAccelerationArmJointspace = new DoubleYoVariable("maxAccelerationArmJointspace" + robotSide, registry);
      maxJerkArmJointspace = new DoubleYoVariable("maxJerkArmJointspace" + robotSide, registry);
      
      maxIntegralErrorArmJointspace = new DoubleYoVariable("maxIntegralErrorArmJointspace" + robotSide, registry);
      maxIntegralErrorArmJointspace.set(0.30);
      
      maxAccelerationArmJointspace.set(armControllerParameters.getArmJointspaceMaxAcceleration());
      maxJerkArmJointspace.set(armControllerParameters.getArmJointspaceMaxJerk());
      
      setupVariableListener();

      trajectories = new LinkedHashMap<OneDoFJoint, DoubleTrajectoryGenerator>();
      pidControllers = new LinkedHashMap<OneDoFJoint, PIDController>();
      rateLimitedAccelerations = new ObjectObjectMap<OneDoFJoint, RateLimitedYoVariable>();
      
      this.oneDoFJoints = ScrewTools.filterJoints(controlledJoints, RevoluteJoint.class);

      for (OneDoFJoint joint : oneDoFJoints)
      {
         PIDController pidController = new PIDController(kpArmJointspace, kiArmJointspace, kdArmJointspace, maxIntegralErrorArmJointspace, joint.getName() + robotSide.getCamelCaseNameForMiddleOfExpression(),
                                        registry);
         pidControllers.put(joint, pidController);
         
         
         RateLimitedYoVariable rateLimitedAcceleration = new RateLimitedYoVariable(joint.getName() + "Acceleration" + robotSide, registry, maxJerkArmJointspace, dt);
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
            kdArmJointspace.set(GainCalculator.computeDerivativeGain(kpArmJointspace.getDoubleValue(), zetaArmJointspace.getDoubleValue()));            
         }
      };

      kpArmJointspace.addVariableChangedListener(listener);
      zetaArmJointspace.addVariableChangedListener(listener);
      kdArmJointspace.addVariableChangedListener(listener);
      
      listener.variableChanged(null);
   }

   private void setDesiredJointAccelerations()
   {
      for (int i = 0 ; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         DoubleTrajectoryGenerator trajectoryGenerator = trajectories.get(joint);
         trajectoryGenerator.compute(getTimeInCurrentState());

         double desiredPosition = trajectoryGenerator.getValue();
         double desiredVelocity = trajectoryGenerator.getVelocity();
         double feedforwardAcceleration = trajectoryGenerator.getAcceleration();

         double currentPosition = joint.getQ();
         double currentVelocity = joint.getQd();

         PIDController pidController = pidControllers.get(joint);
         double desiredAcceleration = feedforwardAcceleration + pidController.computeForAngles(currentPosition, desiredPosition, currentVelocity, desiredVelocity, dt);

         desiredAcceleration = MathTools.clipToMinMax(desiredAcceleration, maxAccelerationArmJointspace.getDoubleValue());
         
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
      for (int i = 0 ; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         trajectories.get(joint).initialize();
         pidControllers.get(joint).setCumulativeError(0.0);
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
