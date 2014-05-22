package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import java.util.LinkedHashMap;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.IndividualHandControlState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.trajectories.OneDoFJointQuinticTrajectoryGenerator;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.FormattingTools;
import us.ihmc.utilities.maps.ObjectObjectMap;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.ScrewTools;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.VariableChangedListener;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.GainCalculator;
import com.yobotics.simulationconstructionset.util.PIDController;
import com.yobotics.simulationconstructionset.util.math.filter.RateLimitedYoVariable;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleTrajectoryGenerator;


public class JointSpaceHandControlControlState extends AbstractJointSpaceHandControlState
{
   private final OneDoFJoint[] oneDoFJoints;
   private final LinkedHashMap<OneDoFJoint, OneDoFJointQuinticTrajectoryGenerator> trajectories;
   private final LinkedHashMap<OneDoFJoint, PIDController> pidControllers;

   private final ObjectObjectMap<OneDoFJoint, RateLimitedYoVariable> rateLimitedAccelerations;

   
   private final DoubleYoVariable kpArmJointspace, kdArmJointspace, kiArmJointspace, zetaArmJointspace, maxAccelerationArmJointspace, maxJerkArmJointspace, maxIntegralErrorArmJointspace;

   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> desiredPositions = new LinkedHashMap<OneDoFJoint, DoubleYoVariable>();
   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> desiredVelocities = new LinkedHashMap<OneDoFJoint, DoubleYoVariable>();
   
   private final YoVariableRegistry registry;
   private final MomentumBasedController momentumBasedController;
   private final BooleanYoVariable initialized;
   
   private final double dt;

   public JointSpaceHandControlControlState(String namePrefix, IndividualHandControlState stateEnum, RobotSide robotSide,
                                            InverseDynamicsJoint[] controlledJoints, MomentumBasedController momentumBasedController,
                                            ArmControllerParameters armControllerParameters, double dt, YoVariableRegistry parentRegistry)
   {
      super(stateEnum);

      this.dt = dt;
      
      registry = new YoVariableRegistry(namePrefix + FormattingTools.underscoredToCamelCase(this.stateEnum.toString(), true) + "State");

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
      maxIntegralErrorArmJointspace.set(armControllerParameters.getArmJointspaceMaxIntegralError());
      
      maxAccelerationArmJointspace.set(armControllerParameters.getArmJointspaceMaxAcceleration());
      maxJerkArmJointspace.set(armControllerParameters.getArmJointspaceMaxJerk());
      
      setupVariableListener();

      trajectories = new LinkedHashMap<OneDoFJoint, OneDoFJointQuinticTrajectoryGenerator>();
      pidControllers = new LinkedHashMap<OneDoFJoint, PIDController>();
      rateLimitedAccelerations = new ObjectObjectMap<OneDoFJoint, RateLimitedYoVariable>();
      initialized = new BooleanYoVariable("jointSpaceHandControlStateInitialized", registry);
      initialized.set(false);
      
      
      this.oneDoFJoints = ScrewTools.filterJoints(controlledJoints, RevoluteJoint.class);

      for (OneDoFJoint joint : oneDoFJoints)
      {
         PIDController pidController = new PIDController(kpArmJointspace, kiArmJointspace, kdArmJointspace, maxIntegralErrorArmJointspace, joint.getName() + robotSide.getCamelCaseNameForMiddleOfExpression(),
                                        registry);
         pidControllers.put(joint, pidController);
         
         
         RateLimitedYoVariable rateLimitedAcceleration = new RateLimitedYoVariable(joint.getName() + "Acceleration" + robotSide, registry, maxJerkArmJointspace, dt);
         rateLimitedAccelerations.add(joint, rateLimitedAcceleration);
         
         DoubleYoVariable desiredPosition = new DoubleYoVariable(joint.getName() + "QDesired", registry);
         DoubleYoVariable desiredVelocity = new DoubleYoVariable(joint.getName() + "QdDesired", registry);
         
         desiredPositions.put(joint, desiredPosition);
         desiredVelocities.put(joint, desiredVelocity);
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
         DoubleYoVariable desiredPosition = desiredPositions.get(joint);
         DoubleYoVariable desiredVelocity = desiredVelocities.get(joint);
         DoubleTrajectoryGenerator trajectoryGenerator = trajectories.get(joint);
         trajectoryGenerator.compute(getTimeInCurrentState());

//         double desiredPosition = trajectoryGenerator.getValue();
//         double desiredVelocity = trajectoryGenerator.getVelocity();
         
         desiredPosition.set(trajectoryGenerator.getValue());
         desiredVelocity.set(trajectoryGenerator.getVelocity());
         double feedforwardAcceleration = trajectoryGenerator.getAcceleration();

         double currentPosition = joint.getQ();
         double currentVelocity = joint.getQd();

         PIDController pidController = pidControllers.get(joint);
         double desiredAcceleration = feedforwardAcceleration + pidController.computeForAngles(currentPosition, desiredPosition.getDoubleValue(), currentVelocity, desiredVelocity.getDoubleValue(), dt);

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
      if(initialized.getBooleanValue() && getPreviousState() == this)
      {
         for (int i = 0 ; i < oneDoFJoints.length; i++)
         {
            OneDoFJoint joint = oneDoFJoints[i];
            trajectories.get(joint).reinitialize(desiredPositions.get(joint).getDoubleValue(), desiredVelocities.get(joint).getDoubleValue());
         }
      }
      else
      {
         for (int i = 0 ; i < oneDoFJoints.length; i++)
         {
            OneDoFJoint joint = oneDoFJoints[i];
            trajectories.get(joint).initialize();
            pidControllers.get(joint).setCumulativeError(0.0);
         }
         initialized.set(true);
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

   public void setTrajectories(Map<OneDoFJoint, ? extends OneDoFJointQuinticTrajectoryGenerator> trajectories)
   {
      this.trajectories.clear();
      this.trajectories.putAll(trajectories);
   }
}
