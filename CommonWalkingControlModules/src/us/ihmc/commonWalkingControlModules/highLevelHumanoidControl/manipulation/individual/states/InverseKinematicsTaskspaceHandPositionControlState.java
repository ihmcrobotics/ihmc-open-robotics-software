package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.IndividualHandControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.TrajectoryBasedNumericalInverseKinematicsCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.trajectories.OrientationTrajectoryGenerator;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.VariableChangedListener;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.GainCalculator;
import com.yobotics.simulationconstructionset.util.PIDController;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.filter.RateLimitedYoVariable;
import com.yobotics.simulationconstructionset.util.trajectory.PositionTrajectoryGenerator;

public class InverseKinematicsTaskspaceHandPositionControlState extends TaskspaceHandPositionControlState
{
   private final double controlDT;
   private final TrajectoryBasedNumericalInverseKinematicsCalculator inverseKinematicsCalculator;
   private final LinkedHashMap<OneDoFJoint, PIDController> pidControllers = new LinkedHashMap<OneDoFJoint, PIDController>();
   private final HashMap<OneDoFJoint, RateLimitedYoVariable> rateLimitedAccelerations = new HashMap<OneDoFJoint, RateLimitedYoVariable>();
   private final DoubleYoVariable kpArmJointspace, kdArmJointspace, kiArmJointspace, zetaArmJointspace, maxAccelerationArmJointspace, maxJerkArmJointspace,
         maxIntegralErrorArmJointspace;

   public InverseKinematicsTaskspaceHandPositionControlState(String namePrefix, IndividualHandControlState stateEnum, RobotSide robotSide,
         MomentumBasedController momentumBasedController, int jacobianId, RigidBody base, RigidBody endEffector,
         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, ArmControllerParameters armControllerParameters, double controlDT,
         YoVariableRegistry parentRegistry)
   {
      super(namePrefix, stateEnum, robotSide, momentumBasedController, jacobianId, base, endEffector, dynamicGraphicObjectsListRegistry, parentRegistry);
      this.controlDT = controlDT;
      inverseKinematicsCalculator = new TrajectoryBasedNumericalInverseKinematicsCalculator(base, endEffector, controlDT, momentumBasedController.getTwistCalculator(), parentRegistry, dynamicGraphicObjectsListRegistry);

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
      
      for (OneDoFJoint joint : inverseKinematicsCalculator.getRevoluteJointsInOrder())
      {
         PIDController pidController = new PIDController(kpArmJointspace, kiArmJointspace, kdArmJointspace, maxIntegralErrorArmJointspace, joint.getName()
               + robotSide.getCamelCaseNameForMiddleOfExpression(), registry);
         pidControllers.put(joint, pidController);

         RateLimitedYoVariable rateLimitedAcceleration = new RateLimitedYoVariable(joint.getName() + "Acceleration" + robotSide, registry,
               maxJerkArmJointspace, controlDT);
         rateLimitedAccelerations.put(joint, rateLimitedAcceleration);
      }

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
   
   @Override
   protected SpatialAccelerationVector computeDesiredSpatialAcceleration()
   {
      throw new RuntimeException("Not controlling task space");
   }

   @Override
   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();
      inverseKinematicsCalculator.initialize();

   }

   @Override
   public void doTransitionOutOfAction()
   {
   }

   @Override
   public void doAction()
   {
      inverseKinematicsCalculator.compute(getTimeInCurrentState());

      RevoluteJoint[] revoluteJoints = inverseKinematicsCalculator.getRevoluteJointsInOrder();
      DenseMatrix64F desiredJointAngles = inverseKinematicsCalculator.getDesiredJointAngles();
      DenseMatrix64F desiredJointVelocities = inverseKinematicsCalculator.getDesiredJointVelocities();

      for (int i = 0; i < revoluteJoints.length; i++)
      {
         OneDoFJoint joint = revoluteJoints[i];
         double currentPosition = joint.getQ();
         double currentVelocity = joint.getQd();

         double desiredPosition = desiredJointAngles.get(i, 0);
         double desiredVelocity = desiredJointVelocities.get(i, 0);

         PIDController pidController = pidControllers.get(joint);
         double desiredAcceleration = pidController.computeForAngles(currentPosition, desiredPosition, currentVelocity, desiredVelocity, controlDT);

         desiredAcceleration = MathTools.clipToMinMax(desiredAcceleration, maxAccelerationArmJointspace.getDoubleValue());

         RateLimitedYoVariable rateLimitedAcceleration = rateLimitedAccelerations.get(joint);
         rateLimitedAcceleration.update(desiredAcceleration);
         desiredAcceleration = rateLimitedAcceleration.getDoubleValue();

         momentumBasedController.setOneDoFJointAcceleration(joint, desiredAcceleration);
      }
   }

//   @Override
//   public boolean isDone()
//   {
//      return positionTrajectoryGenerator.isDone() && orientationTrajectoryGenerator.isDone();
//   }

//   public ReferenceFrame getFrameToControlPoseOf()
//   {
//      return handSpatialAccelerationControlModule.getTrackingFrame();
//   }
   
   @Override
   public void setTrajectory(PositionTrajectoryGenerator positionTrajectoryGenerator, OrientationTrajectoryGenerator orientationTrajectoryGenerator,
         Map<OneDoFJoint, Double> finalDesiredJointAngles, RigidBody base, RigidBodySpatialAccelerationControlModule rigidBodySpatialAccelerationControlModule, ReferenceFrame frameToControlPoseOf)
   {
      super.setTrajectory(positionTrajectoryGenerator, orientationTrajectoryGenerator, finalDesiredJointAngles, base, rigidBodySpatialAccelerationControlModule, frameToControlPoseOf);
      inverseKinematicsCalculator.setTrajectory(positionTrajectoryGenerator, orientationTrajectoryGenerator, frameToControlPoseOf);
   }
}
