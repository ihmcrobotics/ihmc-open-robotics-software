package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.states;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.PDController;
import com.yobotics.simulationconstructionset.util.statemachines.State;
import com.yobotics.simulationconstructionset.util.trajectory.YoPolynomial;
import org.ejml.data.DenseMatrix64F;
import us.ihmc.commonWalkingControlModules.calculators.GainCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.screwTheory.*;

import java.util.LinkedHashMap;
import java.util.Map;


public class JointSpaceHandControlControlState extends State<IndividualHandControlState>
{
   private final DoubleYoVariable yoTime;
   private final OneDoFJoint[] oneDoFJoints;
   private final LinkedHashMap<OneDoFJoint, YoPolynomial> trajectories;
   private final LinkedHashMap<OneDoFJoint, PDController> pdControllers;

   private final DoubleYoVariable kpAllArmJoints, kdAllArmJoints, zetaAllArmJoints;

   private final DoubleYoVariable moveTimeArmJoint;

   private final YoVariableRegistry registry;
   private final MomentumBasedController momentumBasedController;

   private final Map<OneDoFJoint, Double> desiredJointPositions = new LinkedHashMap<OneDoFJoint, Double>();

   private final DoubleYoVariable endMoveTime;

   public JointSpaceHandControlControlState(DoubleYoVariable yoTime, RobotSide robotSide, GeometricJacobian jacobian,
                                            MomentumBasedController momentumBasedController, YoVariableRegistry parentRegistry)
   {
      super(IndividualHandControlState.JOINT_SPACE);
      this.yoTime = yoTime;

      registry = new YoVariableRegistry("ArmJointController" + robotSide.getCamelCaseNameForMiddleOfExpression());

      endMoveTime = new DoubleYoVariable("endMoveTime", registry);

      moveTimeArmJoint = new DoubleYoVariable("moveTimeArmJoint", registry);
      moveTimeArmJoint.set(1.0);

      kpAllArmJoints = new DoubleYoVariable("kpAllArmJoints" + robotSide, registry);
      kpAllArmJoints.set(120.0);

      zetaAllArmJoints = new DoubleYoVariable("zetaAllArmJoints" + robotSide, registry);
      zetaAllArmJoints.set(1.0);

      kdAllArmJoints = new DoubleYoVariable("kdAllArmJoints" + robotSide, registry);
      updateDerivativeGain();

      trajectories = new LinkedHashMap<OneDoFJoint, YoPolynomial>();
      pdControllers = new LinkedHashMap<OneDoFJoint, PDController>();

      InverseDynamicsJoint[] joints = ScrewTools.createJointPath(jacobian.getBase(), jacobian.getEndEffector());
      this.oneDoFJoints = ScrewTools.filterJoints(joints, RevoluteJoint.class);

      int orderOfPolynomial = 6;
      for (OneDoFJoint joint : oneDoFJoints)
      {
         YoPolynomial yoPolynomial = new YoPolynomial(joint.getName() + robotSide.getCamelCaseNameForMiddleOfExpression(), orderOfPolynomial, registry);
         trajectories.put(joint, yoPolynomial);

         PDController pdController = new PDController(kpAllArmJoints, kdAllArmJoints, joint.getName() + robotSide.getCamelCaseNameForMiddleOfExpression(),
                                        registry);
         pdControllers.put(joint, pdController);
      }

      this.momentumBasedController = momentumBasedController;

      parentRegistry.addChild(registry);
   }

   public void setDesiredJointPositions(Map<OneDoFJoint, Double> desiredJointPositions)
   {
      this.desiredJointPositions.clear();
      this.desiredJointPositions.putAll(desiredJointPositions);
   }

   private void updateDerivativeGain()
   {
      kdAllArmJoints.set(GainCalculator.computeDerivativeGain(kpAllArmJoints.getDoubleValue(), zetaAllArmJoints.getDoubleValue()));
   }

   public void initializeForMove()
   {
      double startTime = yoTime.getDoubleValue();
      endMoveTime.set(startTime + moveTimeArmJoint.getDoubleValue());
      for (OneDoFJoint joint : oneDoFJoints)
      {
         YoPolynomial yoPolynomial = trajectories.get(joint);
         double desiredEndPosition = desiredJointPositions.get(joint);
         double desiredEndVelocity = 0.0;

         double currentJointPosition = joint.getQ();

         yoPolynomial.setQuintic(startTime, endMoveTime.getDoubleValue(), currentJointPosition, joint.getQd(), 0.0, desiredEndPosition, desiredEndVelocity, 0.0);
      }
   }

   private void setDesiredJointAccelerations()
   {
      updateDerivativeGain();

      double currentTime = yoTime.getDoubleValue();
      for (OneDoFJoint joint : oneDoFJoints)
      {
         YoPolynomial yoPolynomial = trajectories.get(joint);
         currentTime = MathTools.clipToMinMax(currentTime, Double.NEGATIVE_INFINITY, endMoveTime.getDoubleValue());
         yoPolynomial.compute(currentTime);


         double desiredPosition = yoPolynomial.getPosition();
         double desiredVelocity = yoPolynomial.getVelocity();
         double feedforwardAcceleration = yoPolynomial.getAcceleration();

         double currentPosition = joint.getQ();
         double currentVelocity = joint.getQd();

         PDController pdController = pdControllers.get(joint);
         double desiredAccleration = feedforwardAcceleration + pdController.compute(currentPosition, desiredPosition, currentVelocity, desiredVelocity);

         DenseMatrix64F jointAccelerationMatrix = new DenseMatrix64F(joint.getDegreesOfFreedom(), 1);
         jointAccelerationMatrix.set(0, 0, desiredAccleration);
         momentumBasedController.setDesiredJointAcceleration(joint, jointAccelerationMatrix);
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
      initializeForMove();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      // empty
   }
}
