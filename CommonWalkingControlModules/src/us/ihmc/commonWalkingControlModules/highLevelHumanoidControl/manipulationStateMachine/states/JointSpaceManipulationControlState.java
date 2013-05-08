package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.states;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.PDController;
import com.yobotics.simulationconstructionset.util.trajectory.YoPolynomial;
import org.ejml.data.DenseMatrix64F;
import us.ihmc.commonWalkingControlModules.calculators.GainCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.screwTheory.*;

import java.util.LinkedHashMap;
import java.util.Map;


public class JointSpaceManipulationControlState<T extends Enum<T>> extends IndividualManipulationState
{
   private final DoubleYoVariable yoTime;
   private final OneDoFJoint[] oneDoFJoints;
   private final LinkedHashMap<OneDoFJoint, YoPolynomial> polynomialLinkedHashMap;
   private final LinkedHashMap<OneDoFJoint, PDController> pdControllerLinkedHashMap;

   private final DoubleYoVariable kpAllArmJoints, kdAllArmJoints, zetaAllArmJoints;

   private final DoubleYoVariable moveTimeArmJoint;

   private final YoVariableRegistry registry;
   private final MomentumBasedController momentumBasedController;

   private final Map<OneDoFJoint, Double> desiredJointPositions = new LinkedHashMap<OneDoFJoint, Double>();
   private final SpatialAccelerationVector desiredHandAcceleration;

   private final DoubleYoVariable endMoveTime;

   public JointSpaceManipulationControlState(T stateEnum, DoubleYoVariable yoTime, RobotSide robotSide, GeometricJacobian jacobian, MomentumBasedController momentumBasedController, Map<OneDoFJoint, Double> desiredJointPositions, YoVariableRegistry parentRegistry)
   {
      super(stateEnum);
      this.yoTime = yoTime;

      registry = new YoVariableRegistry("ArmJointController" + robotSide.getCamelCaseNameForMiddleOfExpression());

      this.desiredJointPositions.putAll(desiredJointPositions);

      endMoveTime = new DoubleYoVariable("endMoveTime", registry);

      moveTimeArmJoint = new DoubleYoVariable("moveTimeArmJoint", registry);
      moveTimeArmJoint.set(2.0);

      kpAllArmJoints = new DoubleYoVariable("kpAllArmJoints" + robotSide, registry);
      kpAllArmJoints.set(120.0);

      zetaAllArmJoints = new DoubleYoVariable("zetaAllArmJoints" + robotSide, registry);
      zetaAllArmJoints.set(0.707);

      kdAllArmJoints = new DoubleYoVariable("kdAllArmJoints" + robotSide, registry);
      updateDerivativeGain();

      polynomialLinkedHashMap = new LinkedHashMap<OneDoFJoint, YoPolynomial>();
      pdControllerLinkedHashMap = new LinkedHashMap<OneDoFJoint, PDController>();

      InverseDynamicsJoint[] joints = ScrewTools.createJointPath(jacobian.getBase(), jacobian.getEndEffector());
      this.oneDoFJoints = ScrewTools.filterJoints(joints, RevoluteJoint.class);

      this.desiredHandAcceleration = new SpatialAccelerationVector(jacobian.getEndEffectorFrame(), jacobian.getBaseFrame(), jacobian.getEndEffectorFrame());

      int orderOfPolynomial = 6;
      for (OneDoFJoint joint : oneDoFJoints)
      {
         YoPolynomial yoPolynomial = new YoPolynomial(joint.getName() + robotSide.getCamelCaseNameForMiddleOfExpression(), orderOfPolynomial, registry);
         polynomialLinkedHashMap.put(joint, yoPolynomial);

         PDController pdController = new PDController(kpAllArmJoints, kdAllArmJoints, joint.getName() + robotSide.getCamelCaseNameForMiddleOfExpression(),
                                        registry);
         pdControllerLinkedHashMap.put(joint, pdController);
      }

      this.momentumBasedController = momentumBasedController;

      parentRegistry.addChild(registry);
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
         YoPolynomial yoPolynomial = polynomialLinkedHashMap.get(joint);
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
         YoPolynomial yoPolynomial = polynomialLinkedHashMap.get(joint);
         currentTime = MathTools.clipToMinMax(currentTime, Double.NEGATIVE_INFINITY, endMoveTime.getDoubleValue());
         yoPolynomial.compute(currentTime);


         double desiredPosition = yoPolynomial.getPosition();
         double desiredVelocity = yoPolynomial.getVelocity();
         double feedforwardAcceleration = yoPolynomial.getAcceleration();

         double currentPosition = joint.getQ();
         double currentVelocity = joint.getQd();

         PDController pdController = pdControllerLinkedHashMap.get(joint);
         double desiredAccleration = feedforwardAcceleration + pdController.compute(currentPosition, desiredPosition, currentVelocity, desiredVelocity);

         DenseMatrix64F jointAccelerationMatrix = new DenseMatrix64F(joint.getDegreesOfFreedom(), 1);
         jointAccelerationMatrix.set(0, 0, desiredAccleration);
         momentumBasedController.setDesiredJointAcceleration(joint, jointAccelerationMatrix);
      }
   }

   private void checkIfJointsMatch(LinkedHashMap<OneDoFJoint, Double> desiredJointPositions)
   {
      for (OneDoFJoint joint : oneDoFJoints)
      {
         if (!desiredJointPositions.containsKey(joint))
            throw new RuntimeException(joint.getName() + " not found in desiredJointPositions");
      }
   }

   @Override
   public SpatialAccelerationVector getDesiredHandAcceleration()
   {
      return desiredHandAcceleration;
   }

   @Override
   public boolean isDone()
   {
      return (yoTime.getDoubleValue() >= endMoveTime.getDoubleValue());
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
