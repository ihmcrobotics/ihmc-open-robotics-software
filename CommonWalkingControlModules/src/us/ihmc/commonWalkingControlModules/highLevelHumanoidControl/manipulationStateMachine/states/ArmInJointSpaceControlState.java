package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.states;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.PDController;
import com.yobotics.simulationconstructionset.util.trajectory.YoPolynomial;
import us.ihmc.commonWalkingControlModules.calculators.GainCalculator;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.*;

import java.util.LinkedHashMap;


public class ArmInJointSpaceControlState
{
   private final DoubleYoVariable yoTime;
   private final OneDoFJoint[] oneDoFJoints;
   private final LinkedHashMap<OneDoFJoint, YoPolynomial> polynomialLinkedHashMap;
   private final LinkedHashMap<OneDoFJoint, PDController> pdControllerLinkedHashMap;

   private final DoubleYoVariable kpAllArmJoints, kdAllArmJoints, zetaAllArmJoints;

   private final DoubleYoVariable moveTimeArmJoint;

   private final YoVariableRegistry registry;

   public ArmInJointSpaceControlState(DoubleYoVariable yoTime, RobotSide robotSide, FullRobotModel fullRobotModel, YoVariableRegistry parentRegistry)
   {
      this.yoTime = yoTime;

      registry = new YoVariableRegistry("ArmJointController" + robotSide.getCamelCaseNameForMiddleOfExpression());

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

      RigidBody chest = fullRobotModel.getChest();
      RigidBody hand = fullRobotModel.getHand(robotSide);

      InverseDynamicsJoint[] joints = ScrewTools.createJointPath(chest, hand);
      this.oneDoFJoints = ScrewTools.filterJoints(joints, RevoluteJoint.class);

      int orderOfPolynomial = 6;
      for (OneDoFJoint joint : oneDoFJoints)
      {
         YoPolynomial yoPolynomial = new YoPolynomial(joint.getName() + robotSide.getCamelCaseNameForMiddleOfExpression(), orderOfPolynomial, registry);
         polynomialLinkedHashMap.put(joint, yoPolynomial);

         PDController pdController = new PDController(kpAllArmJoints, kdAllArmJoints, joint.getName() + robotSide.getCamelCaseNameForMiddleOfExpression(),
                                        registry);
         pdControllerLinkedHashMap.put(joint, pdController);
      }
   }

   private void updateDerivativeGain()
   {
      kdAllArmJoints.set(GainCalculator.computeDerivativeGain(kpAllArmJoints.getDoubleValue(), zetaAllArmJoints.getDoubleValue()));
   }


   public void initializeForMove(LinkedHashMap<OneDoFJoint, Double> desiredJointPositions)
   {
      checkIfJointsMatch(desiredJointPositions);

      for (OneDoFJoint joint : oneDoFJoints)
      {
         YoPolynomial yoPolynomial = polynomialLinkedHashMap.get(joint);
         double startTime = yoTime.getDoubleValue();
         double endTime = startTime + moveTimeArmJoint.getDoubleValue();
         double desiredEndPosition = desiredJointPositions.get(joint);
         double desiredEndVelocity = 0.0;

         yoPolynomial.setCubic(startTime, endTime, joint.getQ(), joint.getQd(), desiredEndPosition, desiredEndVelocity);
      }
   }

   private LinkedHashMap<OneDoFJoint, Double> getDesiredJointAccelerations()
   {
      updateDerivativeGain();

      LinkedHashMap<OneDoFJoint, Double> desiredJointAccelerations = new LinkedHashMap<OneDoFJoint, Double>();

      double currentTime = yoTime.getDoubleValue();
      for (OneDoFJoint joint : oneDoFJoints)
      {
         YoPolynomial yoPolynomial = polynomialLinkedHashMap.get(joint);
         yoPolynomial.compute(currentTime);
         double desiredPosition = yoPolynomial.getPosition();
         double desiredVelocity = yoPolynomial.getVelocity();
         double feedforwardAcceleration = yoPolynomial.getAcceleration();

         double currentPosition = joint.getQ();
         double currentVelocity = joint.getQd();

         PDController pdController = pdControllerLinkedHashMap.get(joint);
         double desiredAccleration = feedforwardAcceleration + pdController.compute(currentPosition, desiredPosition, currentVelocity, desiredVelocity);

         desiredJointAccelerations.put(joint, desiredAccleration);
      }

      return desiredJointAccelerations;
   }

   private void checkIfJointsMatch(LinkedHashMap<OneDoFJoint, Double> desiredJointPositions)
   {
      for (OneDoFJoint joint : oneDoFJoints)
      {
         if (!desiredJointPositions.containsKey(joint))
            throw new RuntimeException(joint.getName() + " not found in desiredJointPositions");
      }
   }
}
