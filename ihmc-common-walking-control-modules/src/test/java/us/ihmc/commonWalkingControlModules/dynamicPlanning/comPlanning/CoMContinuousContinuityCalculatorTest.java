package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.robotics.Assert.assertEquals;

public class CoMContinuousContinuityCalculatorTest
{
   private static final double epsilon = 1e-7;

   @Test
   public void testContinuity()
   {
      double omega = 3.0;
      YoDouble omegaProvider = new YoDouble("omega", new YoRegistry("test"));
      omegaProvider.set(omega);
      CoMContinuousContinuityCalculator calculator = new CoMContinuousContinuityCalculator(9.81, omegaProvider, new YoRegistry("test"));

      double height = 9.81 / MathTools.square(omega);

      FramePoint3D finalDesiredDCM = new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 0.5, height);
      FramePoint3D initialCoM = new FramePoint3D();
      FramePoint3D midpoint = new FramePoint3D();
      FrameVector3D initialCoMVelocity = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.25, 0.25, 0.3);
      initialCoM.setZ(height);
      midpoint.interpolate(initialCoM, finalDesiredDCM, 0.5);

      calculator.setFinalICPToAchieve(finalDesiredDCM);
      calculator.setInitialCoMPosition(initialCoM);
      calculator.setInitialCoMVelocity(initialCoMVelocity);

      List<ContactStateProvider> contacts = new ArrayList<>();
      SettableContactStateProvider contact1 = new SettableContactStateProvider();
      SettableContactStateProvider contact2 = new SettableContactStateProvider();

      double firstDuration = 0.25;
      double secondDuration = 0.5;
      initialCoM.subZ(height);
      midpoint.subZ(height);
      finalDesiredDCM.subZ(height);

      contact1.setStartCopPosition(initialCoM);
      contact1.setEndCopPosition(midpoint);
      contact2.setStartCopPosition(midpoint);
      contact2.setEndCopPosition(finalDesiredDCM);
      contact1.getTimeInterval().setInterval(0.0, firstDuration);
      contact2.getTimeInterval().setInterval(firstDuration, firstDuration + secondDuration);

      initialCoM.addZ(height);
      midpoint.addZ(height);
      finalDesiredDCM.addZ(height);

      contacts.add(contact1);
      contacts.add(contact2);

      calculator.solve(contacts);

      // First row is com position constraint
      int row = 0;
      assertEquals(CoMTrajectoryPlannerTools.getCoMPositionFirstCoefficientTimeFunction(omega, 0.0),
                   calculator.coefficientMultipliersSparse.get(row, 0),
                   epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getCoMPositionSecondCoefficientTimeFunction(omega, 0.0),
                   calculator.coefficientMultipliersSparse.get(row, 1),
                   epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getCoMPositionThirdCoefficientTimeFunction(0.0), calculator.coefficientMultipliersSparse.get(row, 2), epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getCoMPositionFourthCoefficientTimeFunction(0.0), calculator.coefficientMultipliersSparse.get(row, 3), epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getCoMPositionFifthCoefficientTimeFunction(0.0), calculator.coefficientMultipliersSparse.get(row, 4), epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getCoMPositionSixthCoefficientTimeFunction(), calculator.coefficientMultipliersSparse.get(row, 5), epsilon);

      assertEquals(initialCoM.getX(), calculator.xEquivalents.get(row, 0), epsilon);
      assertEquals(initialCoM.getY(), calculator.yEquivalents.get(row, 0), epsilon);
      assertEquals(initialCoM.getZ(), calculator.zEquivalents.get(row, 0), epsilon);

      // set the initial com velocity constraint
      row = 1;
      assertEquals(CoMTrajectoryPlannerTools.getCoMVelocityFirstCoefficientTimeFunction(omega, 0.0),
                   calculator.coefficientMultipliersSparse.get(row, 0),
                   epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getCoMVelocitySecondCoefficientTimeFunction(omega, 0.0),
                   calculator.coefficientMultipliersSparse.get(row, 1),
                   epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getCoMVelocityThirdCoefficientTimeFunction(0.0), calculator.coefficientMultipliersSparse.get(row, 2), epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getCoMVelocityFourthCoefficientTimeFunction(0.0), calculator.coefficientMultipliersSparse.get(row, 3), epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getCoMVelocityFifthCoefficientTimeFunction(), calculator.coefficientMultipliersSparse.get(row, 4), epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getCoMVelocitySixthCoefficientTimeFunction(), calculator.coefficientMultipliersSparse.get(row, 5), epsilon);

      assertEquals(initialCoMVelocity.getX(), calculator.xEquivalents.get(row, 0), epsilon);
      assertEquals(initialCoMVelocity.getY(), calculator.yEquivalents.get(row, 0), epsilon);
      assertEquals(initialCoMVelocity.getZ(), calculator.zEquivalents.get(row, 0), epsilon);

      // set the initial VRP position constraint
      row = 2;
      assertEquals(CoMTrajectoryPlannerTools.getVRPPositionFirstCoefficientTimeFunction(), calculator.coefficientMultipliersSparse.get(row, 0), epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getVRPPositionSecondCoefficientTimeFunction(), calculator.coefficientMultipliersSparse.get(row, 1), epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getVRPPositionThirdCoefficientTimeFunction(omega, 0.0),
                   calculator.coefficientMultipliersSparse.get(row, 2),
                   epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getVRPPositionFourthCoefficientTimeFunction(omega, 0.0),
                   calculator.coefficientMultipliersSparse.get(row, 3),
                   epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getVRPPositionFifthCoefficientTimeFunction(0.0), calculator.coefficientMultipliersSparse.get(row, 4), epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getVRPPositionSixthCoefficientTimeFunction(), calculator.coefficientMultipliersSparse.get(row, 5), epsilon);

      assertEquals(contact1.getCopStartPosition().getX(), calculator.xEquivalents.get(row, 0), epsilon);
      assertEquals(contact1.getCopStartPosition().getY(), calculator.yEquivalents.get(row, 0), epsilon);
      assertEquals(contact1.getCopStartPosition().getZ() + height, calculator.zEquivalents.get(row, 0), epsilon);

      // set the VRP velocity at the beginning being equivalent to the end of the segment
      row = 3;
      assertEquals(
            CoMTrajectoryPlannerTools.getVRPVelocityFirstCoefficientTimeFunction() - CoMTrajectoryPlannerTools.getVRPVelocityFirstCoefficientTimeFunction(),
            calculator.coefficientMultipliersSparse.get(row, 0),
            epsilon);
      assertEquals(
            CoMTrajectoryPlannerTools.getVRPVelocitySecondCoefficientTimeFunction() - CoMTrajectoryPlannerTools.getVRPVelocitySecondCoefficientTimeFunction(),
            calculator.coefficientMultipliersSparse.get(row, 1),
            epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getVRPVelocityThirdCoefficientTimeFunction(omega, 0.0)
                   - CoMTrajectoryPlannerTools.getVRPVelocityThirdCoefficientTimeFunction(omega, firstDuration),
                   calculator.coefficientMultipliersSparse.get(row, 2),
                   epsilon);
      assertEquals(
            CoMTrajectoryPlannerTools.getVRPVelocityFourthCoefficientTimeFunction(0.0) - CoMTrajectoryPlannerTools.getVRPVelocityFourthCoefficientTimeFunction(
                  firstDuration), calculator.coefficientMultipliersSparse.get(row, 3), epsilon);
      assertEquals(
            CoMTrajectoryPlannerTools.getVRPVelocityFifthCoefficientTimeFunction() - CoMTrajectoryPlannerTools.getVRPVelocityFifthCoefficientTimeFunction(),
            calculator.coefficientMultipliersSparse.get(row, 4),
            epsilon);
      assertEquals(
            CoMTrajectoryPlannerTools.getVRPVelocitySixthCoefficientTimeFunction() - CoMTrajectoryPlannerTools.getVRPVelocitySixthCoefficientTimeFunction(),
            calculator.coefficientMultipliersSparse.get(row, 5),
            epsilon);

      assertEquals(0.0, calculator.xEquivalents.get(row, 0), epsilon);
      assertEquals(0.0, calculator.yEquivalents.get(row, 0), epsilon);
      assertEquals(0.0, calculator.zEquivalents.get(row, 0), epsilon);

      // set Com position continuity constraint
      row = 4;
      assertEquals(CoMTrajectoryPlannerTools.getCoMPositionFirstCoefficientTimeFunction(omega, firstDuration),
                   calculator.coefficientMultipliersSparse.get(row, 0),
                   epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getCoMPositionSecondCoefficientTimeFunction(omega, firstDuration),
                   calculator.coefficientMultipliersSparse.get(row, 1),
                   epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getCoMPositionThirdCoefficientTimeFunction(firstDuration),
                   calculator.coefficientMultipliersSparse.get(row, 2),
                   epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getCoMPositionFourthCoefficientTimeFunction(firstDuration),
                   calculator.coefficientMultipliersSparse.get(row, 3),
                   epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getCoMPositionFifthCoefficientTimeFunction(firstDuration),
                   calculator.coefficientMultipliersSparse.get(row, 4),
                   epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getCoMPositionSixthCoefficientTimeFunction(), calculator.coefficientMultipliersSparse.get(row, 5), epsilon);

      assertEquals(-CoMTrajectoryPlannerTools.getCoMPositionFirstCoefficientTimeFunction(omega, 0.0),
                   calculator.coefficientMultipliersSparse.get(row, 6),
                   epsilon);
      assertEquals(-CoMTrajectoryPlannerTools.getCoMPositionSecondCoefficientTimeFunction(omega, 0.0),
                   calculator.coefficientMultipliersSparse.get(row, 7),
                   epsilon);
      assertEquals(-CoMTrajectoryPlannerTools.getCoMPositionThirdCoefficientTimeFunction(0.0), calculator.coefficientMultipliersSparse.get(row, 8), epsilon);
      assertEquals(-CoMTrajectoryPlannerTools.getCoMPositionFourthCoefficientTimeFunction(0.0), calculator.coefficientMultipliersSparse.get(row, 9), epsilon);
      assertEquals(-CoMTrajectoryPlannerTools.getCoMPositionFifthCoefficientTimeFunction(0.0), calculator.coefficientMultipliersSparse.get(row, 10), epsilon);
      assertEquals(-CoMTrajectoryPlannerTools.getCoMPositionSixthCoefficientTimeFunction(), calculator.coefficientMultipliersSparse.get(row, 11), epsilon);

      assertEquals(0.0, calculator.xEquivalents.get(row, 0), epsilon);
      assertEquals(0.0, calculator.yEquivalents.get(row, 0), epsilon);
      assertEquals(0.0, calculator.zEquivalents.get(row, 0), epsilon);

      // set Com velocity continuity constraint
      row = 5;
      assertEquals(CoMTrajectoryPlannerTools.getCoMVelocityFirstCoefficientTimeFunction(omega, firstDuration),
                   calculator.coefficientMultipliersSparse.get(row, 0),
                   epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getCoMVelocitySecondCoefficientTimeFunction(omega, firstDuration),
                   calculator.coefficientMultipliersSparse.get(row, 1),
                   epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getCoMVelocityThirdCoefficientTimeFunction(firstDuration),
                   calculator.coefficientMultipliersSparse.get(row, 2),
                   epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getCoMVelocityFourthCoefficientTimeFunction(firstDuration),
                   calculator.coefficientMultipliersSparse.get(row, 3),
                   epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getCoMVelocityFifthCoefficientTimeFunction(), calculator.coefficientMultipliersSparse.get(row, 4), epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getCoMVelocitySixthCoefficientTimeFunction(), calculator.coefficientMultipliersSparse.get(row, 5), epsilon);

      assertEquals(-CoMTrajectoryPlannerTools.getCoMVelocityFirstCoefficientTimeFunction(omega, 0.0),
                   calculator.coefficientMultipliersSparse.get(row, 6),
                   epsilon);
      assertEquals(-CoMTrajectoryPlannerTools.getCoMVelocitySecondCoefficientTimeFunction(omega, 0.0),
                   calculator.coefficientMultipliersSparse.get(row, 7),
                   epsilon);
      assertEquals(-CoMTrajectoryPlannerTools.getCoMVelocityThirdCoefficientTimeFunction(0.0), calculator.coefficientMultipliersSparse.get(row, 8), epsilon);
      assertEquals(-CoMTrajectoryPlannerTools.getCoMVelocityFourthCoefficientTimeFunction(0.0), calculator.coefficientMultipliersSparse.get(row, 9), epsilon);
      assertEquals(-CoMTrajectoryPlannerTools.getCoMVelocityFifthCoefficientTimeFunction(), calculator.coefficientMultipliersSparse.get(row, 10), epsilon);
      assertEquals(-CoMTrajectoryPlannerTools.getCoMVelocitySixthCoefficientTimeFunction(), calculator.coefficientMultipliersSparse.get(row, 11), epsilon);

      assertEquals(0.0, calculator.xEquivalents.get(row, 0), epsilon);
      assertEquals(0.0, calculator.yEquivalents.get(row, 0), epsilon);
      assertEquals(0.0, calculator.zEquivalents.get(row, 0), epsilon);

      // set vrp position continuity constraint
      row = 6;
      assertEquals(CoMTrajectoryPlannerTools.getVRPPositionFirstCoefficientTimeFunction(), calculator.coefficientMultipliersSparse.get(row, 0), epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getVRPPositionSecondCoefficientTimeFunction(), calculator.coefficientMultipliersSparse.get(row, 1), epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getVRPPositionThirdCoefficientTimeFunction(omega, firstDuration),
                   calculator.coefficientMultipliersSparse.get(row, 2),
                   epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getVRPPositionFourthCoefficientTimeFunction(omega, firstDuration),
                   calculator.coefficientMultipliersSparse.get(row, 3),
                   epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getVRPPositionFifthCoefficientTimeFunction(firstDuration),
                   calculator.coefficientMultipliersSparse.get(row, 4),
                   epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getVRPPositionSixthCoefficientTimeFunction(), calculator.coefficientMultipliersSparse.get(row, 5), epsilon);

      assertEquals(-CoMTrajectoryPlannerTools.getVRPPositionFirstCoefficientTimeFunction(), calculator.coefficientMultipliersSparse.get(row, 6), epsilon);
      assertEquals(-CoMTrajectoryPlannerTools.getVRPPositionSecondCoefficientTimeFunction(), calculator.coefficientMultipliersSparse.get(row, 7), epsilon);
      assertEquals(-CoMTrajectoryPlannerTools.getVRPPositionThirdCoefficientTimeFunction(omega, 0.0),
                   calculator.coefficientMultipliersSparse.get(row, 8),
                   epsilon);
      assertEquals(-CoMTrajectoryPlannerTools.getVRPPositionFourthCoefficientTimeFunction(omega, 0.0),
                   calculator.coefficientMultipliersSparse.get(row, 9),
                   epsilon);
      assertEquals(-CoMTrajectoryPlannerTools.getVRPPositionFifthCoefficientTimeFunction(0.0), calculator.coefficientMultipliersSparse.get(row, 10), epsilon);
      assertEquals(-CoMTrajectoryPlannerTools.getVRPPositionSixthCoefficientTimeFunction(), calculator.coefficientMultipliersSparse.get(row, 11), epsilon);

      assertEquals(0.0, calculator.xEquivalents.get(row, 0), epsilon);
      assertEquals(0.0, calculator.yEquivalents.get(row, 0), epsilon);
      assertEquals(0.0, calculator.zEquivalents.get(row, 0), epsilon);

      // TODO set the initial implicit VRP velocity constraints

      // Constrain final VRP position
      row = 9;
      assertEquals(CoMTrajectoryPlannerTools.getVRPPositionFirstCoefficientTimeFunction(), calculator.coefficientMultipliersSparse.get(row, 6), epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getVRPPositionSecondCoefficientTimeFunction(), calculator.coefficientMultipliersSparse.get(row, 7), epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getVRPPositionThirdCoefficientTimeFunction(omega, secondDuration),
                   calculator.coefficientMultipliersSparse.get(row, 8),
                   epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getVRPPositionFourthCoefficientTimeFunction(omega, secondDuration),
                   calculator.coefficientMultipliersSparse.get(row, 9),
                   epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getVRPPositionFifthCoefficientTimeFunction(secondDuration),
                   calculator.coefficientMultipliersSparse.get(row, 10),
                   epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getVRPPositionSixthCoefficientTimeFunction(), calculator.coefficientMultipliersSparse.get(row, 11), epsilon);

      assertEquals(finalDesiredDCM.getX(), calculator.xEquivalents.get(row, 0), epsilon);
      assertEquals(finalDesiredDCM.getY(), calculator.yEquivalents.get(row, 0), epsilon);
      assertEquals(finalDesiredDCM.getZ(), calculator.zEquivalents.get(row, 0), epsilon);

      // Constrain VRP velocity at beginning of second segment to end of second segment
      row = 10;
      assertEquals(
            CoMTrajectoryPlannerTools.getVRPVelocityFirstCoefficientTimeFunction() - CoMTrajectoryPlannerTools.getVRPVelocityFirstCoefficientTimeFunction(),
            calculator.coefficientMultipliersSparse.get(row, 6),
            epsilon);
      assertEquals(
            CoMTrajectoryPlannerTools.getVRPVelocitySecondCoefficientTimeFunction() - CoMTrajectoryPlannerTools.getVRPVelocitySecondCoefficientTimeFunction(),
            calculator.coefficientMultipliersSparse.get(row, 7),
            epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getVRPVelocityThirdCoefficientTimeFunction(omega, 0.0)
                   - CoMTrajectoryPlannerTools.getVRPVelocityThirdCoefficientTimeFunction(omega, secondDuration),
                   calculator.coefficientMultipliersSparse.get(row, 8),
                   epsilon);
      assertEquals(
            CoMTrajectoryPlannerTools.getVRPVelocityFourthCoefficientTimeFunction(0.0) - CoMTrajectoryPlannerTools.getVRPVelocityFourthCoefficientTimeFunction(
                  secondDuration), calculator.coefficientMultipliersSparse.get(row, 9), epsilon);
      assertEquals(
            CoMTrajectoryPlannerTools.getVRPVelocityFifthCoefficientTimeFunction() - CoMTrajectoryPlannerTools.getVRPVelocityFifthCoefficientTimeFunction(),
            calculator.coefficientMultipliersSparse.get(row, 10),
            epsilon);
      assertEquals(
            CoMTrajectoryPlannerTools.getVRPVelocitySixthCoefficientTimeFunction() - CoMTrajectoryPlannerTools.getVRPVelocitySixthCoefficientTimeFunction(),
            calculator.coefficientMultipliersSparse.get(row, 11),
            epsilon);

      assertEquals(0.0, calculator.xEquivalents.get(row, 0), epsilon);
      assertEquals(0.0, calculator.yEquivalents.get(row, 0), epsilon);
      assertEquals(0.0, calculator.zEquivalents.get(row, 0), epsilon);

      // Constrain final DCM position
      row = 11;
      assertEquals(CoMTrajectoryPlannerTools.getDCMPositionFirstCoefficientTimeFunction(omega, secondDuration),
                   calculator.coefficientMultipliersSparse.get(row, 6),
                   epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getDCMPositionSecondCoefficientTimeFunction(), calculator.coefficientMultipliersSparse.get(row, 7), epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getDCMPositionThirdCoefficientTimeFunction(omega, secondDuration),
                   calculator.coefficientMultipliersSparse.get(row, 8),
                   epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getDCMPositionFourthCoefficientTimeFunction(omega, secondDuration),
                   calculator.coefficientMultipliersSparse.get(row, 9),
                   epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getDCMPositionFifthCoefficientTimeFunction(omega, secondDuration),
                   calculator.coefficientMultipliersSparse.get(row, 10),
                   epsilon);
      assertEquals(CoMTrajectoryPlannerTools.getDCMPositionSixthCoefficientTimeFunction(), calculator.coefficientMultipliersSparse.get(row, 11), epsilon);

      assertEquals(finalDesiredDCM.getX(), calculator.xEquivalents.get(row, 0), epsilon);
      assertEquals(finalDesiredDCM.getY(), calculator.yEquivalents.get(row, 0), epsilon);
      assertEquals(finalDesiredDCM.getZ(), calculator.zEquivalents.get(row, 0), epsilon);

      // since the VRP is a linear function, the second and third coefficients should be zero
      assertEquals(0.0, calculator.xCoefficientVector.get(2, 0), epsilon);
      assertEquals(0.0, calculator.xCoefficientVector.get(3, 0), epsilon);

      assertEquals(0.0, calculator.yCoefficientVector.get(2, 0), epsilon);
      assertEquals(0.0, calculator.yCoefficientVector.get(3, 0), epsilon);

      assertEquals(0.0, calculator.zCoefficientVector.get(2, 0), epsilon);
      assertEquals(0.0, calculator.zCoefficientVector.get(3, 0), epsilon);

      assertEquals(0.0, calculator.xCoefficientVector.get(8, 0), epsilon);
      assertEquals(0.0, calculator.xCoefficientVector.get(9, 0), epsilon);

      assertEquals(0.0, calculator.yCoefficientVector.get(8, 0), epsilon);
      assertEquals(0.0, calculator.yCoefficientVector.get(9, 0), epsilon);

      assertEquals(0.0, calculator.zCoefficientVector.get(8, 0), epsilon);
      assertEquals(0.0, calculator.zCoefficientVector.get(9, 0), epsilon);
   }
}
