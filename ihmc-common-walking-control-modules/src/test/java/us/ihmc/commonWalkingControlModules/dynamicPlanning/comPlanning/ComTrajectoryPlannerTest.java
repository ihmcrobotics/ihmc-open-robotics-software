package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.time.TimeInterval;

import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public class ComTrajectoryPlannerTest extends CoMTrajectoryPlannerInterfaceTest
{
   private static final double epsilon = 1e-4;

   protected CoMTrajectoryPlannerInterface createComTrajectoryPlanner()
   {
      CoMTrajectoryPlanner planner = new CoMTrajectoryPlanner(gravityZ, nominalHeight, registry);
      planner.setMaintainInitialCoMVelocityContinuity(false);
      return planner;
   }

   @Test
   public void testStartingInFlight()
   {
      CoMTrajectoryPlanner planner = (CoMTrajectoryPlanner) createComTrajectoryPlanner();

      SettableContactStateProvider firstContact = new SettableContactStateProvider();
      SettableContactStateProvider secondContact = new SettableContactStateProvider();

      double flightDuration = 0.25;
      FramePoint3D contactPosition = new FramePoint3D(ReferenceFrame.getWorldFrame(), 2.0, 0.0, 0.0);
      firstContact.setTimeInterval(new TimeInterval(0.0, flightDuration));
      firstContact.setContactState(ContactState.FLIGHT);
      secondContact.setTimeInterval(new TimeInterval(flightDuration, 1.25));
      secondContact.setStartCopPosition(contactPosition);
      secondContact.setEndCopPosition(contactPosition);

      contactSequence.add(firstContact);
      contactSequence.add(secondContact);

      FramePoint3D comPosition = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.05, 0.05, nominalHeight + 0.05);
      FrameVector3D comVelocity = new FrameVector3D();
      planner.setInitialCenterOfMassState(comPosition, comVelocity);

      planner.solveForTrajectory(contactSequence);
      planner.compute(0.0);
      checkPlannerDynamics(planner, omega.getDoubleValue());

      FrameVector3DReadOnly acceleration = planner.getDesiredCoMAcceleration();

      assertEquals(0.0, acceleration.getX(), epsilon);
      assertEquals(0.0, acceleration.getY(), epsilon);
      assertEquals(-gravityZ, acceleration.getZ(), epsilon);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals("Desired CoM is invalid.", comPosition, planner.getDesiredCoMPosition(), epsilon);

      FramePoint3D secondVRP = new FramePoint3D(firstContact.getCopStartPosition());
      FramePoint3D thirdVRP = new FramePoint3D(secondContact.getCopStartPosition());
      secondVRP.addZ(nominalHeight);
      thirdVRP.addZ(nominalHeight);

      Random random = new Random(1738L);
      for (int i = 0; i < 100; i++)
      {
         double time = RandomNumbers.nextDouble(random, 0.0, contactSequence.get(0).getTimeInterval().getDuration());
         planner.compute(time);
         checkPlannerDynamics(planner, omega.getDoubleValue());
      }

      FramePoint3D preTouchdownCoMPosition = new FramePoint3D();
      FramePoint3D preTouchdownDCMPosition = new FramePoint3D();
      FramePoint3D preTouchdownVRPPosition = new FramePoint3D();
      FramePoint3D preTouchdownCMPPosition = new FramePoint3D();
      FrameVector3D preTouchdownCoMVelocity = new FrameVector3D();
      FrameVector3D preTouchdownCoMAcceleration = new FrameVector3D();
      FrameVector3D preTouchdownDCMVelocity = new FrameVector3D();
      FramePoint3D postTouchdownCoMPosition = new FramePoint3D();
      FramePoint3D postTouchdownDCMPosition = new FramePoint3D();
      FramePoint3D postTouchdownVRPPosition = new FramePoint3D();
      FramePoint3D postTouchdownCMPPosition = new FramePoint3D();
      FrameVector3D postTouchdownCoMVelocity = new FrameVector3D();
      FrameVector3D postTouchdownCoMAcceleration = new FrameVector3D();
      FrameVector3D postTouchdownDCMVelocity = new FrameVector3D();
      planner.compute(0, flightDuration, preTouchdownCoMPosition, preTouchdownCoMVelocity, preTouchdownCoMAcceleration, preTouchdownDCMPosition,
                      preTouchdownDCMVelocity, preTouchdownVRPPosition, preTouchdownCMPPosition);
      planner.compute(1, 0.0, postTouchdownCoMPosition, postTouchdownCoMVelocity, postTouchdownCoMAcceleration, postTouchdownDCMPosition,
                      postTouchdownDCMVelocity, postTouchdownVRPPosition, postTouchdownCMPPosition);

      FramePoint3D assembledPreTouchdownCoMPosition = new FramePoint3D();
      FramePoint3D assembledPreTouchdownDCMPosition = new FramePoint3D();
      FramePoint3D assembledPreTouchdownVRPPosition = new FramePoint3D();
      FrameVector3D assembledPreTouchdownCoMVelocity = new FrameVector3D();
      FrameVector3D assembledPreTouchdownCoMAcceleration = new FrameVector3D();
      FramePoint3D assembledPostTouchdownCoMPosition = new FramePoint3D();
      FramePoint3D assembledPostTouchdownDCMPosition = new FramePoint3D();
      FramePoint3D assembledPostTouchdownVRPPosition = new FramePoint3D();
      FrameVector3D assembledPostTouchdownCoMVelocity = new FrameVector3D();
      FrameVector3D assembledPostTouchdownCoMAcceleration = new FrameVector3D();

      FramePoint3D firstCoefficient = new FramePoint3D();
      FramePoint3D secondCoefficient = new FramePoint3D();
      FramePoint3D thirdCoefficient = new FramePoint3D();
      FramePoint3D fourthCoefficient = new FramePoint3D();
      FramePoint3D fifthCoefficient = new FramePoint3D();
      FramePoint3D sixthCoefficient = new FramePoint3D();

      DMatrix xCoefficientVector = planner.xCoefficientVector;
      DMatrix yCoefficientVector = planner.yCoefficientVector;
      DMatrix zCoefficientVector = planner.zCoefficientVector;

      int startIndex = 0;
      firstCoefficient.setX(xCoefficientVector.get(startIndex, 0));
      firstCoefficient.setY(yCoefficientVector.get(startIndex, 0));
      firstCoefficient.setZ(zCoefficientVector.get(startIndex, 0));

      int secondCoefficientIndex = startIndex + 1;
      secondCoefficient.setX(xCoefficientVector.get(secondCoefficientIndex, 0));
      secondCoefficient.setY(yCoefficientVector.get(secondCoefficientIndex, 0));
      secondCoefficient.setZ(zCoefficientVector.get(secondCoefficientIndex, 0));

      int thirdCoefficientIndex = startIndex + 2;
      thirdCoefficient.setX(xCoefficientVector.get(thirdCoefficientIndex, 0));
      thirdCoefficient.setY(yCoefficientVector.get(thirdCoefficientIndex, 0));
      thirdCoefficient.setZ(zCoefficientVector.get(thirdCoefficientIndex, 0));

      int fourthCoefficientIndex = startIndex + 3;
      fourthCoefficient.setX(xCoefficientVector.get(fourthCoefficientIndex, 0));
      fourthCoefficient.setY(yCoefficientVector.get(fourthCoefficientIndex, 0));
      fourthCoefficient.setZ(zCoefficientVector.get(fourthCoefficientIndex, 0));

      int fifthCoefficientIndex = startIndex + 4;
      fifthCoefficient.setX(xCoefficientVector.get(fifthCoefficientIndex, 0));
      fifthCoefficient.setY(yCoefficientVector.get(fifthCoefficientIndex, 0));
      fifthCoefficient.setZ(zCoefficientVector.get(fifthCoefficientIndex, 0));

      int sixthCoefficientIndex = startIndex + 5;
      sixthCoefficient.setX(xCoefficientVector.get(sixthCoefficientIndex, 0));
      sixthCoefficient.setY(yCoefficientVector.get(sixthCoefficientIndex, 0));
      sixthCoefficient.setZ(zCoefficientVector.get(sixthCoefficientIndex, 0));

      CoMTrajectoryPlannerTools
            .constructDesiredCoMPosition(assembledPreTouchdownCoMPosition, firstCoefficient, secondCoefficient, thirdCoefficient, fourthCoefficient, fifthCoefficient, sixthCoefficient, flightDuration, omega.getDoubleValue());
      CoMTrajectoryPlannerTools.constructDesiredCoMVelocity(assembledPreTouchdownCoMVelocity, firstCoefficient, secondCoefficient, thirdCoefficient, fourthCoefficient, fifthCoefficient, sixthCoefficient, flightDuration, omega.getDoubleValue());
      CoMTrajectoryPlannerTools.constructDesiredCoMAcceleration(assembledPreTouchdownCoMAcceleration, firstCoefficient, secondCoefficient, thirdCoefficient, fourthCoefficient, fifthCoefficient, sixthCoefficient, flightDuration, omega.getDoubleValue());
      CoMTrajectoryPlannerTools.constructDesiredDCMPosition(assembledPreTouchdownDCMPosition, firstCoefficient, secondCoefficient, thirdCoefficient, fourthCoefficient, fifthCoefficient, sixthCoefficient, flightDuration, omega.getDoubleValue());
      CoMTrajectoryPlannerTools.constructDesiredVRPPosition(assembledPreTouchdownVRPPosition, firstCoefficient, secondCoefficient, thirdCoefficient, fourthCoefficient, fifthCoefficient, sixthCoefficient, flightDuration, omega.getDoubleValue());

      startIndex = 6;
      firstCoefficient.setX(xCoefficientVector.get(startIndex, 0));
      firstCoefficient.setY(yCoefficientVector.get(startIndex, 0));
      firstCoefficient.setZ(zCoefficientVector.get(startIndex, 0));

      secondCoefficientIndex = startIndex + 1;
      secondCoefficient.setX(xCoefficientVector.get(secondCoefficientIndex, 0));
      secondCoefficient.setY(yCoefficientVector.get(secondCoefficientIndex, 0));
      secondCoefficient.setZ(zCoefficientVector.get(secondCoefficientIndex, 0));

      thirdCoefficientIndex = startIndex + 2;
      thirdCoefficient.setX(xCoefficientVector.get(thirdCoefficientIndex, 0));
      thirdCoefficient.setY(yCoefficientVector.get(thirdCoefficientIndex, 0));
      thirdCoefficient.setZ(zCoefficientVector.get(thirdCoefficientIndex, 0));

      fourthCoefficientIndex = startIndex + 3;
      fourthCoefficient.setX(xCoefficientVector.get(fourthCoefficientIndex, 0));
      fourthCoefficient.setY(yCoefficientVector.get(fourthCoefficientIndex, 0));
      fourthCoefficient.setZ(zCoefficientVector.get(fourthCoefficientIndex, 0));

      fifthCoefficientIndex = startIndex + 4;
      fifthCoefficient.setX(xCoefficientVector.get(fifthCoefficientIndex, 0));
      fifthCoefficient.setY(yCoefficientVector.get(fifthCoefficientIndex, 0));
      fifthCoefficient.setZ(zCoefficientVector.get(fifthCoefficientIndex, 0));

      sixthCoefficientIndex = startIndex + 5;
      sixthCoefficient.setX(xCoefficientVector.get(sixthCoefficientIndex, 0));
      sixthCoefficient.setY(yCoefficientVector.get(sixthCoefficientIndex, 0));
      sixthCoefficient.setZ(zCoefficientVector.get(sixthCoefficientIndex, 0));

      CoMTrajectoryPlannerTools.constructDesiredCoMPosition(assembledPostTouchdownCoMPosition, firstCoefficient, secondCoefficient, thirdCoefficient, fourthCoefficient, fifthCoefficient, sixthCoefficient, 0.0, omega.getDoubleValue());
      CoMTrajectoryPlannerTools.constructDesiredCoMVelocity(assembledPostTouchdownCoMVelocity, firstCoefficient, secondCoefficient, thirdCoefficient, fourthCoefficient, fifthCoefficient, sixthCoefficient, 0.0, omega.getDoubleValue());
      CoMTrajectoryPlannerTools.constructDesiredCoMAcceleration(assembledPostTouchdownCoMAcceleration, firstCoefficient, secondCoefficient, thirdCoefficient, fourthCoefficient, fifthCoefficient, sixthCoefficient, 0.0, omega.getDoubleValue());
      CoMTrajectoryPlannerTools.constructDesiredDCMPosition(assembledPostTouchdownDCMPosition, firstCoefficient, secondCoefficient, thirdCoefficient, fourthCoefficient, fifthCoefficient, sixthCoefficient, 0.0, omega.getDoubleValue());
      CoMTrajectoryPlannerTools.constructDesiredVRPPosition(assembledPostTouchdownVRPPosition, firstCoefficient, secondCoefficient, thirdCoefficient, fourthCoefficient, fifthCoefficient, sixthCoefficient, 0.0, omega.getDoubleValue());

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(preTouchdownCoMPosition, postTouchdownCoMPosition, 1e-4);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(preTouchdownCoMPosition, assembledPostTouchdownCoMPosition, 1e-4);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(preTouchdownCoMPosition, assembledPreTouchdownCoMPosition, 1e-4);

      EuclidCoreTestTools.assertVector3DGeometricallyEquals(preTouchdownCoMVelocity, postTouchdownCoMVelocity, 1e-4);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(preTouchdownCoMVelocity, assembledPreTouchdownCoMVelocity, 1e-4);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(preTouchdownCoMVelocity, assembledPostTouchdownCoMVelocity, 1e-4);

      EuclidCoreTestTools.assertVector3DGeometricallyEquals(preTouchdownCoMAcceleration, assembledPreTouchdownCoMAcceleration, 1e-4);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(postTouchdownCoMAcceleration, assembledPostTouchdownCoMAcceleration, 1e-4);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(preTouchdownDCMPosition, postTouchdownDCMPosition, 1e-4);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(preTouchdownDCMPosition, assembledPreTouchdownDCMPosition, 1e-4);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(preTouchdownDCMPosition, assembledPostTouchdownDCMPosition, 1e-4);

//      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(preTouchdownVRPPosition, postTouchdownVRPPosition, 1e-4); // FIXME THIS SHOULD NOT HOLD
//      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(preTouchdownCMPPosition, postTouchdownCMPPosition, 1e-4); // FIXME THIS SHOULD NOT HOLD

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(postTouchdownVRPPosition, assembledPostTouchdownVRPPosition, 1e-4);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(preTouchdownVRPPosition, assembledPreTouchdownVRPPosition, 1e-4);

      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(contactPosition), new Point2D(postTouchdownCMPPosition), 1e-4);



      FramePoint3D finalPosition = new FramePoint3D(contactPosition);
      finalPosition.addZ(nominalHeight);

      planner.compute(1, 100.0, postTouchdownCoMPosition, postTouchdownCoMVelocity, postTouchdownCoMAcceleration, postTouchdownDCMPosition,
                      postTouchdownDCMVelocity, postTouchdownVRPPosition, postTouchdownCMPPosition);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(finalPosition, postTouchdownCoMPosition, 1e-4);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new FrameVector3D(), postTouchdownCoMVelocity, 1e-4);
//      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new FrameVector3D(), assembledPostTouchdownCoMAcceleration, 1e-4);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(finalPosition, postTouchdownDCMPosition, 1e-4);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(finalPosition, assembledPostTouchdownVRPPosition, 1e-4);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(contactPosition, postTouchdownCMPPosition, 1e-4);
   }

}
