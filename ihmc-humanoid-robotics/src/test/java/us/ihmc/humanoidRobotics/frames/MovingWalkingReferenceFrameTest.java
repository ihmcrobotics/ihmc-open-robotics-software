package us.ihmc.humanoidRobotics.frames;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.screwTheory.MovingMidFootZUpGroundFrame;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.MovingZUpFrame;
import us.ihmc.robotics.screwTheory.NumericalMovingReferenceFrame;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculatorTest;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.SettableDoubleProvider;

public class MovingWalkingReferenceFrameTest
{

   @Test(timeout = 30000)
   public void testAgainstFiniteDifference()
   {
      Random random = new Random(23423L);

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      SettableDoubleProvider timeProvider = new SettableDoubleProvider();
      double updateDT = 1.0e-8;

      MovingReferenceFrame frameOne = createMovingReferenceFrame("frameOne", worldFrame, random, timeProvider);
      MovingReferenceFrame frameTwo = createMovingReferenceFrame("frameTwo", worldFrame, random, timeProvider);
      MovingReferenceFrame pelvisFrame = createMovingReferenceFrame("pelvisFrame", worldFrame, random, timeProvider);
      MovingZUpFrame frameOneZUp = new MovingZUpFrame(frameOne, frameOne.getName() + "ZUp");
      MovingZUpFrame frameTwoZUp = new MovingZUpFrame(frameTwo, frameTwo.getName() + "ZUp");
      MovingMidFootZUpGroundFrame midFootZUpGroundFrame = new MovingMidFootZUpGroundFrame("midFootZUp", frameOneZUp, frameTwoZUp);
      MovingWalkingReferenceFrame walkingReferenceFrame = new MovingWalkingReferenceFrame("walkingFrame", pelvisFrame, midFootZUpGroundFrame);

      NumericalMovingReferenceFrame frameOneFD = new NumericalMovingReferenceFrame("FD", frameOne, updateDT);
      NumericalMovingReferenceFrame frameTwoFD = new NumericalMovingReferenceFrame("FD", frameTwo, updateDT);
      NumericalMovingReferenceFrame frameOneZUpFD = new NumericalMovingReferenceFrame("FD", frameOneZUp, updateDT);
      NumericalMovingReferenceFrame frameTwoZUpFD = new NumericalMovingReferenceFrame("FD", frameTwoZUp, updateDT);
      NumericalMovingReferenceFrame midFootZUpGroundFrameFD = new NumericalMovingReferenceFrame("FD", midFootZUpGroundFrame, updateDT);
      NumericalMovingReferenceFrame walkingReferenceFrameFD = new NumericalMovingReferenceFrame("FD", walkingReferenceFrame, updateDT);

      List<ReferenceFrame> allFrames = new ArrayList<>();
      allFrames.add(frameOne);
      allFrames.add(frameTwo);
      allFrames.add(frameOneFD);
      allFrames.add(frameTwoFD);
      allFrames.add(pelvisFrame);
      allFrames.add(frameOneZUp);
      allFrames.add(frameTwoZUp);
      allFrames.add(midFootZUpGroundFrame);
      allFrames.add(walkingReferenceFrame);
      allFrames.add(frameOneZUpFD);
      allFrames.add(frameTwoZUpFD);
      allFrames.add(midFootZUpGroundFrameFD);
      allFrames.add(walkingReferenceFrameFD);

      double epsilon = 1.0e-5;

      for (int i = 0; i < 10000; i++)
      {
         timeProvider.add(updateDT);
         allFrames.forEach(ReferenceFrame::update);

         if (i == 0)
            continue;

         assertFrameTwistEquals(frameOneFD, frameOne, epsilon);
         assertFrameTwistEquals(frameTwoFD, frameTwo, epsilon);
         assertFrameTwistEquals(frameOneZUpFD, frameOneZUp, epsilon);
         assertFrameTwistEquals(frameTwoZUpFD, frameTwoZUp, epsilon);
         assertFrameTwistEquals(midFootZUpGroundFrameFD, midFootZUpGroundFrame, epsilon);
         assertFrameTwistEquals(walkingReferenceFrameFD, walkingReferenceFrame, epsilon);
      }
   }

   private static void assertFrameTwistEquals(MovingReferenceFrame expected, MovingReferenceFrame actual, double epsilon)
   {
      EuclidCoreTestTools.assertRigidBodyTransformEquals(expected.getTransformToRoot(), actual.getTransformToRoot(), 1.0e-12);

      Twist actualTwist = new Twist();
      Twist expectedTwist = new Twist();

      expected.getTwistOfFrame(expectedTwist);
      actual.getTwistOfFrame(actualTwist);

      expectedTwist.changeBodyFrameNoRelativeTwist(actual);
      expectedTwist.changeFrame(actual);

      TwistCalculatorTest.assertTwistEquals(expectedTwist, actualTwist, epsilon);
   }

   private static MovingReferenceFrame createMovingReferenceFrame(String name, ReferenceFrame parentFrame, Random random, DoubleProvider timeProvider)
   {
      return new MovingReferenceFrame(name, parentFrame)
      {
         private final Vector3D linearAmplitude = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
         private final Vector3D linearFrequency = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
         private final Vector3D linearPhase = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());

         private final Vector3D yawPitchRollAmplitude = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
         private final Vector3D yawPitchRollFrequency = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
         private final Vector3D yawPitchRollPhase = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());

         private final Point3D position = new Point3D();
         private final double[] yawPitchRoll = new double[3];

         private final FrameVector3D linearVelocity = new FrameVector3D();
         private final double[] yawPitchRollDot = new double[3];
         private final FrameVector3D angularVelocity = new FrameVector3D();

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            linearVelocity.setToZero(parentFrame);

            for (int i = 0; i < 3; i++)
            {
               double amp = linearAmplitude.getElement(i);
               double f = linearFrequency.getElement(i);
               double pulse = 2.0 * Math.PI * f;
               double t = timeProvider.getValue();
               double phi = linearPhase.getElement(i);
               double value = amp * Math.sin(pulse * t + phi);
               double valueDot = amp * pulse * Math.cos(pulse * t + phi);
               position.setElement(i, value);
               linearVelocity.setElement(i, valueDot);
            }

            for (int i = 0; i < 3; i++)
            {
               double amp = yawPitchRollAmplitude.getElement(i);
               double f = yawPitchRollFrequency.getElement(i);
               double pulse = 2.0 * Math.PI * f;
               double t = timeProvider.getValue();
               double phi = yawPitchRollPhase.getElement(i);
               double value = amp * Math.sin(pulse * t + phi);
               double valueDot = amp * pulse * Math.cos(pulse * t + phi);
               yawPitchRoll[i] = value;
               yawPitchRollDot[i] = valueDot;
            }

            RotationMatrix yaw = new RotationMatrix();
            RotationMatrix pitch = new RotationMatrix();
            yaw.setToYawMatrix(yawPitchRoll[0]);
            pitch.setToPitchMatrix(yawPitchRoll[1]);

            angularVelocity.setToZero(parentFrame);
            angularVelocity.add(yawPitchRollDot[2], 0.0, 0.0);
            pitch.transform(angularVelocity);
            angularVelocity.add(0.0, yawPitchRollDot[1], 0.0);
            yaw.transform(angularVelocity);
            angularVelocity.add(0.0, 0.0, yawPitchRollDot[0]);

            transformToParent.setTranslation(position);
            transformToParent.setRotationYawPitchRoll(yawPitchRoll);
         }

         @Override
         protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
         {
            twistRelativeToParentToPack.setToZero(this, parentFrame, this);
            linearVelocity.changeFrame(this);
            twistRelativeToParentToPack.setLinearPart(linearVelocity);
            angularVelocity.changeFrame(this);
            twistRelativeToParentToPack.setAngularPart(angularVelocity);
         }
      };
   }
}
