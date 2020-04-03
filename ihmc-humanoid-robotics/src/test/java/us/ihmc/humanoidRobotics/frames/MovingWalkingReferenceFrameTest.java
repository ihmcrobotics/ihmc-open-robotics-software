package us.ihmc.humanoidRobotics.frames;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.tools.MecanoTestTools;
import us.ihmc.robotics.screwTheory.MovingMidFootZUpGroundFrame;
import us.ihmc.robotics.screwTheory.MovingZUpFrame;
import us.ihmc.robotics.screwTheory.NumericalMovingReferenceFrame;
import us.ihmc.robotics.trajectories.providers.SettableDoubleProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class MovingWalkingReferenceFrameTest
{
   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
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

      expectedTwist.setBodyFrame(actual);
      expectedTwist.changeFrame(actual);

      MecanoTestTools.assertTwistEquals(expectedTwist, actualTwist, epsilon);
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
         private final YawPitchRoll yawPitchRoll = new YawPitchRoll();

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
               yawPitchRoll.setElement(i, value);
               yawPitchRollDot[i] = valueDot;
            }

            RotationMatrix yaw = new RotationMatrix();
            RotationMatrix pitch = new RotationMatrix();
            yaw.setToYawOrientation(yawPitchRoll.getYaw());
            pitch.setToPitchOrientation(yawPitchRoll.getPitch());

            angularVelocity.setToZero(parentFrame);
            angularVelocity.add(yawPitchRollDot[2], 0.0, 0.0);
            pitch.transform(angularVelocity);
            angularVelocity.add(0.0, yawPitchRollDot[1], 0.0);
            yaw.transform(angularVelocity);
            angularVelocity.add(0.0, 0.0, yawPitchRollDot[0]);

            transformToParent.getTranslation().set(position);
            transformToParent.getRotation().set(yawPitchRoll);
         }

         @Override
         protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
         {
            twistRelativeToParentToPack.setToZero(this, parentFrame, this);
            linearVelocity.changeFrame(this);
            twistRelativeToParentToPack.getLinearPart().set(linearVelocity);
            angularVelocity.changeFrame(this);
            twistRelativeToParentToPack.getAngularPart().set(angularVelocity);
         }
      };
   }
}
