package us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.manipulation.states.fingerToroidManipulation;

import javax.vecmath.Matrix3d;

import us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.manipulation.ManipulableToroid;
import us.ihmc.robotics.geometry.CylindricalCoordinatesCalculator;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.geometry.RotationFunctions;
import us.ihmc.robotics.trajectories.providers.SE3ConfigurationProvider;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;


/**
 * @author twan
 *         Date: 5/21/13
 */
public class ValveConfigurationProvider implements SE3ConfigurationProvider
{
   private final ManipulableToroid manipulableToroid;
   private final RobotSide robotSide;
   private final CylindricalCoordinatesCalculator cylindricalCoordinatesCalculator = new CylindricalCoordinatesCalculator();

   private final DoubleYoVariable offsetAngle;
   private final DoubleYoVariable offsetZ;
   private final DoubleYoVariable outwardRotation;
   private final DoubleYoVariable pitchRotation;
   private final DoubleYoVariable extraRadius;

   public ValveConfigurationProvider(ManipulableToroid manipulableToroid, RobotSide robotSide, DoubleYoVariable offsetAngle,
                                     DoubleYoVariable offsetZ,
                                     DoubleYoVariable outwardRotation, DoubleYoVariable pitchRotation, DoubleYoVariable extraRadius)
   {
      this.manipulableToroid = manipulableToroid;
      this.robotSide = robotSide;
      this.offsetAngle = offsetAngle;
      this.offsetZ = offsetZ;
      this.outwardRotation = outwardRotation;
      this.pitchRotation = pitchRotation;
      this.extraRadius = extraRadius;
   }

   public void get(FrameOrientation orientationToPack)
   {
      FramePose handPose = getHandPose();
      handPose.getOrientationIncludingFrame(orientationToPack);
   }

   public void get(FramePoint positionToPack)
   {
      FramePose handPose = getHandPose();
      handPose.getPositionIncludingFrame(positionToPack);
   }

   private FramePose getHandPose()
   {
      ReferenceFrame frame = manipulableToroid.getStaticToroidReferenceFrame();

      double q = manipulableToroid.getQ();
      double qAdjustedForSymmetry = q % (Math.PI / 2.0);

      double radiansFromYAxis = robotSide.negateIfRightSide(Math.PI / 2.0) + offsetAngle.getDoubleValue() + qAdjustedForSymmetry;
      double twelveOClockAngle = Math.PI / 2.0;
      double radiansFromXAxis = twelveOClockAngle + radiansFromYAxis;

      double radius = manipulableToroid.getToroidRadius() + extraRadius.getDoubleValue();

      FramePoint position = new FramePoint();
      cylindricalCoordinatesCalculator.getPosition(position, frame, radiansFromXAxis, radius, offsetZ.getDoubleValue());

      // difference between torus frame (facing robot) and finger position control frames:
      Matrix3d rotation = new Matrix3d();
      rotation.setColumn(0, 0.0, 0.0, -1.0);
      rotation.setColumn(1, -1.0, 0.0, 0.0);
      rotation.setColumn(2, 0.0, 1.0, 0.0);

      Matrix3d postRotation = new Matrix3d();
      RotationFunctions.setYawPitchRoll(postRotation, robotSide.negateIfRightSide(outwardRotation.getDoubleValue()), 0.0, 0.0);

      rotation.mul(postRotation);

      FrameOrientation orientation = new FrameOrientation(frame, rotation);

      FramePose pose = new FramePose(position, orientation);
      return pose;
   }
}
