package us.ihmc.atlas.utilities.kinematics;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.NumericalInverseKinematicsCalculatorWithRobotTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;

public class AtlasNumericalInverseKinematicsCalculatorWithRobotTest extends NumericalInverseKinematicsCalculatorWithRobotTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Disabled
   @Test
   public void testTroublesomeCaseOne()
   {
      Random random = new Random(1234L);

      FramePoint3D handEndEffectorPositionFK = new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.10094331252710122, 0.702327488375448, 0.8842020873774364);
      FrameQuaternion handEndEffectorOrientationFK = new FrameQuaternion(ReferenceFrame.getWorldFrame(), 0.5948015455279927, -0.24418998175404205, 0.11864264766705496, 0.7566414582898712);

      InitialGuessForTests initialGuessForTests = InitialGuessForTests.MIDRANGE;
      boolean updateListenersEachStep = true;
      double errorThreshold = 0.01;
      boolean success = testAPose(random, handEndEffectorPositionFK, handEndEffectorOrientationFK, initialGuessForTests, errorThreshold , updateListenersEachStep);
      assertTrue("testAPose returned false", success);
   }

   @Override
   @Test
   public void testRandomFeasibleRobotPoses()
   {
      super.testRandomFeasibleRobotPoses();
   }

   @Override
   @Test
   public void testSimpleCase()
   {
      super.testSimpleCase();
   }
}
