package us.ihmc.atlas.utilities.kinematics;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.NumericalInverseKinematicsCalculatorWithRobotTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionsettools.bambooTools.BambooTools;

public class AtlasNumericalInverseKinematicsCalculatorWithRobotTest extends NumericalInverseKinematicsCalculatorWithRobotTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.SCS, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0, categoriesOverride = IntegrationCategory.IN_DEVELOPMENT)
   @Test(timeout = 120000)
   public void testTroublesomeCaseOne()
   {
      Random random = new Random(1234L);

      FramePoint handEndEffectorPositionFK = new FramePoint(ReferenceFrame.getWorldFrame(), -0.10094331252710122, 0.702327488375448, 0.8842020873774364);
      FrameOrientation handEndEffectorOrientationFK = new FrameOrientation(ReferenceFrame.getWorldFrame(), 0.5948015455279927, -0.24418998175404205, 0.11864264766705496, 0.7566414582898712);

      InitialGuessForTests initialGuessForTests = InitialGuessForTests.MIDRANGE;
      boolean updateListenersEachStep = true;
      double errorThreshold = 0.01;
      boolean success = testAPose(random, handEndEffectorPositionFK, handEndEffectorOrientationFK, initialGuessForTests, errorThreshold , updateListenersEachStep);
      assertTrue("testAPose returned false", success);
   }
}
