package us.ihmc.zulu;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.zulu.parameters.model.ZuluPhysicalProperties;
import us.ihmc.avatar.AvatarLiftOffAndTouchDownTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.yoVariables.variable.YoDouble;

import static org.junit.jupiter.api.Assertions.*;

public class ZuluLiftOffAndTouchDownTest
{
   private final double footLength = new ZuluPhysicalProperties().getFootLengthForControl();

   @Tag("humanoid-flat-ground")
   @Test
   public void testForwardStepRotated() throws SimulationExceededMaximumTimeException
   {
      DRCRobotModel robotModel = new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT)
      {
//         @Override
//         public WalkingControllerParameters getWalkingControllerParameters()
//         {
//            return new ZuluWalkingControllerParameters(getRobotVersion(), getTarget(), getJointMap(), getPhysicalProperties(), getContactPointParameters())
//            {
//               @Override
//               public double nominalHeightAboveAnkle()
//               {
//                  return 0.76;
//               }
//            };
//         }
      };
      SCS2AvatarTestingSimulation testHelper = AvatarLiftOffAndTouchDownTest.setupTest(robotModel, Math.toRadians(90.0));
      // We need to turn off the CoP threshold fraction here. If it's left on, contact isn't triggered until the CoP is far enough within the foot. This means
      // that the detection of contact is significantly delayed. It then misses the cotnact timing, and also ahs a tendency to "slap" the foot onto the ground,
      // rather than smoothly loading it.
      String namespace = HighLevelHumanoidControllerFactory.class.getSimpleName();
      ((YoDouble) testHelper.findVariable(namespace, "CoPThresholdFraction")).set(Double.NaN);

      double stepLength = 0.4;
      double startPitch = Math.toRadians(20.0);
      double finalPitch = Math.toRadians(-20.0);

      boolean success = AvatarLiftOffAndTouchDownTest.doStep(robotModel, testHelper, stepLength, startPitch, finalPitch, footLength);

      testHelper.finishTest();

      assertTrue(success, "A check failed. See console output.");
   }
}
