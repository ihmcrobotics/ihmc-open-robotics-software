package us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule;

import org.junit.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieKinematicsPlanningToolboxControllerTest extends AvatarKinematicsPlanningToolboxControllerTest
{
   private final DRCRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);
   private final DRCRobotModel ghostRobotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return robotModel.getSimpleRobotName();
   }

   @Override
   public DRCRobotModel getGhostRobotModel()
   {
      return ghostRobotModel;
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 30000)
   public void testMessages() throws Exception
   {
      super.testMessages();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 30000)
   public void testRaiseUpHand() throws Exception
   {
      super.testRaiseUpHand();
   }
}
