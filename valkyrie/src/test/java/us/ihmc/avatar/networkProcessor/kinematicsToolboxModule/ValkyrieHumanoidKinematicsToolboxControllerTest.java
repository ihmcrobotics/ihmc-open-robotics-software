package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieHumanoidKinematicsToolboxControllerTest extends HumanoidKinematicsToolboxControllerTest
{
   private final DRCRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS);
   private final DRCRobotModel ghostRobotModel = new ValkyrieRobotModel(RobotTarget.SCS);

   @Tag("humanoid-toolbox")
   @Override
   @Test // (timeout = 30000)
   public void testHoldBodyPose() throws Exception
   {
      super.testHoldBodyPose();
   }

   @Tag("humanoid-toolbox")
   @Override
   @Test // (timeout = 30000)
   public void testRandomHandPoses() throws Exception
   {
      super.testRandomHandPoses();
   }

   @Tag("humanoid-toolbox")
   @Override
   @Test // (timeout = 30000)
   public void testRandomHandPositions() throws Exception
   {
      super.testRandomHandPositions();
   }

   @Tag("humanoid-toolbox")
   @Override
   @Test // (timeout = 30000)
   public void testSingleSupport() throws Exception
   {
      super.testSingleSupport();
   }

   @Tag("humanoid-toolbox")
   @Override
   @Test
   public void testCenterOfMassConstraint() throws Exception
   {
      super.testCenterOfMassConstraint();
   }

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
}
