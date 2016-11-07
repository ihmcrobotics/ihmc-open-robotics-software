package us.ihmc.atlas.initialSetup;

import javax.vecmath.Vector3d;

import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.SquaredUpDRCRobotInitialSetup;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class SquaredUpVRCQual1SteppingStones extends SquaredUpDRCRobotInitialSetup implements DRCRobotInitialSetup<HumanoidFloatingRootJointRobot>
{
   private final Vector3d additionalOffset = new Vector3d(17.0,8.0,0.0);
   private final double yaw = 0.0;

   private Vector3d newOffset = null;

   public SquaredUpVRCQual1SteppingStones(Vector3d additionalOffset, double yaw)
   {
      this(0.0);
   }

   public SquaredUpVRCQual1SteppingStones(double groundHeight)
   {
      super(groundHeight);
   }

   public void initializeRobot(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
   {
      super.initializeRobot(robot, jointMap);

      if (newOffset == null)
      {
         newOffset = new Vector3d();
         super.getOffset(newOffset);
         newOffset.add(additionalOffset);
      }

      super.setOffset(newOffset);
      robot.setPositionInWorld(newOffset);
      robot.setOrientation(yaw, 0.0, 0.0);
   }
}
