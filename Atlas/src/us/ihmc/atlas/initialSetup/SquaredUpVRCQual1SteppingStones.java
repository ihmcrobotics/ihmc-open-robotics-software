package us.ihmc.atlas.initialSetup;

import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.SquaredUpDRCRobotInitialSetup;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class SquaredUpVRCQual1SteppingStones extends SquaredUpDRCRobotInitialSetup implements DRCRobotInitialSetup<HumanoidFloatingRootJointRobot>
{
   private final Vector3D additionalOffset = new Vector3D(17.0,8.0,0.0);
   private final double yaw = 0.0;

   private Vector3D newOffset = null;

   public SquaredUpVRCQual1SteppingStones(Vector3D additionalOffset, double yaw)
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
         newOffset = new Vector3D();
         super.getOffset(newOffset);
         newOffset.add(additionalOffset);
      }

      super.setOffset(newOffset);
      robot.setPositionInWorld(newOffset);
      robot.setOrientation(yaw, 0.0, 0.0);
   }
}
