package us.ihmc.valkyrie.parameters;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.FootContactPoints;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

public class ValkyrieContactPointParameters extends RobotContactPointParameters<RobotSide>
{
   private final HumanoidJointNameMap jointMap;

   public ValkyrieContactPointParameters(HumanoidJointNameMap jointMap, ValkyriePhysicalProperties physicalProperties,
                                         FootContactPoints<RobotSide> footContactPoints)
   {
      super(jointMap, physicalProperties.getFootWidth(), physicalProperties.getFootLength(), physicalProperties.getSoleToAnkleFrameTransforms());

      this.jointMap = jointMap;

      if (footContactPoints == null)
         createDefaultFootContactPoints();
      else
         createFootContactPoints(footContactPoints);
   }

   public void createAdditionalHandContactPoints()
   {
      for (RobotSide robotSide : RobotSide.values)
         createFistContactPoints(robotSide);
   }

   private void createFistContactPoints(RobotSide robotSide)
   {
      String wrist = jointMap.getJointBeforeHandName(robotSide);

      Vector3D knuckleIndex1 = new Vector3D(-0.02, robotSide.negateIfRightSide(0.115), 0.03);
      Vector3D knuckleMiddle1 = new Vector3D(-0.02, robotSide.negateIfRightSide(0.11), -0.015);
      Vector3D knucklePinky1 = new Vector3D(-0.02, robotSide.negateIfRightSide(0.105), -0.06);
      knuckleIndex1.scale(jointMap.getModelScale());
      knuckleMiddle1.scale(jointMap.getModelScale());
      knuckleIndex1.scale(jointMap.getModelScale());

      addSimulationContactPoint(wrist, knuckleIndex1);
      addSimulationContactPoint(wrist, knuckleMiddle1);
      addSimulationContactPoint(wrist, knucklePinky1);
   }
}
