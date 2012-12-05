package us.ihmc.SdfLoader;

import java.util.List;

import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.ArmJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LimbName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.NeckJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.RobotSpecificJointNames;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.SpineJointName;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.Pair;

public interface SDFJointNameMap extends RobotSpecificJointNames
{
   public enum JointRole
      {
         LEG, ARM, SPINE, NECK
      }
   
   public JointRole getJointRole(String jointName);
   public Pair<RobotSide, LegJointName> getLegJointName(String jointName);
   public Pair<RobotSide, ArmJointName> getArmJointName(String jointName);
   public NeckJointName getNeckJointName(String jointName);
   public SpineJointName getSpineJointName(String jointName);
   public Pair<RobotSide, LimbName> getLimbName(String limbName);
   
   public String getPelvisName();
   public String getChestName();
   public String getHeadName();
   
   public String getJointBeforeFootName(RobotSide robotSide);
   public double getAnkleHeight();
   public List<Vector3d> getGroundContactPointOffset(RobotSide robotSide);
}
