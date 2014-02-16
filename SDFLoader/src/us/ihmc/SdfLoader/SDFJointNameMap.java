package us.ihmc.SdfLoader;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

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
         LEG, ARM, SPINE, NECK;
      }
   public String getModelName();

   public String getLidarJointName();

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
   public List<Pair<String,Vector3d>> getJointNameGroundContactPointMap();
   public ArrayList<Vector3d> getFootGroundContactPointsForController();
   
   public boolean enableTorqueVelocityLimits();
   
   public Set<String> getLastSimulatedJoints();

   public String[] getIMUSensorsToUse();

   public String getLidarSensorName();

   public String getLeftCameraName();

}
