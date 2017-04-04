package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.Map;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.euclid.tuple3D.Vector3D;

public abstract class MomentumOptimizationSettings implements ControllerCoreOptimizationSettings
{
   public abstract Vector3D getLinearMomentumWeight();

   public abstract Vector3D getHighLinearMomentumWeightForRecovery();

   public abstract Vector3D getAngularMomentumWeight();

   /**
    * The map returned contains all optimization weights for jointspace objectives. The key of the map
    * is the joint name as defined in the robot joint map. If a joint is not contained in the map,
    * jointspace control is not supported for that joint.
    *
    * @return map containing jointspace QP weights by joint name
    */
   public abstract TObjectDoubleHashMap<String> getJointspaceWeights();

   /**
    * The map returned contains all optimization weights for user desired acceleration objectives. The
    * key of the map is the joint name as defined in the robot joint map. If a joint is not contained
    * in the map, user desired acceleration commands are not supported for that joint.
    *
    * @return map containing user desired acceleration QP weights by joint name
    */
   public abstract TObjectDoubleHashMap<String> getUserModeWeights();

   /**
    * The map returned contains all optimization weights for taskspace orientation objectives. The key
    * of the map is the rigid body name as defined in the robot joint map. If a rigid body is not
    * contained in the map, taskspace orientation objectives are not supported for that body.
    *
    * @return map containing taskspace orientation QP weights by rigid body name
    */
   public abstract Map<String, Vector3D> getTaskspaceAngularWeights();

   /**
    * The map returned contains all optimization weights for taskspace position objectives. The key
    * of the map is the rigid body name as defined in the robot joint map. If a rigid body is not
    * contained in the map, taskspace position objectives are not supported for that body.
    *
    * @return map containing taskspace position QP weights by rigid body name
    */
   public abstract Map<String, Vector3D> getTaskspaceLinearWeights();

   // TODO: nuke this once the rigid body manager stuff is done and these weights are contained in the maps above
   public abstract double getHandJointspaceWeight();
   public abstract double getHeadJointspaceWeight();
   public abstract double getChestUserModeWeight();
   public abstract double getHandUserModeWeight();
   public abstract double getHeadUserModeWeight();
   public abstract Vector3D getHeadAngularWeight();
   public abstract Vector3D getChestAngularWeight();
   public abstract Vector3D getPelvisAngularWeight();
   public abstract Vector3D getHandAngularTaskspaceWeight();
   public abstract Vector3D getHandLinearTaskspaceWeight();

   // TODO: figure out how to handle these with the maps:
   public abstract Vector3D getDefaultLinearFootWeight();
   public abstract Vector3D getDefaultAngularFootWeight();
   public abstract Vector3D getHighLinearFootWeight();
   public abstract Vector3D getHighAngularFootWeight();
}
