package us.ihmc.zulu.parameters.controller;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;

public class ZuluMomentumOptimizationSettings extends MomentumOptimizationSettings
{
   private final int nContactableBodies;

   private final List<GroupParameter<Double>> jointspaceWeights = new ArrayList<>();

   private final List<GroupParameter<Double>> userModeWeights = new ArrayList<>();

   private final List<GroupParameter<Vector3DReadOnly>> taskspaceAngularWeights = new ArrayList<>();

   private final List<GroupParameter<Vector3DReadOnly>> taskspaceLinearWeights = new ArrayList<>();

   public ZuluMomentumOptimizationSettings(HumanoidJointNameMap jointMap, int numberOfContactableBodies)
   {
      List<String> neckNames = jointMap.getNeckJointNamesAsStrings();
      List<String> spineNames = jointMap.getSpineJointNamesAsStrings();
      List<String> armNames = jointMap.getArmJointNamesAsStrings();
      List<String> legNames = jointMap.getLegJointNamesAsStrings();

      List<String> allUserModeJoints = new ArrayList<>();
      allUserModeJoints.addAll(neckNames);
      allUserModeJoints.addAll(spineNames);
      allUserModeJoints.addAll(armNames);
      allUserModeJoints.addAll(legNames);

      Vector3D handAngularWeight;
      if (jointMap.getArmJointNames().length == 4)
      { // For 4-dof arm, only control Z orientation of hand and place at higher weight
         handAngularWeight = new Vector3D(0.0, 0.0, 0.5);
      }
      else
      { // Full 7dof arm mode
         handAngularWeight = new Vector3D(1.0, 1.0, 1.0);
      }

      // We don't really use this feature, bundling all joints together.
      double jointUserModeWeight = 50.0;
      userModeWeights.add(new GroupParameter<>("allJoints", jointUserModeWeight, allUserModeJoints));
      // TODO Needs tune up.
      double armJointspaceWeight = ZuluWalkingControllerParameters.RESPONSIVE_STREAMING_MODE ? 3.0 : 1.0;

      // TODO Needs tune up.
      double spineJointspaceWeight = 10.0;
      jointspaceWeights.add(new GroupParameter<>("Spine", spineJointspaceWeight, jointMap.getSpineJointNamesAsStrings()));
      jointspaceWeights.add(new GroupParameter<>("Arms", armJointspaceWeight, jointMap.getArmJointNamesAsStrings()));
      // TODO Needs tune up.
      double legJointspaceWeight = 1.0;
      jointspaceWeights.add(new GroupParameter<>("Legs", legJointspaceWeight, jointMap.getLegJointNamesAsStrings()));
      double neckJointspaceWeight = 50.0;
      jointspaceWeights.add(new GroupParameter<>("Neck", neckJointspaceWeight, jointMap.getNeckJointNamesAsStrings()));

      // TODO Needs tune up.
      Vector3D chestAngularWeight = new Vector3D(30.0, 30.0, 5.0);
      taskspaceAngularWeights.add(new GroupParameter<>("Chest", chestAngularWeight, Collections.singletonList(jointMap.getChestName())));
      // TODO Needs tune up.
      Vector3D headAngularWeight = new Vector3D(500.0, 500.0, 500.0);
      taskspaceAngularWeights.add(new GroupParameter<>("Head", headAngularWeight, Collections.singletonList(jointMap.getHeadName())));

      Vector3D pelvisAngularWeight = new Vector3D(10.0, 10.0, 3.0);
      taskspaceAngularWeights.add(new GroupParameter<>("Pelvis", pelvisAngularWeight, Collections.singletonList(jointMap.getPelvisName())));
      // TODO Needs tune up.
      Vector3D pelvisLinearWeight = new Vector3D(5.0, 5.0, 30.0);
      taskspaceLinearWeights.add(new GroupParameter<>("Pelvis", pelvisLinearWeight, Collections.singletonList(jointMap.getPelvisName())));

      List<String> handNames = jointMap.getHandNames();
      List<String> footNames = jointMap.getFootNames();
      taskspaceAngularWeights.add(new GroupParameter<>("Hand", handAngularWeight, handNames));
      // TODO Needs tune up.
      Vector3D handLinearWeight = new Vector3D(5.0, 5.0, 5.0);
      taskspaceLinearWeights.add(new GroupParameter<>("Hand", handLinearWeight, handNames));
      // TODO Needs tune up.
      Vector3D footAngularWeight = new Vector3D(0.5, 0.5, 0.5);
      taskspaceAngularWeights.add(new GroupParameter<>("Foot", footAngularWeight, footNames));
      // TODO Needs tune up.
      Vector3D footLinearWeight = new Vector3D(30.0, 30.0, 30.0);
      taskspaceLinearWeights.add(new GroupParameter<>("Foot", footLinearWeight, footNames));

      nContactableBodies = numberOfContactableBodies;
   }

   /** @inheritDoc */
   @Override
   public int getNumberOfContactableBodies()
   {
      return nContactableBodies;
   }

   /** @inheritDoc */
   @Override
   public List<GroupParameter<Double>> getJointspaceWeights()
   {
      return jointspaceWeights;
   }

   /** @inheritDoc */
   @Override
   public List<GroupParameter<Double>> getUserModeWeights()
   {
      return userModeWeights;
   }

   /** @inheritDoc */
   @Override
   public List<GroupParameter<Vector3DReadOnly>> getTaskspaceAngularWeights()
   {
      return taskspaceAngularWeights;
   }

   /** @inheritDoc */
   @Override
   public List<GroupParameter<Vector3DReadOnly>> getTaskspaceLinearWeights()
   {
      return taskspaceLinearWeights;
   }
}
