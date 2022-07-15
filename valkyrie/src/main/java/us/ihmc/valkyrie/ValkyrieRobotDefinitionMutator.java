package us.ihmc.valkyrie;

import java.util.List;
import java.util.function.Consumer;

import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.robot.CameraSensorDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.MomentOfInertiaDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.WrenchSensorDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.simulationToolkit.RobotDefinitionTools;
import us.ihmc.valkyrie.parameters.ValkyrieSensorInformation;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlController;

public class ValkyrieRobotDefinitionMutator implements Consumer<RobotDefinition>
{
   private HumanoidJointNameMap jointMap;
   private boolean useOBJGraphics;

   public ValkyrieRobotDefinitionMutator(HumanoidJointNameMap jointMap, boolean useOBJGraphics)
   {
      this.jointMap = jointMap;
      this.useOBJGraphics = useOBJGraphics;
   }

   @Override
   public void accept(RobotDefinition robotDefinition)
   {
      for (String forceSensorName : ValkyrieSensorInformation.forceSensorNames)
      {
         JointDefinition jointDefinition = robotDefinition.getJointDefinition(forceSensorName);
         jointDefinition.addSensorDefinition(new WrenchSensorDefinition(forceSensorName, ValkyrieSensorInformation.getForceSensorTransform(forceSensorName)));
      }

      RobotDefinitionTools.setDefaultMaterial(robotDefinition);

      if (useOBJGraphics)
      {
         for (RigidBodyDefinition body : robotDefinition.getAllRigidBodies())
         {
            for (VisualDefinition visual : body.getVisualDefinitions())
            {
               if (visual.getGeometryDefinition() instanceof ModelFileGeometryDefinition)
               {
                  ModelFileGeometryDefinition geometry = (ModelFileGeometryDefinition) visual.getGeometryDefinition();
                  geometry.setFileName(geometry.getFileName().replace(".dae", ".obj"));
               }
            }
         }
      }

      for (RigidBodyDefinition body : robotDefinition.getAllRigidBodies())
      {
         if (body.getParentJoint() != null)
         {
            List<CameraSensorDefinition> cameras = body.getParentJoint().getSensorDefinitions(CameraSensorDefinition.class);
            if (cameras != null && !cameras.isEmpty())
            {
               for (CameraSensorDefinition camera : cameras)
               {
                  camera.setClipFar(100000.0); // TODO Allows to view the entire scene, not sure if that's what we want
                  camera.setUpdatePeriod(1000 / 25); // 25Hz // TODO Weird this is not present in the description.
               }
            }
         }
      }

      modifyHokuyoInertia(robotDefinition.getRigidBodyDefinition("hokuyo_link"));
      modifyChestMass(robotDefinition.getRigidBodyDefinition(jointMap.getChestName()));

      if (jointMap.getModelScale() != 1.0)
         RobotDefinitionTools.scaleRobotDefinition(robotDefinition,
                                                   jointMap.getModelScale(),
                                                   jointMap.getMassScalePower(),
                                                   j -> !j.getName().contains("hokuyo"));
   }

   private void modifyChestMass(RigidBodyDefinition chestDefinition)
   {
      if (chestDefinition == null)
         return;

      if (ValkyrieRosControlController.HAS_LIGHTER_BACKPACK)
         chestDefinition.setMass(chestDefinition.getMass() - 8.6);
   }

   private void modifyHokuyoInertia(RigidBodyDefinition hokuyoDefinition)
   {
      if (hokuyoDefinition == null)
         return;

      MomentOfInertiaDefinition inertia = hokuyoDefinition.getMomentOfInertia();
      inertia.setM00(4.223114008735548E-4); // i_xx
      inertia.setM01(-5.064933200111187E-8); // i_xy
      inertia.setM02(1.682757619662739E-4); // i_xz
      inertia.setM11(0.002081150000050006); // i_yy
      inertia.setM12(-2.437817069606712E-9); // i_yz
      inertia.setM22(0.0017633145990764395); // i_zz
   }
}
