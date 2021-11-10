package us.ihmc.valkyrie;

import java.util.function.Consumer;

import us.ihmc.avatar.factory.RobotDefinitionTools;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.MomentOfInertiaDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.WrenchSensorDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
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
      inertia.setM00(0.000401606); // i_xx
      inertia.setM01(4.9927e-08); // i_xy
      inertia.setM02(1.0997e-05); // i_xz
      inertia.setM11(0.00208115); // i_yy
      inertia.setM12(-9.8165e-09); // i_yz
      inertia.setM22(0.00178402); // i_zz
   }
}
