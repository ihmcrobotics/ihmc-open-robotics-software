package us.ihmc.valkyrie;

import java.util.List;

import us.ihmc.modelFileLoaders.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.modelFileLoaders.SdfLoader.SDFContactSensor;
import us.ihmc.modelFileLoaders.SdfLoader.SDFDescriptionMutator;
import us.ihmc.modelFileLoaders.SdfLoader.SDFForceSensor;
import us.ihmc.modelFileLoaders.SdfLoader.SDFJointHolder;
import us.ihmc.modelFileLoaders.SdfLoader.SDFLinkHolder;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFGeometry;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFSensor;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFVisual;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlController;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;

public class ValkyrieSDFDescriptionMutator implements SDFDescriptionMutator
{
   private HumanoidJointNameMap jointMap;
   private boolean useOBJGraphics;

   public ValkyrieSDFDescriptionMutator(HumanoidJointNameMap jointMap, boolean useOBJGraphics)
   {
      this.jointMap = jointMap;
      this.useOBJGraphics = useOBJGraphics;
   }

   @Override
   public void mutateJointForModel(GeneralizedSDFRobotModel model, SDFJointHolder jointHolder)
   {
      if (jointMap.getModelName().equals(model.getName()))
      {

      }
   }

   @Override
   public void mutateLinkForModel(GeneralizedSDFRobotModel model, SDFLinkHolder linkHolder)
   {
      if (jointMap.getModelName().equals(model.getName()))
      {
         if (useOBJGraphics)
         {
            List<SDFVisual> visuals = linkHolder.getVisuals();
            if (visuals != null)
            {
               for (SDFVisual sdfVisual : visuals)
               {
                  SDFGeometry geometry = sdfVisual.getGeometry();
                  if (geometry == null)
                     continue;

                  SDFGeometry.Mesh mesh = geometry.getMesh();
                  if (mesh == null)
                     continue;

                  String meshUri = mesh.getUri();
                  if (meshUri.contains("meshes"))
                  {
                     String replacedURI = meshUri.replace(".dae", ".obj");
                     mesh.setUri(replacedURI);
                  }
               }
            }
         }

         switch (linkHolder.getName())
         {
            case "hokuyo_link":
               modifyHokuyoInertia(linkHolder);
               break;
            case "torso":
               modifyChestMass(linkHolder);
               break;
            default:
               break;
         }
      }
   }

   private void modifyChestMass(SDFLinkHolder chestSDFLink)
   {
      if (ValkyrieRosControlController.HAS_LIGHTER_BACKPACK)
         chestSDFLink.setMass(chestSDFLink.getMass() - 8.6);
   }

   @Override
   public void mutateSensorForModel(GeneralizedSDFRobotModel model, SDFSensor sensor)
   {
      if (jointMap.getModelName().equals(model.getName()))
      {

      }
   }

   @Override
   public void mutateForceSensorForModel(GeneralizedSDFRobotModel model, SDFForceSensor forceSensor)
   {
      if (jointMap.getModelName().equals(model.getName()))
      {

      }
   }

   @Override
   public void mutateContactSensorForModel(GeneralizedSDFRobotModel model, SDFContactSensor contactSensor)
   {
      if (jointMap.getModelName().equals(model.getName()))
      {

      }
   }

   @Override
   public void mutateModelWithAdditions(GeneralizedSDFRobotModel model)
   {
      if (jointMap.getModelName().equals(model.getName()))
      {

      }
   }

   private void modifyHokuyoInertia(SDFLinkHolder linkHolder)
   {
      linkHolder.getInertia().setM00(0.000401606); // i_xx
      linkHolder.getInertia().setM01(4.9927e-08); // i_xy
      linkHolder.getInertia().setM02(1.0997e-05); // i_xz
      linkHolder.getInertia().setM11(0.00208115); // i_yy
      linkHolder.getInertia().setM12(-9.8165e-09); // i_yz
      linkHolder.getInertia().setM22(0.00178402); // i_zz
   }
}
