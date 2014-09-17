package us.ihmc.SdfLoader;

import java.util.ArrayList;

import us.ihmc.utilities.math.geometry.Transform3d;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;

public class SDFModelVisual extends Graphics3DObject
{
   private final ArrayList<String> resourceDirectories;
   
   public SDFModelVisual(GeneralizedSDFRobotModel generalizedSDFRobotModel)
   {
      this(generalizedSDFRobotModel, false);
   }
   
   public SDFModelVisual(GeneralizedSDFRobotModel generalizedSDFRobotModel, boolean useCollisionMeshes)
   {
      resourceDirectories = generalizedSDFRobotModel.getResourceDirectories();
      ArrayList<SDFLinkHolder> rootLinks = generalizedSDFRobotModel.getRootLinks();
      
      Transform3d modelTransform = generalizedSDFRobotModel.getTransformToRoot();
      for(SDFLinkHolder link : rootLinks)
      {  
         recursivelyAddLinks(link, modelTransform, useCollisionMeshes);
      }
   }
   
   private void recursivelyAddLinks(SDFLinkHolder link, Transform3d modelTransform, boolean useCollisionMeshes)
   {
      if(link.getVisuals() != null)
      {
         
         Transform3d transformToModel = new Transform3d(modelTransform);
         transformToModel.mul(link.getTransformFromModelReferenceFrame());
         
         
         SDFGraphics3DObject sdfGraphics3DObject;
         if(useCollisionMeshes)
         {
            sdfGraphics3DObject = new SDFGraphics3DObject(link.getCollisions(), resourceDirectories, transformToModel);
         }
         else
         {
            sdfGraphics3DObject = new SDFGraphics3DObject(link.getVisuals(), resourceDirectories, transformToModel);
         }
         getGraphics3DInstructions().addAll(sdfGraphics3DObject.getGraphics3DInstructions());
         
      }
      
      for(SDFJointHolder joint: link.getChildren())
      {
         recursivelyAddLinks(joint.getChild(), modelTransform, useCollisionMeshes);
      }
   }
}
