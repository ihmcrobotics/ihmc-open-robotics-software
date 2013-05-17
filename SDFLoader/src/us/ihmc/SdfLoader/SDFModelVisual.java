package us.ihmc.SdfLoader;

import java.util.ArrayList;

import javax.media.j3d.Transform3D;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;

public class SDFModelVisual extends Graphics3DObject
{
   private final ArrayList<String> resourceDirectories;
   
   public SDFModelVisual(GeneralizedSDFRobotModel generalizedSDFRobotModel)
   {
      resourceDirectories = generalizedSDFRobotModel.getResourceDirectories();
      ArrayList<SDFLinkHolder> rootLinks = generalizedSDFRobotModel.getRootLinks();
      
      Transform3D modelTransform = new Transform3D(generalizedSDFRobotModel.getTransformToRoot());
      for(SDFLinkHolder link : rootLinks)
      {  
         recursivelyAddLinks(link, modelTransform);
      }
   }
   
   private void recursivelyAddLinks(SDFLinkHolder link, Transform3D modelTransform)
   {
      if(link.getVisuals() != null)
      {

//         Vector3d translation = new Vector3d();
//         modelTransform.get(translation);
//         translation.setX(translation.getX() + 0.25);
//         translation.setZ(1.0);
//         modelTransform.setTranslation(translation);
         
         
         Transform3D transformToModel = new Transform3D(modelTransform);
         transformToModel.mul(link.getTransformFromModelReferenceFrame());
         
         SDFGraphics3DObject sdfGraphics3DObject = new SDFGraphics3DObject(link.getVisuals(), resourceDirectories, transformToModel);
         getGraphics3DInstructions().addAll(sdfGraphics3DObject.getGraphics3DInstructions());
         
      }
      
      for(SDFJointHolder joint: link.getChildren())
      {
         recursivelyAddLinks(joint.getChild(), modelTransform);
      }
   }
}
