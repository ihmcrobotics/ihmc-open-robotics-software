package us.ihmc.SdfLoader;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;

import javax.media.j3d.Transform3D;

import us.ihmc.SdfLoader.xmlDescription.SDFJoint;
import us.ihmc.SdfLoader.xmlDescription.SDFLink;
import us.ihmc.SdfLoader.xmlDescription.SDFModel;

public class GeneralizedSDFRobotModel
{
   private final String name;
   private final File resourceDirectory;
   private final ArrayList<SDFLinkHolder> rootLinks = new ArrayList<SDFLinkHolder>();
   private final Transform3D transformToRoot;

   public GeneralizedSDFRobotModel(String name, SDFModel model, File resourceDirectory)
   {
      this.name = name;
      this.resourceDirectory = resourceDirectory;
      
      List<SDFLink> sdfLinks = model.getLinks();
      List<SDFJoint> sdfJoints = model.getJoints();
      
      HashMap<String, SDFJointHolder> joints = new HashMap<String, SDFJointHolder>();
      HashMap<String, SDFLinkHolder> links = new HashMap<String, SDFLinkHolder>();

      // Populate maps
      for (SDFLink sdfLink : sdfLinks)
      {
         links.put(sdfLink.getName(), new SDFLinkHolder(sdfLink));
      }
      
      if(sdfJoints != null)
      {
         for (SDFJoint sdfJoint : sdfJoints)
         {
            joints.put(sdfJoint.getName(), new SDFJointHolder(sdfJoint, links.get(sdfJoint.getParent()), links.get(sdfJoint.getChild())));
         }
      }

      // Calculate transformations between joints
      for (Entry<String, SDFJointHolder> joint : joints.entrySet())
      {
         joint.getValue().calculateTransformToParent();
      }

      for (Entry<String, SDFLinkHolder> link : links.entrySet())
      {
         link.getValue().calculateCoMOffset();
      }

      findRootLinks(links);
      
      transformToRoot = SDFConversionsHelper.poseToTransform(model.getPose());

   }

   private void findRootLinks(HashMap<String, SDFLinkHolder> links)
   {
      for (Entry<String, SDFLinkHolder> linkEntry : links.entrySet())
      {
         SDFLinkHolder link = linkEntry.getValue();
         if (link.getJoint() == null)
         {
            rootLinks.add(link);
         }
      }
   }

   public ArrayList<SDFLinkHolder> getRootLinks()
   {
      return rootLinks;
   }
   
   public Transform3D getTransformToRoot()
   {
      return transformToRoot;
   }
   
   public String getName()
   {
      return name;
   }

   public File getResourceDirectory()
   {
      return resourceDirectory;
   }

}
