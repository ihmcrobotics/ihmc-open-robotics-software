package us.ihmc.SdfLoader;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;

import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.xmlDescription.SDFJoint;
import us.ihmc.SdfLoader.xmlDescription.SDFLink;
import us.ihmc.SdfLoader.xmlDescription.SDFModel;

public class GeneralizedSDFRobotModel
{
   private final String name;
   private final String resourceDirectory;
   private final ArrayList<SDFLinkHolder> rootLinks = new ArrayList<SDFLinkHolder>();
   private final Vector3d rootOffset = new Vector3d();

   public GeneralizedSDFRobotModel(String name, SDFModel model, String resourceDirectory)
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
      for (SDFJoint sdfJoint : sdfJoints)
      {
         joints.put(sdfJoint.getName(), new SDFJointHolder(sdfJoint, links.get(sdfJoint.getParent()), links.get(sdfJoint.getChild())));
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
      
      SDFConversionsHelper.poseToTransform(model.getPose()).get(rootOffset);

      System.out.println(rootLinks.get(0).getChilderen());
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
   
   public Vector3d getRootOffset()
   {
      return rootOffset;
   }
   
   public String getName()
   {
      return name;
   }

   public String getResourceDirectory()
   {
      return resourceDirectory;
   }

}
