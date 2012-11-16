package us.ihmc.SdfLoader;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;

import com.sun.xml.internal.bind.v2.runtime.unmarshaller.XsiNilLoader.Array;

import us.ihmc.SdfLoader.xmlDescription.SDFJoint;
import us.ihmc.SdfLoader.xmlDescription.SDFLink;

public class GeneralizedSDFRobotModel
{
   private final String name;
   private final String resourceDirectory;
   private final ArrayList<SDFLinkHolder> rootLinks = new ArrayList<SDFLinkHolder>();

   public GeneralizedSDFRobotModel(String name, List<SDFJoint> sdfJoints, List<SDFLink> sdfLinks, String resourceDirectory)
   {
      this.name = name;
      this.resourceDirectory = resourceDirectory;
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
   
   public String getName()
   {
      return name;
   }

   public String getResourceDirectory()
   {
      return resourceDirectory;
   }

}
