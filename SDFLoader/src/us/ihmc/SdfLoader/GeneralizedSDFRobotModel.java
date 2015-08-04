package us.ihmc.SdfLoader;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map.Entry;

import us.ihmc.SdfLoader.xmlDescription.SDFJoint;
import us.ihmc.SdfLoader.xmlDescription.SDFLink;
import us.ihmc.SdfLoader.xmlDescription.SDFModel;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.simulationconstructionset.graphics.GraphicsObjectsHolder;
import us.ihmc.robotics.sensors.ContactSensorType;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class GeneralizedSDFRobotModel implements GraphicsObjectsHolder
{
   private final String name;
   private final List<String> resourceDirectories;
   private final ArrayList<SDFLinkHolder> rootLinks = new ArrayList<SDFLinkHolder>();
   private final RigidBodyTransform transformToRoot;
   private final LinkedHashMap<String, SDFJointHolder> joints = new LinkedHashMap<String, SDFJointHolder>();
   private final LinkedHashMap<String, SDFLinkHolder> links = new LinkedHashMap<String, SDFLinkHolder>();
   
   
   public GeneralizedSDFRobotModel(String name, SDFModel model, List<String> resourceDirectories)
   {
      this.name = name;
      this.resourceDirectories = resourceDirectories;
      List<SDFLink> sdfLinks = model.getLinks();
      List<SDFJoint> sdfJoints = model.getJoints();
      
      

      // Populate maps
      for (SDFLink sdfLink : sdfLinks)
      {
         links.put(SDFConversionsHelper.sanitizeJointName(sdfLink.getName()), new SDFLinkHolder(sdfLink));
      }
      
      if(sdfJoints != null)
      {
         for (SDFJoint sdfJoint : sdfJoints)
         {
            String parent = SDFConversionsHelper.sanitizeJointName(sdfJoint.getParent());
            String child = SDFConversionsHelper.sanitizeJointName(sdfJoint.getChild());
            try
            {
               joints.put(SDFConversionsHelper.sanitizeJointName(sdfJoint.getName()), new SDFJointHolder(sdfJoint, links.get(parent), links.get(child)));
            }
            catch (IOException e)
            {
               System.err.println(e);
            }
         }
      }

      // Calculate transformations between joints
      for (Entry<String, SDFJointHolder> joint : joints.entrySet())
      {
         joint.getValue().calculateTransformToParentJoint();
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
   
   public RigidBodyTransform getTransformToRoot()
   {
      return transformToRoot;
   }
   
   public String getName()
   {
      return name;
   }
   
   public SDFJointHolder getJointHolder(String name)
   {
      return joints.get(name);
   }
   
   public List<String> getResourceDirectories()
   {
      return resourceDirectories;
   }
   
   public Graphics3DObject getCollisionObject(String name)
   {
      for(SDFLinkHolder linkHolder : rootLinks)
      {
         if(linkHolder.getName().equals(name))
         {
            return new SDFGraphics3DObject(linkHolder.getCollisions(), resourceDirectories);
         }
      }
      
      return new SDFGraphics3DObject(joints.get(name).getChildLinkHolder().getCollisions(), resourceDirectories);
   }

   public Graphics3DObject getGraphicsObject(String name)
   {
      
      for(SDFLinkHolder linkHolder : rootLinks)
      {
         if(linkHolder.getName().equals(name))
         {
            return new SDFGraphics3DObject(linkHolder.getVisuals(), resourceDirectories);
         }
      }
      
      SDFJointHolder joint = joints.get(name);
      RigidBodyTransform visualTransform = new RigidBodyTransform();
      visualTransform.setRotation(joint.getLinkRotation());
      return new SDFGraphics3DObject(joint.getChildLinkHolder().getVisuals(), resourceDirectories, visualTransform);
   }

   public void addForceSensor(String sensorName, String parentJointName, RigidBodyTransform transformToParentJoint)
   {
      SDFForceSensor sdfForceSensor = new SDFForceSensor(sensorName, transformToParentJoint);
      joints.get(parentJointName).addForceSensor(sdfForceSensor);
   }
   
   public void addContactSensor(String sensorName, String parentJointName, ContactSensorType type)
   {
      SDFContactSensor sdfContactSensor = new SDFContactSensor(sensorName, parentJointName, type);
      joints.get(parentJointName).addContactSensor(sdfContactSensor);
   }
}
