package us.ihmc.modelFileLoaders.SdfLoader;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map.Entry;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.modelFileLoaders.ModelFileLoaderConversionsHelper;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFJoint;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFLink;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFModel;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFSensor;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.robotics.robotDescription.GraphicsObjectsHolder;
import us.ihmc.robotics.sensors.ContactSensorType;

public class GeneralizedSDFRobotModel implements GraphicsObjectsHolder
{
   private final String name;
   private final List<String> resourceDirectories;
   private final ClassLoader resourceClassLoader;
   private final SDFDescriptionMutator descriptionMutator;
   private final ArrayList<SDFLinkHolder> rootLinks = new ArrayList<SDFLinkHolder>();
   private final RigidBodyTransform transformToRoot;
   private final LinkedHashMap<String, SDFJointHolder> joints = new LinkedHashMap<String, SDFJointHolder>();
   private final LinkedHashMap<String, SDFLinkHolder> links = new LinkedHashMap<String, SDFLinkHolder>();

   public GeneralizedSDFRobotModel(String name, SDFModel model, List<String> resourceDirectories, ClassLoader resourceClassLoader)
   {
      this(name, model, resourceDirectories, resourceClassLoader, null);
   }

   public GeneralizedSDFRobotModel(String name, SDFModel model, List<String> resourceDirectories, ClassLoader resourceClassLoader, SDFDescriptionMutator descriptionMutator)
   {
      this.name = name;
      this.resourceDirectories = resourceDirectories;
      this.resourceClassLoader = resourceClassLoader;
      this.descriptionMutator = descriptionMutator;
      List<SDFLink> sdfLinks = model.getLinks();
      List<SDFJoint> sdfJoints = model.getJoints();



      // Populate maps
      for (SDFLink sdfLink : sdfLinks)
      {
         SDFLinkHolder linkHolder = new SDFLinkHolder(sdfLink);
         if(this.descriptionMutator != null)
         {
            this.descriptionMutator.mutateLinkForModel(this, linkHolder);

            List<SDFSensor> sensors = linkHolder.getSensors();
            if (sensors != null)
            {
               for (SDFSensor sdfSensor : sensors)
               {
                  this.descriptionMutator.mutateSensorForModel(this, sdfSensor);
               }
            }
         }

         links.put(ModelFileLoaderConversionsHelper.sanitizeJointName(sdfLink.getName()), linkHolder);
      }

      if(sdfJoints != null)
      {
         for (SDFJoint sdfJoint : sdfJoints)
         {
            String parent = ModelFileLoaderConversionsHelper.sanitizeJointName(sdfJoint.getParent());
            String child = ModelFileLoaderConversionsHelper.sanitizeJointName(sdfJoint.getChild());
            try
            {
               SDFJointHolder jointHolder = new SDFJointHolder(sdfJoint, links.get(parent), links.get(child));
               if(this.descriptionMutator != null)
               {
                  this.descriptionMutator.mutateJointForModel(this, jointHolder);

                  for (SDFContactSensor sdfContactSensor : jointHolder.getContactSensors())
                  {
                     this.descriptionMutator.mutateContactSensorForModel(this, sdfContactSensor);
                  }

                  for (SDFForceSensor sdfForceSensor : jointHolder.getForceSensors())
                  {
                     this.descriptionMutator.mutateForceSensorForModel(this, sdfForceSensor);
                  }
               }
               joints.put(ModelFileLoaderConversionsHelper.sanitizeJointName(sdfJoint.getName()), jointHolder);
            }
            catch (IOException e)
            {
               System.err.println(e);
            }
         }
      }

      if(this.descriptionMutator != null)
      {
         this.descriptionMutator.mutateModelWithAdditions(this);
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

      transformToRoot = ModelFileLoaderConversionsHelper.poseToTransform(model.getPose());

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

   public ClassLoader getResourceClassLoader()
   {
      return resourceClassLoader;
   }

   @Override
   public ArrayList<CollisionMeshDescription> getCollisionObjects(String name)
   {
      // TODO: SDF collision stuff to RobotDescription collision stuff.
      for(SDFLinkHolder linkHolder : rootLinks)
      {
         if(linkHolder.getName().equals(name))
         {
            SDFGraphics3DObject sdfGraphics3DObject = new SDFGraphics3DObject(linkHolder.getCollisions(), resourceDirectories);
            ArrayList<CollisionMeshDescription> collisionMeshDescriptions = new ArrayList<CollisionMeshDescription>();
            
            //TODO: Figure out and add the collision meshes...
            return collisionMeshDescriptions;
         }
      }

      SDFGraphics3DObject sdfGraphics3DObject = new SDFGraphics3DObject(joints.get(name).getChildLinkHolder().getCollisions(), resourceDirectories);
      CollisionMeshDescription collisionMeshDescription = new CollisionMeshDescription();
      
      ArrayList<CollisionMeshDescription> collisionMeshDescriptions = new ArrayList<CollisionMeshDescription>();
      //TODO: Figure out and add the collision meshes...
      return collisionMeshDescriptions;
   }

   @Override
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
      if(joints.get(parentJointName) != null)
         joints.get(parentJointName).addForceSensor(sdfForceSensor);
   }

   public void addContactSensor(String sensorName, String parentJointName, ContactSensorType type)
   {
      SDFContactSensor sdfContactSensor = new SDFContactSensor(sensorName, parentJointName, type);
      joints.get(parentJointName).addContactSensor(sdfContactSensor);
   }

   public SDFDescriptionMutator getSDFDescriptionMutator()
   {
      return descriptionMutator;
   }
}
