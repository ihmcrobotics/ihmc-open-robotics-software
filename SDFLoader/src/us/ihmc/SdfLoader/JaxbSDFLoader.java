package us.ihmc.SdfLoader;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Unmarshaller;

import us.ihmc.SdfLoader.xmlDescription.SDFModel;
import us.ihmc.SdfLoader.xmlDescription.SDFRoot;
import us.ihmc.SdfLoader.xmlDescription.SDFWorld;
import us.ihmc.SdfLoader.xmlDescription.SDFWorld.Road;
import us.ihmc.SdfLoader.SDFExoskeleton;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.sensors.ContactSensorType;
import us.ihmc.tools.io.printing.PrintTools;

public class JaxbSDFLoader
{
   private final LinkedHashMap<String, GeneralizedSDFRobotModel> generalizedSDFRobotModels = new LinkedHashMap<String, GeneralizedSDFRobotModel>();
   private final ArrayList<SDFWorld.Road> roads = new ArrayList<SDFWorld.Road>();


   public JaxbSDFLoader(File file, List<String> resourceDirectories) throws JAXBException, FileNotFoundException
   {
      this(new FileInputStream(file), resourceDirectories);
   }
   
   public JaxbSDFLoader(InputStream inputStream, List<String> resourceDirectories) throws JAXBException, FileNotFoundException
   {
      this(inputStream, resourceDirectories, null);
   }

   public JaxbSDFLoader(File file, String resourceDirectory, SDFDescriptionMutator mutator) throws JAXBException, FileNotFoundException
   {
      this(new FileInputStream(file), Arrays.asList(resourceDirectory), mutator);
   }
   
   public JaxbSDFLoader(InputStream inputStream, String[] resourceDirectories, SDFDescriptionMutator mutator) throws JAXBException, FileNotFoundException
   {
      this(inputStream, Arrays.asList(resourceDirectories), mutator);
   }

   public JaxbSDFLoader(InputStream inputStream, List<String> resourceDirectories, SDFDescriptionMutator mutator)
           throws JAXBException, FileNotFoundException
   {
      JAXBContext context = JAXBContext.newInstance(SDFRoot.class);
      Unmarshaller um = context.createUnmarshaller();
      SDFRoot sdfRoot = (SDFRoot) um.unmarshal(inputStream);

      List<SDFModel> models;
      if (sdfRoot.getWorld() != null)
      {
         models = sdfRoot.getWorld().getModels();

         if (sdfRoot.getWorld().getRoads() != null)
         {
            roads.addAll(sdfRoot.getWorld().getRoads());
         }

      }
      else
      {
         models = sdfRoot.getModels();
      }

      for (SDFModel modelInstance : models)
      {
         final String modelName = modelInstance.getName();

         generalizedSDFRobotModels.put(modelName, new GeneralizedSDFRobotModel(modelName, modelInstance, resourceDirectories, mutator));
      }
   }

   public JaxbSDFLoader(File file, List<String> resourceDirectories, SDFDescriptionMutator mutator) throws FileNotFoundException, JAXBException
   {
      this(new FileInputStream(file), resourceDirectories, mutator);
   }

   public Collection<GeneralizedSDFRobotModel> getGeneralizedSDFRobotModels()
   {
      return generalizedSDFRobotModels.values();
   }

   public List<Road> getRoads()
   {
      return roads;
   }

   private void checkModelName(String name)
   {
      if (!generalizedSDFRobotModels.containsKey(name))
      {
         throw new RuntimeException(name + " not found");
      }
   }

   public GeneralizedSDFRobotModel getGeneralizedSDFRobotModel(String name)
   {
      checkModelName(name);

      return generalizedSDFRobotModels.get(name);
   }

   public SDFHumanoidRobot createRobot(SDFJointNameMap sdfJointNameMap, boolean useCollisionMeshes)
   {
      return createRobot(sdfJointNameMap.getModelName(), sdfJointNameMap, useCollisionMeshes);
   }

   public SDFRobot createRobot(String modelName, boolean useCollisionMeshes)
   {
      return createRobot(modelName, null, useCollisionMeshes);
   }

   private SDFHumanoidRobot createRobot(String modelName, SDFJointNameMap sdfJointNameMap, boolean useCollisionMeshes)
   {
      return createRobot(modelName, sdfJointNameMap, useCollisionMeshes, true, true);
   }

   public SDFHumanoidRobot createRobot(String modelName, SDFJointNameMap sdfJointNameMap, boolean useCollisionMeshes, boolean enableTorqueVelocityLimits,
           boolean enableDamping)
   {
      checkModelName(modelName);

      GeneralizedSDFRobotModel generalizedSDFRobotModel = generalizedSDFRobotModels.get(modelName);

      return new SDFHumanoidRobot(generalizedSDFRobotModel, generalizedSDFRobotModel.getSDFDescriptionMutator(), sdfJointNameMap, useCollisionMeshes, enableTorqueVelocityLimits, enableDamping);
   }
   
   public SDFExoskeleton createExoskeleton(SDFJointNameMap sdfJointNameMap, boolean useCollisionMeshes)
   {
      return createExoskeleton(sdfJointNameMap.getModelName(), sdfJointNameMap, useCollisionMeshes);
   }
   
   private SDFExoskeleton createExoskeleton(String modelName, SDFJointNameMap sdfJointNameMap, boolean useCollisionMeshes)
   {
      return createExoskeleton(modelName, sdfJointNameMap, useCollisionMeshes, true, true);
   }
   
   public SDFExoskeleton createExoskeleton(String modelName, SDFJointNameMap sdfJointNameMap, boolean useCollisionMeshes, boolean enableTorqueVelocityLimits,
		   boolean enableDamping)
   {
	   checkModelName(modelName);
	   
	   GeneralizedSDFRobotModel generalizedSDFRobotModel = generalizedSDFRobotModels.get(modelName);
	   
	   return new SDFExoskeleton(generalizedSDFRobotModel, generalizedSDFRobotModel.getSDFDescriptionMutator(), sdfJointNameMap, useCollisionMeshes, enableTorqueVelocityLimits, enableDamping);
   }

   public void addForceSensor(SDFJointNameMap jointMap, String sensorName, String parentJointName, RigidBodyTransform transformToParentJoint)
   {
      generalizedSDFRobotModels.get(jointMap.getModelName()).addForceSensor(sensorName, parentJointName, transformToParentJoint);
   }

   public void addContactSensor(SDFJointNameMap jointMap, String sensorName, String parentJointName, ContactSensorType type)
   {
      generalizedSDFRobotModels.get(jointMap.getModelName()).addContactSensor(sensorName, parentJointName, type);
   }

   public SDFFullHumanoidRobotModel createFullRobotModel(SDFJointNameMap sdfJointNameMap)
   {
      return createFullRobotModel(sdfJointNameMap, new String[0]);
   }

   public SDFFullHumanoidRobotModel createFullRobotModel(SDFJointNameMap sdfJointNameMap, String[] sensorFramesToTrack)
   {
      if (sdfJointNameMap != null)
      {
         String modelName = sdfJointNameMap.getModelName();
         checkModelName(modelName);

         return new SDFFullHumanoidRobotModel(generalizedSDFRobotModels.get(modelName).getRootLinks().get(0), sdfJointNameMap, sensorFramesToTrack);
      }
      else
      {
         throw new RuntimeException("Cannot make a fullrobotmodel without a sdfJointNameMap");
      }
   }
}
