package us.ihmc.modelFileLoaders.SdfLoader;

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

import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFModel;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFRoot;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFWorld;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFWorld.Road;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.partNames.JointNameMap;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.sensors.ContactSensorType;

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

//   public FloatingRootJointRobot createRobot(JointNameMap jointNameMap, boolean useCollisionMeshes)
//   {
//      return createRobot(jointNameMap.getModelName(), jointNameMap, useCollisionMeshes);
//   }
//
//   public FloatingRootJointRobot createRobot(String modelName, boolean useCollisionMeshes)
//   {
//      return createRobot(modelName, null, useCollisionMeshes);
//   }
//
//   private FloatingRootJointRobot createRobot(String modelName, JointNameMap jointNameMap, boolean useCollisionMeshes)
//   {
//      return createRobot(modelName, jointNameMap, useCollisionMeshes, true, true);
//   }
//
//   public FloatingRootJointRobot createRobot(String modelName, JointNameMap jointNameMap, boolean useCollisionMeshes, boolean enableTorqueVelocityLimits,
//           boolean enableJointDamping)
//   {
//      checkModelName(modelName);
//
//      GeneralizedSDFRobotModel generalizedSDFRobotModel = generalizedSDFRobotModels.get(modelName);
//      RobotDescriptionFromSDFLoader loader = new RobotDescriptionFromSDFLoader();
//      RobotDescription description = loader.loadRobotDescriptionFromSDF(generalizedSDFRobotModel, jointNameMap, useCollisionMeshes, enableTorqueVelocityLimits, enableJointDamping);
//
//      return new FloatingRootJointRobot(description);
//   }

   public void addForceSensor(JointNameMap jointMap, String sensorName, String parentJointName, RigidBodyTransform transformToParentJoint)
   {
      generalizedSDFRobotModels.get(jointMap.getModelName()).addForceSensor(sensorName, parentJointName, transformToParentJoint);
   }

   public void addContactSensor(JointNameMap jointMap, String sensorName, String parentJointName, ContactSensorType type)
   {
      generalizedSDFRobotModels.get(jointMap.getModelName()).addContactSensor(sensorName, parentJointName, type);
   }

   public RobotDescription createRobotDescription(JointNameMap jointNameMap)
   {
      boolean useCollisionMeshes = false;

      return createRobotDescription(jointNameMap, useCollisionMeshes);
   }

   public RobotDescription createRobotDescription(JointNameMap jointNameMap, boolean useCollisionMeshes)
   {
      if (jointNameMap != null)
      {
         String modelName = jointNameMap.getModelName();
         checkModelName(modelName);


         RobotDescriptionFromSDFLoader loader = new RobotDescriptionFromSDFLoader();
         RobotDescription description = loader.loadRobotDescriptionFromSDF(generalizedSDFRobotModels.get(modelName), jointNameMap, useCollisionMeshes);

         return description;
      }
      else
      {
         throw new RuntimeException("Cannot make a fullrobotmodel without a jointNameMap");
      }
   }
}
