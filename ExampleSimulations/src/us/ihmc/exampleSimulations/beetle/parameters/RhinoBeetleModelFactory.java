package us.ihmc.exampleSimulations.beetle.parameters;

import java.io.FileNotFoundException;

import javax.xml.bind.JAXBException;

import us.ihmc.modelFileLoaders.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.modelFileLoaders.SdfLoader.JaxbSDFLoader;
import us.ihmc.modelFileLoaders.SdfLoader.RobotDescriptionFromSDFLoader;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.multicastLogDataProtocol.modelLoaders.SDFLogModelProvider;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.FullRobotModelFactory;
import us.ihmc.robotModels.FullRobotModelFromDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;

public class RhinoBeetleModelFactory implements FullRobotModelFactory
{
   private final RhinoBeetleSDFParameters sdfParameters;
   private final RhinoBeetleJointNameMap jointNameMap;

   private final GeneralizedSDFRobotModel generalizedSDFRobotModel;

   private final RobotDescription robotDescription;

   public RhinoBeetleModelFactory()
   {
      sdfParameters = new RhinoBeetleSDFParameters();
      jointNameMap = new RhinoBeetleJointNameMap();

      JaxbSDFLoader loader;
      try
      {
         loader = new JaxbSDFLoader(sdfParameters.getSdfAsInputStream(), sdfParameters.getResourceDirectories(), null);
         generalizedSDFRobotModel = loader.getGeneralizedSDFRobotModel(sdfParameters.getSdfModelName());
      }
      catch (FileNotFoundException | JAXBException e)
      {
         throw new RuntimeException("Cannot load model", e);
      }

      boolean useCollisionMeshes = false;

      GeneralizedSDFRobotModel generalizedSDFRobotModel = getGeneralizedRobotModel();
      RobotDescriptionFromSDFLoader descriptionLoader = new RobotDescriptionFromSDFLoader();
      robotDescription = descriptionLoader.loadRobotDescriptionFromSDF(generalizedSDFRobotModel, getJointNameMap(), useCollisionMeshes);
   }

   private GeneralizedSDFRobotModel getGeneralizedRobotModel()
   {
      return generalizedSDFRobotModel;
   }

   public FloatingRootJointRobot createSdfRobot()
   {

      return new FloatingRootJointRobot(robotDescription);
   }

   public FullRobotModel createFullRobotModel()
   {
      String[] sensorLinksToTrack = new String[] {};
      return new FullRobotModelFromDescription(robotDescription, getJointNameMap(), sensorLinksToTrack);
   }

   public LogModelProvider createLogModelProvider()
   {
      return new SDFLogModelProvider(sdfParameters);
   }

   @Override
   public RobotDescription getRobotDescription()
   {
      return robotDescription;
   }

   public RhinoBeetleJointNameMap getJointNameMap()
   {
      return jointNameMap;
   }
   
}
