package us.ihmc.llaQuadrupedController.model;

import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.Collection;

import javax.xml.bind.JAXBException;

import us.ihmc.commons.PrintTools;
import us.ihmc.modelFileLoaders.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.modelFileLoaders.SdfLoader.JaxbSDFLoader;
import us.ihmc.modelFileLoaders.SdfLoader.RobotDescriptionFromSDFLoader;
import us.ihmc.quadrupedRobotics.model.QuadrupedModelFactory;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotModels.FullQuadrupedRobotModelFromDescription;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.robotics.robotDescription.RobotDescription;

public class LLAQuadrupedModelFactory extends QuadrupedModelFactory
{
   private final LLAQuadrupedSDFParameters sdfParameters = new LLAQuadrupedSDFParameters();
   private final GeneralizedSDFRobotModel generalizedSDFRobotModel;

   private JaxbSDFLoader loader;

   private final LLAQuadrupedJointNameMapAndContactDefinition jointMapAndContactInfo;

   private final RobotDescription robotDescription;

   public LLAQuadrupedModelFactory()
   {
      jointMapAndContactInfo = new LLAQuadrupedJointNameMapAndContactDefinition(new LLAQuadrupedPhysicalProperties());

      try
      {
         InputStream sdfAsInputStream = sdfParameters.getSdfAsInputStream();
         loader = new JaxbSDFLoader(sdfAsInputStream, sdfParameters.getResourceDirectories(), null);
         generalizedSDFRobotModel = loader.getGeneralizedSDFRobotModel(sdfParameters.getSdfModelName());
      }
      catch (FileNotFoundException fileNotFoundException)
      {
         PrintTools.error(this, FileNotFoundException.class.getSimpleName() + ": " + fileNotFoundException.getMessage());
         throw new RuntimeException("Unrecoverable error.");
      }
      catch (JAXBException jaxbException)
      {
         PrintTools.error(this, JAXBException.class.getSimpleName() + ": " + jaxbException.getMessage());
         throw new RuntimeException("Unrecoverable error.");
      }

      boolean useCollisionMeshes = false;

      GeneralizedSDFRobotModel generalizedSDFRobotModel = getGeneralizedRobotModel();
      RobotDescriptionFromSDFLoader loader = new RobotDescriptionFromSDFLoader();
      robotDescription = loader.loadRobotDescriptionFromSDF(generalizedSDFRobotModel, jointMapAndContactInfo, jointMapAndContactInfo, useCollisionMeshes);
   }

   private GeneralizedSDFRobotModel getGeneralizedRobotModel()
   {
      return generalizedSDFRobotModel;
   }

   @Override
   public RobotDescription createSdfRobot()
   {
      return robotDescription;
   }

   @Override
   public FullQuadrupedRobotModel createFullRobotModel()
   {
      GeneralizedSDFRobotModel generalizedSDFRobotModel = getGeneralizedRobotModel();
      String[] sensorLinksToTrack = new String[] {};

      FullQuadrupedRobotModel sdfFullRobotMdoel = new FullQuadrupedRobotModelFromDescription(robotDescription, jointMapAndContactInfo, sensorLinksToTrack);

      return sdfFullRobotMdoel;
   }

   @Override
   public Collection<QuadrupedJointName> getQuadrupedJointNames()
   {
      return jointMapAndContactInfo.getQuadrupedJointNames();
   }

   @Override
   public String getSDFNameForJointName(QuadrupedJointName quadrupedJointName)
   {
      return jointMapAndContactInfo.getSDFNameForJointName(quadrupedJointName);
   }

   @Override
   public RobotDescription getRobotDescription()
   {
      return robotDescription;
   }
}
