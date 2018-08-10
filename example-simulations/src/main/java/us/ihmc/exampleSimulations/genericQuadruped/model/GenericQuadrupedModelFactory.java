package us.ihmc.exampleSimulations.genericQuadruped.model;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commons.PrintTools;
import us.ihmc.modelFileLoaders.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.modelFileLoaders.SdfLoader.JaxbSDFLoader;
import us.ihmc.modelFileLoaders.SdfLoader.RobotDescriptionFromSDFLoader;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.model.QuadrupedModelFactory;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotModels.FullQuadrupedRobotModelFromDescription;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.robotics.robotDescription.RobotDescription;

import javax.xml.bind.JAXBException;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.Collection;

public class GenericQuadrupedModelFactory extends QuadrupedModelFactory
{
   private final GenericQuadrupedSDFParameters sdfParameters = new GenericQuadrupedSDFParameters();
   private final GeneralizedSDFRobotModel generalizedSDFRobotModel;

   private JaxbSDFLoader loader;

   private final GenericQuadrupedJointNameMapAndContactDefinition jointMapAndContactInfo;

   private final RobotDescription robotDescription;

   public GenericQuadrupedModelFactory()
   {
      jointMapAndContactInfo = new GenericQuadrupedJointNameMapAndContactDefinition(new GenericQuadrupedPhysicalProperties());

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

   @Override
   public String getParameterResourceName(WholeBodyControllerCoreMode controlMode)
   {
      switch(controlMode)
      {
      case INVERSE_KINEMATICS:
      case VIRTUAL_MODEL:
         return "/parameters/simulation_force_controller.xml";
      default:
         throw new RuntimeException("No parameter file exists for control mode: " + controlMode);
      }
   }

   @Override
   public InputStream getParameterInputStream(WholeBodyControllerCoreMode controlMode)
   {
      return getClass().getResourceAsStream(getParameterResourceName(controlMode));
   }

}
