package us.ihmc.llaQuadruped;

import java.io.FileNotFoundException;
import java.io.InputStream;

import javax.xml.bind.JAXBException;

import us.ihmc.SdfLoader.*;
import us.ihmc.quadrupedRobotics.model.QuadrupedModelFactory;
import us.ihmc.tools.io.printing.PrintTools;

public class LLAQuadrupedModelFactory extends QuadrupedModelFactory implements SDFFullQuadrupedRobotModelFactory
{
   private final LLAQuadrupedSDFParameters sdfParameters = new LLAQuadrupedSDFParameters();
   private final GeneralizedSDFRobotModel generalizedSDFRobotModel;

   private JaxbSDFLoader loader;

   private final LLAQuadrupedJointNameMap jointMap;

   public LLAQuadrupedModelFactory()
   {
      jointMap = new LLAQuadrupedJointNameMap(new LLAQuadrupedPhysicalProperties());

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
   }
   
   @Override
   public GeneralizedSDFRobotModel getGeneralizedRobotModel()
   {
      return generalizedSDFRobotModel;
   }

   @Override
   public SDFRobot createSdfRobot()
   {
      boolean useCollisionMeshes = false;
      boolean enableTorqueVelocityLimits = true;
      boolean enableJointDamping = true;

      GeneralizedSDFRobotModel generalizedSDFRobotModel = getGeneralizedRobotModel();
      SDFRobot sdfRobot = new SDFRobot(generalizedSDFRobotModel, null, jointMap, useCollisionMeshes, enableTorqueVelocityLimits, enableJointDamping);
      return sdfRobot;
   }

   @Override
   public SDFFullQuadrupedRobotModel createFullRobotModel()
   {
      GeneralizedSDFRobotModel generalizedSDFRobotModel = getGeneralizedRobotModel();
      SDFLinkHolder rootLink = generalizedSDFRobotModel.getRootLinks().get(0);
      String[] sensorLinksToTrack = new String[] {};
      SDFFullQuadrupedRobotModel sdfFullRobotMdoel = new SDFFullQuadrupedRobotModel(rootLink, jointMap, sensorLinksToTrack);

      return sdfFullRobotMdoel;
   }
}
