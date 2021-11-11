package us.ihmc.exampleSimulations.beetle.parameters;

import java.util.Arrays;

import us.ihmc.modelFileLoaders.RobotDefinitionLoader;
import us.ihmc.modelFileLoaders.SdfLoader.SDFModelLoader;
import us.ihmc.multicastLogDataProtocol.modelLoaders.DefaultLogModelProvider;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.FullRobotModelFactory;
import us.ihmc.robotModels.FullRobotModelFromDescription;
import us.ihmc.robotModels.description.RobotDefinitionConverter;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;

public class RhinoBeetleModelFactory implements FullRobotModelFactory
{
   private final RhinoBeetleSDFParameters sdfParameters;
   private final RhinoBeetleJointNameMapAndContactDefinition jointMapAndContactInfo;

   private final RobotDefinition robotDefinition;
   private final RobotDescription robotDescription;

   public RhinoBeetleModelFactory()
   {
      sdfParameters = new RhinoBeetleSDFParameters();
      jointMapAndContactInfo = new RhinoBeetleJointNameMapAndContactDefinition();

      robotDefinition = RobotDefinitionLoader.loadSDFModel(sdfParameters.getSdfAsInputStream(),
                                                           Arrays.asList(sdfParameters.getResourceDirectories()),
                                                           getClass().getClassLoader(),
                                                           sdfParameters.getSdfModelName(),
                                                           jointMapAndContactInfo,
                                                           jointMapAndContactInfo,
                                                           true);
      robotDescription = RobotDefinitionConverter.toRobotDescription(robotDefinition);
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
      return new DefaultLogModelProvider<>(SDFModelLoader.class,
                                           sdfParameters.getSdfModelName(),
                                           sdfParameters.getSdfAsInputStream(),
                                           sdfParameters.getResourceDirectories());
   }

   @Override
   public RobotDefinition getRobotDefinition()
   {
      return robotDefinition;
   }

   public RobotDescription getRobotDescription()
   {
      return robotDescription;
   }

   public RhinoBeetleJointNameMapAndContactDefinition getJointNameMap()
   {
      return jointMapAndContactInfo;
   }

}
