package us.ihmc.exampleSimulations.beetle.parameters;

import java.util.Arrays;

import us.ihmc.modelFileLoaders.RobotDefinitionLoader;
import us.ihmc.modelFileLoaders.SdfLoader.SDFModelLoader;
import us.ihmc.multicastLogDataProtocol.modelLoaders.DefaultLogModelProvider;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.FullRobotModelFactory;
import us.ihmc.robotModels.FullRobotModelWrapper;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;

public class RhinoBeetleModelFactory implements FullRobotModelFactory
{
   private final RhinoBeetleSDFParameters sdfParameters;
   private final RhinoBeetleJointNameMapAndContactDefinition jointMapAndContactInfo;

   private final RobotDefinition robotDefinition;

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
   }

   public FloatingRootJointRobot createSdfRobot()
   {

      return new FloatingRootJointRobot(robotDefinition);
   }

   @Override
   public FullRobotModel createFullRobotModel()
   {
      /* TODO impose frame uniqueness if needed */
      return new FullRobotModelWrapper(robotDefinition, jointMapAndContactInfo, false);
   }

   @Override
   public FullRobotModel createFullRobotModel(boolean enforceUniqueReferenceFrames)
   {
      return new FullRobotModelWrapper(robotDefinition, jointMapAndContactInfo, enforceUniqueReferenceFrames);
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

   public RhinoBeetleJointNameMapAndContactDefinition getJointNameMap()
   {
      return jointMapAndContactInfo;
   }

}
