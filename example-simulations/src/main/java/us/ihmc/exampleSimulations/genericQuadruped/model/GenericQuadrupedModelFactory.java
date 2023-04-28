package us.ihmc.exampleSimulations.genericQuadruped.model;

import java.io.InputStream;
import java.util.Arrays;
import java.util.Collection;
import java.util.concurrent.atomic.AtomicInteger;

import us.ihmc.modelFileLoaders.RobotDefinitionLoader;
import us.ihmc.quadrupedRobotics.model.QuadrupedModelFactory;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotModels.FullQuadrupedRobotModelWrapper;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.robotics.partNames.QuadrupedJointNameMap;
import us.ihmc.scs2.definition.robot.RobotDefinition;

public class GenericQuadrupedModelFactory extends QuadrupedModelFactory
{

   private static final String parameterFileName = "/parameters/simulation_force_controller.xml";

   private final GenericQuadrupedSDFParameters sdfParameters = new GenericQuadrupedSDFParameters();

   private final GenericQuadrupedJointNameMapAndContactDefinition jointMapAndContactInfo;

   private final RobotDefinition robotDefinition;

   public GenericQuadrupedModelFactory()
   {
      jointMapAndContactInfo = new GenericQuadrupedJointNameMapAndContactDefinition(new GenericQuadrupedPhysicalProperties());

      robotDefinition = RobotDefinitionLoader.loadSDFModel(sdfParameters.getSdfAsInputStream(),
                                                           Arrays.asList(sdfParameters.getResourceDirectories()),
                                                           getClass().getClassLoader(),
                                                           sdfParameters.getSdfModelName(),
                                                           jointMapAndContactInfo,
                                                           jointMapAndContactInfo,
                                                           true);
   }

   @Override
   public FullQuadrupedRobotModel createFullRobotModel()
   {
      return new FullQuadrupedRobotModelWrapper(robotDefinition, jointMapAndContactInfo);
   }

   @Override
   public FullQuadrupedRobotModel createFullRobotModel(boolean enforceUniqueReferenceFrames)
   {
      return new FullQuadrupedRobotModelWrapper(robotDefinition, jointMapAndContactInfo, enforceUniqueReferenceFrames);
   }

   @Override
   public QuadrupedJointNameMap getJointMap()
   {
      return jointMapAndContactInfo;
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
   public RobotDefinition getRobotDefinition()
   {
      return robotDefinition;
   }

   @Override
   public String getParameterResourceName()
   {
      return parameterFileName;
   }

   @Override
   public InputStream getParameterInputStream()
   {
      return getClass().getResourceAsStream(getParameterResourceName());
   }

}
