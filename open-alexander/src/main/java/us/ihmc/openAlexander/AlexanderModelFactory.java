package us.ihmc.openAlexander;

import org.apache.commons.lang3.SystemUtils;
import us.ihmc.openAlexander.parameters.model.HumanoidURDFParameterInterface;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.modelFileLoaders.RobotDefinitionLoader;
import us.ihmc.multicastLogDataProtocol.modelLoaders.DefaultLogModelProvider;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.openAlexander.parameters.model.OpenAlexanderURDFParameters;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.WrenchSensorDefinition;
import us.ihmc.scs2.definition.robot.urdf.URDFTools;
import us.ihmc.scs2.definition.robot.urdf.items.URDFModel;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

import java.util.Arrays;
import java.util.function.Consumer;
import java.util.function.Predicate;

public class AlexanderModelFactory
{
   private final String[] resourceModelsToBeLogged;
   private final HumanoidURDFParameterInterface urdfParameters;
   private final RobotContactPointParameters<RobotSide> contactPointParameters;
   private RobotDefinition simulationRobotDefinition;
   private final AlexanderVersionInterface alexanderVersion;
   private RobotDefinition controllerRobotDefinition;
   private final AlexanderJointMap jointMap;
   private final Consumer<RobotDefinition> robotDefinitionMutator;

   public AlexanderModelFactory(AlexanderVersionInterface alexanderVersion,
                                AlexanderJointMap jointMap,
                                RobotContactPointParameters<RobotSide> contactPointParameters,
                                Consumer<RobotDefinition> robotDefinitionMutator)
   {
      this.alexanderVersion = alexanderVersion;
      this.jointMap = jointMap;
      this.contactPointParameters = contactPointParameters;
      this.robotDefinitionMutator = robotDefinitionMutator;
//
      urdfParameters = alexanderVersion.getURDFParameters();
      resourceModelsToBeLogged = urdfParameters.getLoggedResources();
   }

   public LogModelProvider createLogModelProvider()
   {
      // Converts the file paths to windows for compatibility
      if (SystemUtils.OS_NAME.contains("Windows"))
      {
         for (int i = 0; i < resourceModelsToBeLogged.length; i++)
         {
            resourceModelsToBeLogged[i] = resourceModelsToBeLogged[i].replace('/', '\\');
         }
      }

      Predicate<String> filter = resourcePath ->
      {
         for (String model : resourceModelsToBeLogged)
         {
            if(resourcePath.startsWith(model))
               return true;
         }

         return false;
      };

      // Nadia supports URDF models
      Class<?> clazz = URDFModel.class;
      return new DefaultLogModelProvider<>(clazz, urdfParameters.getURDFModelName(), urdfParameters.getURDFAsInputStream(), filter, urdfParameters.getResourceDirectories());
   }

   public RobotDefinition getSCS1RobotDefinition()
   {
      if (simulationRobotDefinition == null)
      {
         // The URDF loading doesn't work because the 4-bar creates duplicate children
         URDFTools.URDFParserProperties parserProperties = new URDFTools.URDFParserProperties();
         parserProperties.setTransformToZUp(false);
         parserProperties.setHandleImplicitJointDefinitions(false);
         simulationRobotDefinition = RobotDefinitionLoader.loadURDFModel(urdfParameters.getURDFAsInputStream(),
                                                                         Arrays.asList(urdfParameters.getResourceDirectories()),
                                                                         getClass().getClassLoader(),
                                                                         urdfParameters.getURDFModelName(),
                                                                         contactPointParameters,
                                                                         jointMap,
                                                                         true,
                                                                         parserProperties);
         if (alexanderVersion.getSensorInformation() != null)
         {
            addForceSensors(simulationRobotDefinition, alexanderVersion.getSensorInformation());
         }

         if (robotDefinitionMutator != null)
            robotDefinitionMutator.accept(simulationRobotDefinition);
      }

      return simulationRobotDefinition;
   }

   private void addForceSensors(RobotDefinition robotDefinition, AlexanderSensorInformation alexanderSensorInformation)
   {
      SideDependentList<String> feetForceSensorNames = alexanderSensorInformation.getFeetForceSensorNames();
      SideDependentList<String> feetForceSensorParentJointNames = alexanderSensorInformation.getFeetForceSensorParentJointNames();

      for (RobotSide robotSide : RobotSide.values)
      {
         String forceSensorName = feetForceSensorNames.get(robotSide);
         String forceSensorParentJointName = feetForceSensorParentJointNames.get(robotSide);

         RigidBodyTransform transform = new RigidBodyTransform();
         transform.getTranslation().setX(0.024);//from cad, really this time
         transform.getTranslation().setZ(-0.036);//from cad, really this time

         robotDefinition.getJointDefinition(forceSensorParentJointName).addSensorDefinition(new WrenchSensorDefinition(forceSensorName, transform));
      }
   }

   public RobotDefinition getControllerRobotDefinition()
   {
      if (controllerRobotDefinition == null)
      {
         controllerRobotDefinition = getSCS1RobotDefinition();
      }
      return controllerRobotDefinition;
   }
}
