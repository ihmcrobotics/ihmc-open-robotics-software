package us.ihmc.alexander;

import org.apache.commons.lang3.SystemUtils;
import us.ihmc.alexander.parameters.controller.AlexanderContactPointParameters;
import us.ihmc.alexander.parameters.model.AlexanderURDFParameters;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.modelFileLoaders.RobotDefinitionLoader;
import us.ihmc.multicastLogDataProtocol.modelLoaders.DefaultLogModelProvider;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
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
   private final String[] resourceModelsToBeLogged = {"models/alexander_v0"};
   private final AlexanderURDFParameters urdfParameters;
   private final RobotContactPointParameters<RobotSide> contactPointParameters;
   private RobotDefinition simulationRobotDefinition;
   private final AlexanderVersion alexanderVersion;
   private RobotDefinition controllerRobotDefinition;
   private final AlexanderJointMap jointMap;
   private final Consumer<RobotDefinition> robotDefinitionMutator;

   public AlexanderModelFactory(AlexanderVersion alexanderVersion,
                               AlexanderJointMap jointMap,
                               RobotContactPointParameters<RobotSide> contactPointParameters,
                               Consumer<RobotDefinition> robotDefinitionMutator)
   {
      this.alexanderVersion = alexanderVersion;
      this.jointMap = jointMap;
      this.contactPointParameters = contactPointParameters;
      this.robotDefinitionMutator = robotDefinitionMutator;
//
      urdfParameters = new AlexanderURDFParameters(alexanderVersion);
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

      for (RobotSide robotSide : RobotSide.values)
      {
         String forceSensorName = feetForceSensorNames.get(robotSide);

         RigidBodyTransform transform = new RigidBodyTransform();
         transform.getTranslation().setX(0.053);//from cad
         transform.getTranslation().setZ(-0.06497200);//from cad
         // New force torque sensor for the right foot was installed with an additional yaw offset.
         // TODO Fixme once the sensor is mounter properly
         transform.appendYawRotation(robotSide == RobotSide.LEFT ? Math.PI : -1.0 / 3.0 * Math.PI);
         transform.appendRollRotation(Math.PI);

         robotDefinition.getJointDefinition(forceSensorName).addSensorDefinition(new WrenchSensorDefinition(forceSensorName, transform));
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
