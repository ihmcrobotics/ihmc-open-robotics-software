package us.ihmc.zulu;

import jakarta.xml.bind.JAXBException;
import us.ihmc.zulu.parameters.model.ZuluURDFParameters;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.urdf.URDFTools;
import us.ihmc.scs2.definition.robot.urdf.items.URDFModel;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationSession;

import java.util.Arrays;

public class SimpleModelViewerForZulu
{
   public static void main(String[] args) throws JAXBException
   {
      // We define the version of the robot we want to us
      ZuluVersion version = ZuluVersion.V1_FULL_ROBOT;
      ZuluURDFParameters modelParameters = new ZuluURDFParameters(version);

      // We create the URDF model to be loaded into SCS2
      URDFModel urdfModel = URDFTools.loadURDFModel(modelParameters.getURDFAsInputStream(),
                                                    Arrays.asList(modelParameters.getResourceDirectories()),
                                                    modelParameters.getClass().getClassLoader());

      // Set some default settings for the URDF in order to load properly
      URDFTools.URDFParserProperties urdfParserProperties = new URDFTools.URDFParserProperties();
      urdfParserProperties.setRootJointFactory(null);
      urdfParserProperties.setTransformToZUp(false);

      // Get a robot definition that can be loaded into SCS2
      RobotDefinition robotDefinition = URDFTools.toRobotDefinition(urdfModel, urdfParserProperties);

      // Start the simulation and add our robot to it!
      SimulationSession session = new SimulationSession();
      session.addRobot(robotDefinition);
      SessionVisualizer.startSessionVisualizer(session);
   }
}