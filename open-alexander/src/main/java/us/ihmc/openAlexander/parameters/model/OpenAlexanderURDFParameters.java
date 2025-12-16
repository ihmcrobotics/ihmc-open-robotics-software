package us.ihmc.openAlexander.parameters.model;

import jakarta.xml.bind.JAXBException;
import us.ihmc.openAlexander.AlexanderVersionInterface;
import us.ihmc.scs2.definition.robot.urdf.URDFTools;
import us.ihmc.scs2.definition.robot.urdf.items.URDFModel;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;

/**
 * This class contains the paths to the individual URDF files that will be used to build the robot. It also holds some simple methods to combine the different
 * files into one input stream that will be used to build the URDF.
 */
public class OpenAlexanderURDFParameters implements HumanoidURDFParameterInterface
{
   public static final String URDF_MODEL_NAME = "Alex";
   private static final String[] RESOURCE_DIRECTORIES = new String[] {"alex_V1_description/",
                                                                      "alex_V1_description/urdf/",
                                                                      "alex_V1_description/meshes/"};

   private static final String[] LOGGED_RESOURCES = {"alex_V1_description/"};

   public static final String URDF_FULL_BODY = "alex_v1.fullBody.urdf";
   public static final String URDF_LOWER_BODY = "alex_v1.lowerBody.urdf";
   public static final String URDF_HEAD = "alex_v1.head.urdf";

   public static final String URDF_LOWER_BODY_ONLY = "alex_v1.lowerBodyOnly.urdf";
   
   public static final String URDF_LEFT_ARM_NUB_FOREARM = "alex_v0.leftArmFixedForearm.urdf";
   public static final String URDF_RIGHT_ARM_NUB_FOREARM = "alex_v0.rightArmFixedForearm.urdf";

   private final String[] robotModelResourceDirectory = new String[1];
   private final String[] urdfResourceDirectories;
   private final String[] urdfResourcesWithPath;

   private final URDFTools.URDFParserProperties urdfParserProperties = new URDFTools.URDFParserProperties();

   public OpenAlexanderURDFParameters(AlexanderVersionInterface alexanderVersion)
   {
      urdfParserProperties.setHandleImplicitJointDefinitions(false);

      // Directory containing all robot model resources for the given robot version
      robotModelResourceDirectory[0] = alexanderVersion.getRobotModelResourceDirectory();

      // List of directories containing robot model URDF resources for the given robot version
      List<String> urdfResourceDirectoriesList = new ArrayList<>();
      urdfResourceDirectoriesList.add(alexanderVersion.getRobotModelResourceDirectory());
      urdfResourceDirectoriesList.add(alexanderVersion.getRobotModelResourceDirectory() + "urdf" + '/');
      urdfResourceDirectoriesList.add(alexanderVersion.getRobotModelResourceDirectory() + "meshes" + '/');
      urdfResourceDirectories = urdfResourceDirectoriesList.toArray(new String[0]);

      // List of urdf files and their path within the robot model resource directory for the given robot version
      List<String> urdfResourcesWithPathList = new ArrayList<>();
      for(String file : alexanderVersion.getURDFDescriptionResources())
      {
         String urdfResource;
         if (file.contains("ezGripper/") || file.contains("abilityHand/"))
            urdfResource = file;
         else
            urdfResource = alexanderVersion.getRobotModelResourceDirectory() + "urdf" + '/' + file;

         urdfResourcesWithPathList.add(urdfResource);
      }

      urdfResourcesWithPath = urdfResourcesWithPathList.toArray(new String[0]);   }

   @Override
   public String getURDFModelName()
   {
      return URDF_MODEL_NAME;
   }

   @Override
   public String[] getResourceDirectories()
   {
      return RESOURCE_DIRECTORIES;
   }

   @Override
   public String[] getLoggedResources()
   {
      return LOGGED_RESOURCES;
   }

   @Override
   public InputStream getURDFAsInputStream()
   {
      List<InputStream> inputStreamList = new ArrayList<>();
      InputStream inputStream;

      // Add all the input streams to a list
      for (String path : urdfResourcesWithPath)
      {
         InputStream is = getClass().getClassLoader().getResourceAsStream(path);
         inputStreamList.add(is);

         if (is == null)
         {
            throw new RuntimeException("Unable to open robot model file: " + path);
         }
      }

      // Load a URDFModel and convert that back into one single InputStream to send over the network
      try
      {
         URDFModel model = URDFTools.loadURDFModel(inputStreamList, Arrays.asList(urdfResourceDirectories), getClass().getClassLoader(), urdfParserProperties);
         ByteArrayOutputStream bos = new ByteArrayOutputStream();
         URDFTools.saveURDFModel(bos, model);
         inputStream = new ByteArrayInputStream(bos.toByteArray());
      }
      catch (JAXBException e)
      {
         throw new RuntimeException(e);
      }

      return inputStream;
   }
}