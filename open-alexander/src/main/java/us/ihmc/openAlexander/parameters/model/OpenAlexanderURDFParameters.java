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
   public static final String URDF_MODEL_NAME = "Alexander";
   private static final String[] RESOURCE_DIRECTORIES = new String[] {"alexander_V1_description/",
                                                                      "alexander_V1_description/urdf/",
                                                                      "alexander_V1_description/meshes/"};

   private static final String[] LOGGED_RESOURCES = {"alexander_V1_description/"};

   public static final String URDF_FULL_BODY = "alexander_V1_description/urdf/alexander_v1.fullBody.urdf";
   public static final String URDF_LOWER_BODY = "alexander_V1_description/urdf/alexander_v1.lowerBody.urdf";
   public static final String URDF_HEAD = "alexander_V1_description/urdf/alexander_v1.head.urdf";

   public static final String URDF_LOWER_BODY_ONLY = "alexander_V1_description/urdf/alexander_v1.lowerBodyOnly.urdf";
   
   public static final String URDF_LEFT_ARM_NUB_FOREARM = "alexander_V1_description/urdf/alexander_v0.leftArmFixedForearm.urdf";
   public static final String URDF_RIGHT_ARM_NUB_FOREARM = "alexander_V1_description/urdf/alexander_v0.rightArmFixedForearm.urdf";

   // Model paths are represented as Strings because resource paths should always be separated with "/" -- this is not platform dependent
   private final Collection<String> urdfModelPath;

   public OpenAlexanderURDFParameters(AlexanderVersionInterface alexanderVersion)
   {
      urdfModelPath = alexanderVersion.getModelPath();
   }

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
      for (String path : urdfModelPath)
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
         URDFModel model = URDFTools.loadURDFModel(inputStreamList, Arrays.asList(RESOURCE_DIRECTORIES), getClass().getClassLoader());
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