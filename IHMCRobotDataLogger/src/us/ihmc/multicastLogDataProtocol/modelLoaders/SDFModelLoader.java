package us.ihmc.multicastLogDataProtocol.modelLoaders;

import java.io.ByteArrayInputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.net.MalformedURLException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.Arrays;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;

import javax.management.IntrospectionException;
import javax.xml.bind.JAXBException;

import us.ihmc.modelFileLoaders.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.modelFileLoaders.SdfLoader.JaxbSDFLoader;
import us.ihmc.modelFileLoaders.SdfLoader.RobotDescriptionFromSDFLoader;
import us.ihmc.modelFileLoaders.SdfLoader.SDFDescriptionMutator;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.tools.ClassLoaderTools;

public class SDFModelLoader implements LogModelLoader
{
   private final static String resourceDirectoryLocation = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "resources";

   private String modelName;
   private byte[] model;
   private String[] resourceDirectories;
   private byte[] resourceZip;
   private SDFDescriptionMutator descriptionMutator;

   @Override
   public void load(String modelName, byte[] model, String[] resourceDirectories, byte[] resourceZip, SDFDescriptionMutator descriptionMutator)
   {
      this.modelName = modelName;
      this.model = model;
      this.resourceDirectories = resourceDirectories;
      this.resourceZip = resourceZip;
      this.descriptionMutator = descriptionMutator;
   }

   @Override
   public String toString()
   {
      return "SDFModelLoader [modelName=" + modelName + ", model=" + model.length + ", resourceDirectories=" + Arrays.toString(resourceDirectories)
            + ", resourceZip=" + resourceZip.length + "]";
   }

   @Override
   public RobotDescription createRobot()
   {
      boolean useCollisionMeshes = false;

      GeneralizedSDFRobotModel generalizedSDFRobotModel = createJaxbSDFLoader().getGeneralizedSDFRobotModel(modelName);
      RobotDescriptionFromSDFLoader loader = new RobotDescriptionFromSDFLoader();
      RobotDescription description = loader.loadRobotDescriptionFromSDF(generalizedSDFRobotModel, null, null, useCollisionMeshes);
      return description;
   }

   public JaxbSDFLoader createJaxbSDFLoader()
   {
      if(resourceZip != null)
      {
         Path resourceDirectory = Paths.get(resourceDirectoryLocation, modelName);
         try
         {
            Files.createDirectories(resourceDirectory);
         }
         catch (IOException e)
         {
            throw new RuntimeException(e);
         }

         ByteArrayInputStream is = new ByteArrayInputStream(resourceZip);
         ZipInputStream zip = new ZipInputStream(is);
         ZipEntry ze = null;
         try
         {
            while((ze = zip.getNextEntry()) != null)
            {
               Path target = resourceDirectory.resolve(ze.getName());
               Files.createDirectories(target.getParent());
               Files.copy(zip, target, StandardCopyOption.REPLACE_EXISTING);
            }
            zip.close();
            is.close();
         }
         catch (IOException e)
         {
            System.err.println("SDFModelLoader: Cannot load model zip file. Not unpacking robot model.");
         }

         try
         {
            ClassLoaderTools.addURLToSystemClassLoader(resourceDirectory.toUri().toURL());
         }
         catch (IntrospectionException | MalformedURLException e)
         {
            throw new RuntimeException(e);
         }
      }

      ByteArrayInputStream is = new ByteArrayInputStream(model);
      try
      {
         return new JaxbSDFLoader(is, Arrays.asList(resourceDirectories), descriptionMutator);
      }
      catch (FileNotFoundException | JAXBException e)
      {
         throw new RuntimeException(e);
      }
   }

   public String getModelName()
   {
      return modelName;
   }

   public byte[] getModel()
   {
      return model;
   }
   public String[] getResourceDirectories()
   {
      return resourceDirectories;
   }

   public byte[] getResourceZip()
   {
      return resourceZip;
   }


}
