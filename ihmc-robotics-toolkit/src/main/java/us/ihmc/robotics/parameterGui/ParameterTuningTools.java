package us.ihmc.robotics.parameterGui;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.List;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Marshaller;
import javax.xml.bind.Unmarshaller;

import us.ihmc.yoVariables.parameters.xml.Parameters;
import us.ihmc.yoVariables.parameters.xml.Registry;

public class ParameterTuningTools
{
   public static List<Registry> getParameters(File file) throws IOException
   {
      try
      {
         JAXBContext jaxbContext = JAXBContext.newInstance(Parameters.class);
         Unmarshaller jaxbUnmarshaller = jaxbContext.createUnmarshaller();
         Parameters parameterRoot = (Parameters) jaxbUnmarshaller.unmarshal(file);
         return parameterRoot.getRegistries();
      }
      catch (JAXBException e)
      {
         throw new IOException(e);
      }
   }

   public static void save(List<Registry> registries, File file) throws IOException
   {
      Parameters parameterRoot = new Parameters();
      parameterRoot.setRegistries(registries);

      try
      {
         JAXBContext jaxbContext = JAXBContext.newInstance(Parameters.class);
         Marshaller jaxbMarshaller = jaxbContext.createMarshaller();
         jaxbMarshaller.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, true);
         FileOutputStream os = new FileOutputStream(file);
         jaxbMarshaller.marshal(parameterRoot, os);
         os.close();
      }
      catch (JAXBException e)
      {
         throw new IOException(e);
      }
      catch (FileNotFoundException e)
      {
         throw new IOException(e);
      }
   }
}
