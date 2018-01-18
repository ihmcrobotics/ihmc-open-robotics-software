package us.ihmc.robotics.parameterGui;

import java.io.IOException;
import java.io.InputStream;
import java.util.List;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Unmarshaller;

import us.ihmc.yoVariables.parameters.xml.Parameters;
import us.ihmc.yoVariables.parameters.xml.Registry;

public class ParameterTuningTools
{
   public static List<Registry> getParameters(InputStream fileStream) throws IOException
   {
      try
      {
         JAXBContext jaxbContext = JAXBContext.newInstance(Parameters.class);
         Unmarshaller jaxbUnmarshaller = jaxbContext.createUnmarshaller();
         Parameters parameterRoot = (Parameters) jaxbUnmarshaller.unmarshal(fileStream);
         return parameterRoot.getRegistries();
      }
      catch (JAXBException e)
      {
         throw new IOException(e);
      }
   }
}
