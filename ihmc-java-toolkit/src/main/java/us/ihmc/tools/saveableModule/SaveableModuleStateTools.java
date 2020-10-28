package us.ihmc.tools.saveableModule;

import us.ihmc.tools.io.WorkspacePathTools;
import us.ihmc.yoVariables.euclid.YoTuple2D;
import us.ihmc.yoVariables.euclid.YoTuple3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.parameters.ParameterData;
import us.ihmc.yoVariables.parameters.YoParameter;
import us.ihmc.yoVariables.parameters.xml.Parameter;
import us.ihmc.yoVariables.parameters.xml.Parameters;
import us.ihmc.yoVariables.variable.YoVariable;

import javax.swing.*;
import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Marshaller;
import javax.xml.bind.Unmarshaller;
import java.io.*;
import java.nio.file.Path;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class SaveableModuleStateTools
{
   private static final Path rootPath = WorkspacePathTools.handleWorkingDirectoryFuzziness("ihmc-open-robotics-software");
   private static final SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmssSSS");
   private static final String testFormatPostfix = "_Log";

   public static void registerYoTuple3DToSave(YoTuple3D framePoint3D, SaveableModuleState state)
   {
      state.registerVariableToSave(framePoint3D.getYoX());
      state.registerVariableToSave(framePoint3D.getYoY());
      state.registerVariableToSave(framePoint3D.getYoZ());
   }

   public static void registerYoTuple2DToSave(YoTuple2D framePoint2D, SaveableModuleState state)
   {
      state.registerVariableToSave(framePoint2D.getYoX());
      state.registerVariableToSave(framePoint2D.getYoY());
   }

   public static void registerYoFrameQuaternionToSave(YoFrameQuaternion frameQuaternion, SaveableModuleState state)
   {
      state.registerVariableToSave(frameQuaternion.getYoQs());
      state.registerVariableToSave(frameQuaternion.getYoQx());
      state.registerVariableToSave(frameQuaternion.getYoQy());
      state.registerVariableToSave(frameQuaternion.getYoQz());
   }

   public static void registerYoFramePose3DToSave(YoFramePose3D framePose3D, SaveableModuleState state)
   {
      registerYoTuple3DToSave(framePose3D.getPosition(), state);
      registerYoFrameQuaternionToSave(framePose3D.getOrientation(), state);
   }

   public static void save(String moduleName, SaveableModuleState state)
   {
      JFileChooser fileChooser = new JFileChooser();
      Path directory = rootPath;
      File logDirectory = new File(directory.toString());
      SaveableModuleTools.ensureFileExists(logDirectory);
      File fileToSave = new File(moduleName + dateFormat.format(new Date()) + testFormatPostfix + File.separator);

      fileChooser.setCurrentDirectory(logDirectory);
      fileChooser.setFileSelectionMode(JFileChooser.FILES_ONLY);
      fileChooser.setSelectedFile(fileToSave);
      int chooserState = fileChooser.showOpenDialog(null);

      if (chooserState != JFileChooser.APPROVE_OPTION)
         return;

      File file = fileChooser.getSelectedFile();

      save(file, state);
   }

   public static void save(File fileToSaveTo, SaveableModuleState stateToSave)
   {
      if (fileToSaveTo == null)
         throw new IllegalArgumentException("File has not been set.");
      if (stateToSave == null)
         throw new IllegalArgumentException("State has not been set.");

      SaveableModuleTools.ensureFileExists(fileToSaveTo);

      try
      {
         FileOutputStream os = new FileOutputStream(fileToSaveTo);
         writeStream(os, stateToSave);
         os.close();
      }
      catch (IOException ex)
      {
         throw new RuntimeException("Problem when saving module.");
      }
   }

   public static void load(SaveableModuleState state) throws NoSuchFieldException, IllegalAccessException
   {
      JFileChooser fileChooser = new JFileChooser();
      Path directory = rootPath;
      File logDirectory = new File(directory.toString());
      SaveableModuleTools.ensureFileExists(logDirectory);

      fileChooser.setCurrentDirectory(logDirectory);
      fileChooser.setFileSelectionMode(JFileChooser.FILES_ONLY);
      int chooserState = fileChooser.showOpenDialog(null);

      if (chooserState != JFileChooser.APPROVE_OPTION)
         return;

      File file = fileChooser.getSelectedFile();

      load(file, state);
   }

   public static void load(File fileToLoad, SaveableModuleState stateToLoad) throws NoSuchFieldException, IllegalAccessException
   {
      if (fileToLoad == null)
         throw new IllegalArgumentException("File has not been set.");
      if (stateToLoad == null)
         throw new IllegalArgumentException("State has not been set.");

      try
      {
         InputStream inputStream = new FileInputStream(fileToLoad);
         stateToLoad.loadValues(readStream(inputStream));
         inputStream.close();
      }
      catch (IOException ex)
      {
         throw new RuntimeException("Problem when saving module.");
      }
   }

   private static Map<String, ParameterData> readStream(InputStream inputStream) throws IOException
   {
      Map<String, ParameterData> parameterValues = new HashMap<>();

      try
      {
         JAXBContext jaxbContext = JAXBContext.newInstance(SaveableRegistry.class);
         Unmarshaller jaxbUnmarshaller = jaxbContext.createUnmarshaller();

         SaveableRegistry registry = (SaveableRegistry) jaxbUnmarshaller.unmarshal(inputStream);
         if (registry.getParameters() != null)
         {
            for (Parameter param : registry.getParameters())
            {
               String name = param.getName();
               ParameterData data = new ParameterData(param.getValue(), param.getMin(), param.getMax());
               if (parameterValues.put(name, data) != null)
               {
                  throw new IllegalArgumentException("Invalid file, more than one entry for parameter " + name);
               }
            }
         }
      }
      catch (JAXBException e)
      {
         throw new IOException(e);
      }

      return parameterValues;
   }

   public static void writeStream(OutputStream outputStream, SaveableModuleState stateToSave) throws IOException
   {
      try
      {
         JAXBContext jaxbContext = JAXBContext.newInstance(SaveableRegistry.class);
         Marshaller jaxbMarshaller = jaxbContext.createMarshaller();

         jaxbMarshaller.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, true);

         SaveableRegistry registry = new SaveableRegistry(stateToSave.getClass().getSimpleName());
         List<YoParameter> parameters = stateToSave.getParametersToSave();
         List<YoVariable> variables = stateToSave.getVariablesToSave();

         for (int i = 0; i < parameters.size(); i++)
            addParameter(parameters.get(i), registry);
         for (int i = 0; i < variables.size(); i++)
            addVariable(variables.get(i), registry);

         jaxbMarshaller.marshal(registry, outputStream);
      }
      catch (JAXBException e)
      {
         throw new IOException(e);
      }
   }

   private static void addParameter(YoParameter parameter, SaveableRegistry registryToPack)
   {
      String name = parameter.getName();
      String type = parameter.getClass().getSimpleName();
      String value = parameter.getValueAsString();
      String min = String.valueOf(0.0);
      String max = String.valueOf(1.0);

      Parameter newParameter = new Parameter(name, type, value, min, max);
      registryToPack.getParameters().add(newParameter);
   }

   private static void addVariable(YoVariable parameter, SaveableRegistry registryToPack)
   {
      String name = parameter.getName();
      String type = parameter.getClass().getSimpleName();
      String value = parameter.getValueAsString();
      String min = String.valueOf(0.0);
      String max = String.valueOf(1.0);

      Parameter newParameter = new Parameter(name, type, value, min, max);
      registryToPack.getParameters().add(newParameter);
   }
}
