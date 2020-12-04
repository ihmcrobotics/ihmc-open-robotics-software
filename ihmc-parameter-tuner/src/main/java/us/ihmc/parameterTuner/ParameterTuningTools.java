package us.ihmc.parameterTuner;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Unmarshaller;

import javafx.application.Platform;
import javafx.scene.Node;
import javafx.scene.input.KeyCode;
import us.ihmc.parameterTuner.guiElements.GuiParameter;
import us.ihmc.parameterTuner.guiElements.GuiRegistry;
import us.ihmc.parameterTuner.guiElements.tuners.Tuner;
import us.ihmc.yoVariables.parameters.ParameterLoadStatus;
import us.ihmc.yoVariables.parameters.xml.Parameter;
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

   public static List<GuiRegistry> buildGuiRegistryFromXML(List<Registry> registries)
   {
      List<GuiRegistry> guiRegistries = new ArrayList<>();
      if (registries != null)
      {
         registries.stream().forEach(registry -> {
            try
            {
               guiRegistries.add(buildGuiRegistryFromXML(registry));
            }
            catch (Exception e)
            {
               throw new RuntimeException("Your file has valid XML structure but is not a valid parameter file. "
                     + "Likely reasons include duplicate registries and malformed identification strings. " + "Original message:\n" + "\"" + e.getMessage()
                     + "\"");
            }
         });
      }
      return guiRegistries;
   }

   public static GuiRegistry buildGuiRegistryFromXML(Registry registry)
   {
      GuiRegistry guiRoot = new GuiRegistry(registry.getName(), null);
      recursiveAddXmlToGui(guiRoot, registry);
      return guiRoot;
   }

   private static void recursiveAddXmlToGui(GuiRegistry guiRegistry, Registry xmlRegistry)
   {
      if (xmlRegistry == null)
      {
         return;
      }

      List<Parameter> xmlParameters = xmlRegistry.getParameters();
      if (xmlParameters != null)
      {
         for (Parameter xmlParameter : xmlParameters)
         {
            GuiParameter guiParameter = new GuiParameter(xmlParameter.getName(), xmlParameter.getType(), guiRegistry);
            guiParameter.setValue(xmlParameter.getValue());
            guiParameter.setMin(xmlParameter.getMin());
            guiParameter.setMax(xmlParameter.getMax());
            guiParameter.setDescription(xmlParameter.getDescription());
            guiParameter.setLoadStatus(ParameterLoadStatus.LOADED);
            guiRegistry.addParameter(guiParameter);
         }
      }

      List<Registry> xmlChildren = xmlRegistry.getRegistries();
      if (xmlChildren != null)
      {
         for (Registry xmlChild : xmlChildren)
         {
            GuiRegistry guiChild = new GuiRegistry(xmlChild.getName(), guiRegistry);
            guiRegistry.addRegistry(guiChild);
            recursiveAddXmlToGui(guiChild, xmlChild);
         }
      }
   }

   public static List<Registry> buildXMLRegistriesFromGui(List<GuiRegistry> guiRegistries)
   {
      List<Registry> xmlRegistries = new ArrayList<>();
      guiRegistries.stream().forEach(guiRegistry -> xmlRegistries.add(buildXMLRegistryFromGui(guiRegistry)));
      return xmlRegistries;
   }

   public static Registry buildXMLRegistryFromGui(GuiRegistry guiRegistry)
   {
      Registry xmlRegistry = new Registry(guiRegistry.getName());
      recursiveAddGuiToXml(xmlRegistry, guiRegistry);
      return xmlRegistry;
   }

   private static void recursiveAddGuiToXml(Registry xmlRegistry, GuiRegistry guiRegistry)
   {
      guiRegistry.getParameters().stream().forEach(guiParameter -> {
         Parameter xmlParameter = new Parameter();
         xmlParameter.setName(guiParameter.getName());
         xmlParameter.setType(guiParameter.getType());
         xmlParameter.setValue(guiParameter.getCurrentValue());
         xmlParameter.setDescription(guiParameter.getCurrentDescription());
         xmlParameter.setMin(guiParameter.getCurrentMin());
         xmlParameter.setMax(guiParameter.getCurrentMax());
         xmlRegistry.getParameters().add(xmlParameter);
      });

      guiRegistry.getRegistries().stream().forEach(guiChild -> {
         Registry xmlChild = new Registry(guiChild.getName());
         xmlRegistry.getRegistries().add(xmlChild);
         recursiveAddGuiToXml(xmlChild, guiChild);
      });
   }

   public static void addThreadSafeListeners(Node node, Runnable runnable)
   {
      addListeners(node, () -> Platform.runLater(runnable));
   }

   private static void addListeners(Node node, Runnable runnable)
   {
      node.setOnKeyPressed(event -> {
         if (event.getCode() == KeyCode.ENTER)
         {
            runnable.run();
         }
      });
      node.focusedProperty().addListener((observable, oldValue, newValue) -> {
         if (oldValue && !newValue)
         {
            runnable.run();
         }
      });
   }

   public static Map<String, Tuner> createTunerMap(List<GuiRegistry> registries)
   {
      Map<String, Tuner> tunerMap = new HashMap<>();

      List<GuiParameter> allParameters = new ArrayList<>();
      registries.stream().forEach(registry -> allParameters.addAll(registry.getAllParameters()));
      allParameters.stream().forEach(parameter -> {
         Tuner tuner = new Tuner(parameter);
         tunerMap.put(parameter.getUniqueName(), tuner);
      });

      return tunerMap;
   }

   public static void sortRegistriesAlphabetically(List<Registry> registries)
   {
      Collections.sort(registries, (r1, r2) -> r1.getName().compareTo(r2.getName()));
      registries.forEach(root -> sortRegistriesAlphabeticallyRecursive(root));
   }

   private static void sortRegistriesAlphabeticallyRecursive(Registry start)
   {
      Collections.sort(start.getRegistries(), (r1, r2) -> r1.getName().compareTo(r2.getName()));
      start.getRegistries().forEach(child -> sortRegistriesAlphabeticallyRecursive(child));
   }
}
