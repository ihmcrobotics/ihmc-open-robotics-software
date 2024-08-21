package us.ihmc.parameterTuner;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import jakarta.xml.bind.JAXBContext;
import jakarta.xml.bind.JAXBException;
import jakarta.xml.bind.Marshaller;

import javafx.stage.FileChooser;
import us.ihmc.commons.PrintTools;
import us.ihmc.parameterTuner.guiElements.GuiParameter;
import us.ihmc.parameterTuner.guiElements.GuiParameterStatus;
import us.ihmc.parameterTuner.guiElements.GuiRegistry;
import us.ihmc.yoVariables.parameters.xml.Parameters;
import us.ihmc.yoVariables.parameters.xml.Registry;

public class ParameterSavingTools
{
   public static void save(File file, List<Registry> registries) throws IOException
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

   public static FileChooser.ExtensionFilter getExtensionFilter()
   {
      return new FileChooser.ExtensionFilter("XML files (*.xml)", "*.xml");
   }

   public static List<GuiRegistry> merge(List<GuiRegistry> registriesA, List<GuiRegistry> registriesB)
   {
      Map<String, GuiRegistry> registryMapA = new LinkedHashMap<>();
      Map<String, GuiRegistry> registryMapB = new LinkedHashMap<>();
      registriesA.stream().forEach(registryA -> registryMapA.put(registryA.getUniqueName(), registryA));
      registriesB.stream().forEach(registryB -> registryMapB.put(registryB.getUniqueName(), registryB));

      Map<String, GuiRegistry> mergedRegistryMap = new LinkedHashMap<>();
      mergedRegistryMap.putAll(registryMapA);
      mergedRegistryMap.putAll(registryMapB);
      List<GuiRegistry> mergedRegistries = new ArrayList<>();
      mergedRegistryMap.values().forEach(registry -> {
         String key = registry.getUniqueName();
         GuiRegistry mergedRegistry = merge(registryMapA.get(key), registryMapB.get(key));
         mergedRegistries.add(mergedRegistry);
      });

      return mergedRegistries;
   }

   public static GuiRegistry merge(GuiRegistry registryA, GuiRegistry registryB)
   {
      if (registryA == null && registryB != null)
      {
         GuiRegistry merged = new GuiRegistry(registryB.getName(), null);
         add(registryB, merged);
         return merged;
      }
      if (registryA != null && registryB == null)
      {
         GuiRegistry merged = new GuiRegistry(registryA.getName(), null);
         add(registryA, merged);
         return merged;
      }
      if (!registryA.getUniqueName().equals(registryB.getUniqueName()))
      {
         throw new RuntimeException("Can not merge different regsitries.");
      }

      GuiRegistry merged = new GuiRegistry(registryA.getName(), null);
      merge(registryA, registryB, merged);
      return merged;
   }

   private static void merge(GuiRegistry registryA, GuiRegistry registryB, GuiRegistry merged)
   {
      if (registryA == null && registryB != null)
      {
         add(registryB, merged);
         return;
      }
      if (registryA != null && registryB == null)
      {
         add(registryA, merged);
         return;
      }
      if (!registryA.getUniqueName().equals(registryB.getUniqueName()))
      {
         throw new RuntimeException("Can not merge different regsitries.");
      }
      if (!registryA.getUniqueName().equals(merged.getUniqueName()))
      {
         throw new RuntimeException("Can not merge into different registry.");
      }

      Map<String, GuiParameter> parameterMapA = registryA.getParameterMap();
      Map<String, GuiParameter> parameterMapB = registryB.getParameterMap();
      Map<String, GuiParameter> mergedParameters = new LinkedHashMap<>();
      mergedParameters.putAll(parameterMapA);
      mergedParameters.putAll(parameterMapB);
      mergedParameters.values().forEach(parameter -> {
         GuiParameter newParameter = parameter.createCopy(merged);
         merged.addParameter(newParameter);
      });

      Map<String, GuiRegistry> registryMapA = registryA.getRegistryMap();
      Map<String, GuiRegistry> registryMapB = registryB.getRegistryMap();
      Map<String, GuiRegistry> mergedRegistries = new LinkedHashMap<>();
      mergedRegistries.putAll(registryMapA);
      mergedRegistries.putAll(registryMapB);
      mergedRegistries.values().forEach(registry -> {
         GuiRegistry mergedChild = new GuiRegistry(registry.getName(), merged);
         merged.addRegistry(mergedChild);
         String key = registry.getUniqueName();
         merge(registryMapA.get(key), registryMapB.get(key), mergedChild);
      });
   }

   private static void add(GuiRegistry registry, GuiRegistry registryToAddTo)
   {
      registry.getParameters().forEach(parameter -> {
         GuiParameter newParameter = parameter.createCopy(registryToAddTo);
         registryToAddTo.addParameter(newParameter);
      });
      registry.getRegistries().forEach(child -> {
         GuiRegistry newChild = new GuiRegistry(child.getName(), registryToAddTo);
         registryToAddTo.addRegistry(newChild);
         add(child, newChild);
      });
   }

   public static List<GuiRegistry> filterModified(List<GuiRegistry> registries)
   {
      List<GuiRegistry> filteredRegistries = new ArrayList<>();
      registries.forEach(registry -> {
         GuiRegistry filtered = filterModified(registry);
         if (!filtered.getAllParameters().isEmpty())
         {
            filteredRegistries.add(filtered);
         }
      });
      return filteredRegistries;
   }

   public static GuiRegistry filterModified(GuiRegistry registry)
   {
      return registry.createFullCopy(parameter -> parameter.getStatus() == GuiParameterStatus.MODIFIED);
   }

   public static List<GuiRegistry> findRootRegistries(List<GuiRegistry> registries, List<String> rootRegistryNames)
   {
      List<GuiRegistry> rootRegistries = new ArrayList<>();
      registries.forEach(registry -> recursiveAddMatchingRoots(registry, rootRegistryNames, rootRegistries));

      if (rootRegistries.size() != rootRegistryNames.size())
      {
         PrintTools.error("Could not find all root registries. Aborting save.");
         return Collections.emptyList();
      }

      return rootRegistries;
   }

   public static void recursiveAddMatchingRoots(GuiRegistry registry, List<String> rootRegistryNames, List<GuiRegistry> matching)
   {
      if (rootRegistryNames.contains(registry.getUniqueName()))
      {
         matching.add(registry);
      }
      else
      {
         registry.getRegistries().forEach(child -> recursiveAddMatchingRoots(child, rootRegistryNames, matching));
      }
   }
}
