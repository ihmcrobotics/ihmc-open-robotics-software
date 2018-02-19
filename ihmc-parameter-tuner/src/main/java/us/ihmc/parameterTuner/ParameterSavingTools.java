package us.ihmc.parameterTuner;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Marshaller;

import javafx.scene.control.Alert;
import javafx.scene.control.Alert.AlertType;
import javafx.scene.control.ButtonType;
import javafx.stage.FileChooser;
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

   public static boolean confirmSave(boolean isModified, boolean isMerge, String fileName)
   {
      String string = "Saving to " + fileName + ".";
      string += isModified ? "\nSaving modified parameters only." : "\nSaving all parameters.";
      string += isMerge ? "\nMerging parameters with existing file." : "\nOverwriting all parameters in file.";

      Alert alert = new Alert(AlertType.CONFIRMATION);
      alert.setTitle("Confirmation Dialog");
      alert.setHeaderText("Confirm Save");
      alert.setContentText(string);
      Optional<ButtonType> result = alert.showAndWait();
      return result.get() == ButtonType.OK;
   }

   public static FileChooser.ExtensionFilter getExtensionFilter()
   {
      return new FileChooser.ExtensionFilter("XML files (*.xml)", "*.xml");
   }

   public static GuiRegistry merge(GuiRegistry registryA, GuiRegistry registryB)
   {
      if (!registryA.getUniqueName().equals(registryB.getUniqueName()))
      {
         throw new RuntimeException("Can not merge different regsitries.");
      }
      if (registryA.getParent() != null)
      {
         throw new RuntimeException("Can only merge root registries.");
      }

      GuiRegistry merged = new GuiRegistry(registryA.getName(), null);
      merge(registryA, registryB, merged);
      return merged;
   }

   public static void merge(GuiRegistry registryA, GuiRegistry registryB, GuiRegistry merged)
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
      Map<String, GuiParameter> mergedParameters = new HashMap<>();
      mergedParameters.putAll(parameterMapA);
      mergedParameters.putAll(parameterMapB);
      mergedParameters.values().forEach(parameter -> {
         GuiParameter newParameter = parameter.createCopy(merged);
         merged.addParameter(newParameter);
      });

      Map<String, GuiRegistry> registryMapA = registryA.getRegistryMap();
      Map<String, GuiRegistry> registryMapB = registryB.getRegistryMap();
      Map<String, GuiRegistry> mergedRegistries = new HashMap<>();
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

   public static GuiRegistry filterModified(GuiRegistry registry)
   {
      GuiRegistry filtered = registry.createFullCopy(parameter -> parameter.getStatus() == GuiParameterStatus.MODIFIED);
      return filtered;
   }
}
