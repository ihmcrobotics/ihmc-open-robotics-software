package us.ihmc.parameterTuner.guiElements;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Predicate;

public class GuiRegistry extends GuiElement
{
   private final List<GuiRegistry> registries = new ArrayList<>();
   private final List<GuiParameter> parameters = new ArrayList<>();

   private final Map<String, GuiRegistry> registryMap = new HashMap<>();
   private final Map<String, GuiParameter> parameterMap = new HashMap<>();

   public GuiRegistry(String name, GuiRegistry parent)
   {
      super(name, parent);
   }

   public void addRegistry(GuiRegistry registry)
   {
      if (!registry.getParent().equals(this))
      {
         throw new RuntimeException("Can not add a parameter with other parent.");
      }
      registries.add(registry);
      if (registryMap.put(registry.getUniqueName(), registry) != null)
      {
         throw new RuntimeException("Already have registry " + registry.getName());
      }
   }

   public void addParameter(GuiParameter parameter)
   {
      if (!parameter.getParent().equals(this))
      {
         throw new RuntimeException("Can not add a parameter with other parent.");
      }
      parameters.add(parameter);
      if (parameterMap.put(parameter.getUniqueName(), parameter) != null)
      {
         throw new RuntimeException("Already have parameter " + parameter.getName());
      }
   }

   public List<GuiRegistry> getRegistries()
   {
      return registries;
   }

   public List<GuiParameter> getParameters()
   {
      return parameters;
   }

   public Map<String, GuiRegistry> getRegistryMap()
   {
      return registryMap;
   }

   public Map<String, GuiParameter> getParameterMap()
   {
      return parameterMap;
   }

   public List<GuiParameter> getAllParameters()
   {
      List<GuiParameter> ret = new ArrayList<>();
      packParametersRecursive(ret);
      return ret;
   }

   private void packParametersRecursive(List<GuiParameter> listToPack)
   {
      listToPack.addAll(parameters);
      registries.stream().forEach(registry -> registry.packParametersRecursive(listToPack));
   }

   public GuiRegistry createFullCopy()
   {
      return createFullCopy(p -> true);
   }

   public GuiRegistry createFullCopy(Predicate<GuiParameter> predicate)
   {
      return createFullCopy(null, predicate);
   }

   private GuiRegistry createFullCopy(GuiRegistry parentToAttachTo, Predicate<GuiParameter> predicate)
   {
      GuiRegistry copy = new GuiRegistry(getName(), parentToAttachTo);
      getParameters().stream().filter(predicate).forEach(parameter -> copy.addParameter(parameter.createCopy(copy)));
      getRegistries().stream().forEach(registry -> {
         GuiRegistry childCopy = registry.createFullCopy(copy, predicate);
         if (!childCopy.getAllParameters().isEmpty())
         {
            copy.addRegistry(childCopy);
         }
      });
      return copy;
   }

   @Override
   public String toString()
   {
      String ret = getUniqueName();
      ret += "\n" + parameters.size() + " parameters";
      ret += "\n" + registries.size() + " registries";
      return ret;
   }
}
