package us.ihmc.parameterTuner.guiElements;

import java.util.ArrayList;
import java.util.List;

public class GuiRegistry extends GuiElement
{
   private final List<GuiRegistry> registries = new ArrayList<>();
   private final List<GuiParameter> parameters = new ArrayList<>();

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
   }

   public void addParameter(GuiParameter parameter)
   {
      if (!parameter.getParent().equals(this))
      {
         throw new RuntimeException("Can not add a parameter with other parent.");
      }
      parameters.add(parameter);
   }

   public List<GuiRegistry> getRegistries()
   {
      return registries;
   }

   public List<GuiParameter> getParameters()
   {
      return parameters;
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
      return createFullCopy(null);
   }

   private GuiRegistry createFullCopy(GuiRegistry parentToAttachTo)
   {
      GuiRegistry copy = new GuiRegistry(getName(), parentToAttachTo);
      getParameters().stream().forEach(parameter -> copy.addParameter(parameter.createCopy()));
      getRegistries().stream().forEach(registry -> copy.addRegistry(registry.createFullCopy(copy)));
      return copy;
   }
}
