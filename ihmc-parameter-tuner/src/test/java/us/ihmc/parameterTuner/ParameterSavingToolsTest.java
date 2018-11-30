package us.ihmc.parameterTuner;

import java.util.List;

import org.junit.Assert;
import org.junit.Test;

import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.parameterTuner.guiElements.GuiParameter;
import us.ihmc.parameterTuner.guiElements.GuiRegistry;

public class ParameterSavingToolsTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMerge()
   {
      /**
       * Registry A will be
       * root
       *    - parameter_1 = A
       *    - parameter_2 = A
       *    - parameter_3 = A
       *    child1
       *       - parameter_1_1 = A
       *       - parameter_1_2 = A
       *       - parameter_1_3 = A
       *       child11
       *          child111
       *             - parameter_111_1 = A
       *             - paramerer_111_2 = A
       *             - parameter_111_3 = A
       *       child12
       *          - parameter_12_1 = A
       *          - parameter_12_2 = A
       *    child2
       *       - parameter_2_1 = A
       *       - parameter_2_2 = A
       *       - parameter_2_3 = A
       */
      GuiRegistry rootA = new GuiRegistry("root", null);
      {
         createParameterInRegistry(rootA, "parameter_1", "A");
         createParameterInRegistry(rootA, "parameter_2", "A");
         createParameterInRegistry(rootA, "parameter_3", "A");
         GuiRegistry child1 = createRegistryInRegistry(rootA, "child1");
         createParameterInRegistry(child1, "parameter_1_1", "A");
         createParameterInRegistry(child1, "parameter_1_2", "A");
         createParameterInRegistry(child1, "parameter_1_3", "A");
         GuiRegistry child11 = createRegistryInRegistry(child1, "child11");
         GuiRegistry child111 = createRegistryInRegistry(child11, "child111");
         createParameterInRegistry(child111, "parameter_111_1", "A");
         createParameterInRegistry(child111, "parameter_111_2", "A");
         createParameterInRegistry(child111, "parameter_111_3", "A");
         GuiRegistry child12 = createRegistryInRegistry(child1, "child12");
         createParameterInRegistry(child12, "parameter_12_1", "A");
         createParameterInRegistry(child12, "parameter_12_2", "A");
         GuiRegistry child2 = createRegistryInRegistry(rootA, "child2");
         createParameterInRegistry(child2, "parameter_2_1", "A");
         createParameterInRegistry(child2, "parameter_2_2", "A");
         createParameterInRegistry(child2, "parameter_2_3", "A");
      }

      /**
       * Registry B will be
       * root
       *    - parameter_2 = B
       *    - parameter_4 = B
       *    child1
       *       child11
       *          - parameter_11_1 = B
       *       child12
       *          - parameter_12_3 = B
       *    child3
       *       - parameter_3_1 = B
       *       - parameter_3_2 = B
       */
      GuiRegistry rootB = new GuiRegistry("root", null);
      {
         createParameterInRegistry(rootB, "parameter_2", "B");
         createParameterInRegistry(rootB, "parameter_4", "B");
         GuiRegistry child1 = createRegistryInRegistry(rootB, "child1");
         GuiRegistry child11 = createRegistryInRegistry(child1, "child11");
         createParameterInRegistry(child11, "parameter_11_1", "B");
         GuiRegistry child12 = createRegistryInRegistry(child1, "child12");
         createParameterInRegistry(child12, "parameter_12_1", "B");
         GuiRegistry child3 = createRegistryInRegistry(rootB, "child3");
         createParameterInRegistry(child3, "parameter_3_1", "B");
         createParameterInRegistry(child3, "parameter_3_2", "B");
      }

      /**
       * The expected result of merging B into A is:
       * root
       *    - parameter_1 = A
       *    - parameter_2 = B
       *    - parameter_3 = A
       *    - parameter_4 = B
       *    child1
       *       - parameter_1_1 = A
       *       - parameter_1_2 = A
       *       - parameter_1_3 = A
       *       child11
       *          - parameter_11_1 = B
       *          child111
       *             - parameter_111_1 = A
       *             - paramerer_111_2 = A
       *             - parameter_111_3 = A
       *       child12
       *          - parameter_12_1 = B
       *          - parameter_12_2 = A
       *    child2
       *       - parameter_2_1 = A
       *       - parameter_2_2 = A
       *       - parameter_2_3 = A
       *    child3
       *       - parameter_3_1 = B
       *       - parameter_3_2 = B
       */
      GuiRegistry expected = new GuiRegistry("root", null);
      {
         createParameterInRegistry(expected, "parameter_1", "A");
         createParameterInRegistry(expected, "parameter_2", "B");
         createParameterInRegistry(expected, "parameter_3", "A");
         createParameterInRegistry(expected, "parameter_4", "B");
         GuiRegistry child1 = createRegistryInRegistry(expected, "child1");
         createParameterInRegistry(child1, "parameter_1_1", "A");
         createParameterInRegistry(child1, "parameter_1_2", "A");
         createParameterInRegistry(child1, "parameter_1_3", "A");
         GuiRegistry child11 = createRegistryInRegistry(child1, "child11");
         createParameterInRegistry(child11, "parameter_11_1", "B");
         GuiRegistry child111 = createRegistryInRegistry(child11, "child111");
         createParameterInRegistry(child111, "parameter_111_1", "A");
         createParameterInRegistry(child111, "parameter_111_2", "A");
         createParameterInRegistry(child111, "parameter_111_3", "A");
         GuiRegistry child12 = createRegistryInRegistry(child1, "child12");
         createParameterInRegistry(child12, "parameter_12_1", "B");
         createParameterInRegistry(child12, "parameter_12_2", "A");
         GuiRegistry child2 = createRegistryInRegistry(expected, "child2");
         createParameterInRegistry(child2, "parameter_2_1", "A");
         createParameterInRegistry(child2, "parameter_2_2", "A");
         createParameterInRegistry(child2, "parameter_2_3", "A");
         GuiRegistry child3 = createRegistryInRegistry(expected, "child3");
         createParameterInRegistry(child3, "parameter_3_1", "B");
         createParameterInRegistry(child3, "parameter_3_2", "B");
      }

      GuiRegistry actual = ParameterSavingTools.merge(rootA, rootB);

      if (!checkEqual(expected, actual))
      {
         printRegistry(rootA, "Registry A");
         printRegistry(rootB, "Registry B");
         printRegistry(expected, "Expected Result");
         printRegistry(actual, "Actual Result");
         Assert.fail("The merged registry did not match the expected. See console output for results.");
      }
   }

   private static boolean checkEqual(GuiRegistry registryA, GuiRegistry registryB)
   {
      if (!registryA.getName().equals(registryB.getName()))
      {
         return false;
      }

      List<GuiParameter> parametersA = registryA.getParameters();
      List<GuiParameter> parametersB = registryB.getParameters();
      if (parametersA.size() != parametersB.size())
      {
         return false;
      }
      for (int paramIdx = 0; paramIdx < parametersA.size(); paramIdx++)
      {
         GuiParameter parameterA = parametersA.get(paramIdx);
         GuiParameter parameterB = parametersB.get(paramIdx);
         if (!parameterA.getName().equals(parameterB.getName()))
         {
            return false;
         }
         if (!parameterA.getCurrentValue().equals(parameterB.getCurrentValue()))
         {
            return false;
         }
      }

      List<GuiRegistry> registriesA = registryA.getRegistries();
      List<GuiRegistry> registriesB = registryB.getRegistries();
      if (registriesA.size() != registriesB.size())
      {
         return false;
      }
      for (int regIdx = 0; regIdx < registriesA.size(); regIdx++)
      {
         GuiRegistry childA = registriesA.get(regIdx);
         GuiRegistry childB = registriesB.get(regIdx);
         if (!checkEqual(childA, childB))
         {
            return false;
         }
      }

      return true;
   }

   private static GuiParameter createParameterInRegistry(GuiRegistry registry, String parameterName, String value)
   {
      GuiParameter newParameter = new GuiParameter(parameterName, "", registry);
      newParameter.setValue(value);
      registry.addParameter(newParameter);
      return newParameter;
   }

   private static GuiRegistry createRegistryInRegistry(GuiRegistry registry, String registryName)
   {
      GuiRegistry newRegistry = new GuiRegistry(registryName, registry);
      registry.addRegistry(newRegistry);
      return newRegistry;
   }

   private static void printRegistry(GuiRegistry rootA, String infoString)
   {
      PrintTools.info(infoString);
      printRecursive(rootA, "");
   }

   private static void printRecursive(GuiRegistry registry, String prefix)
   {
      System.out.println(prefix + registry.getName());
      registry.getParameters().stream().forEach(parameter -> System.out.println(prefix + "   - " + parameter.getName() + ": " + parameter.getCurrentValue()));
      registry.getRegistries().stream().forEach(child -> printRecursive(child, prefix + "   "));
   }
}
