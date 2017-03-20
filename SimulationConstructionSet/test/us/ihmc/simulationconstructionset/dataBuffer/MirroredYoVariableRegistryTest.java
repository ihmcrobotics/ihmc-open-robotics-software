package us.ihmc.simulationconstructionset.dataBuffer;

import static org.junit.Assert.*;

import org.apache.commons.lang3.StringUtils;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;


public class MirroredYoVariableRegistryTest
{
    private static final int TEST_VARIABLE_COUNT = 10;

    @ContinuousIntegrationTest(estimatedDuration = 0.0)
    @Test(timeout = 30000)
    public void testMirroredRegistryIsTheSameAsOriginalAfterCreation()
    {
        YoVariableRegistry originalRegistry = createTestRegistry("OriginalRegistry", TEST_VARIABLE_COUNT);

        MirroredYoVariableRegistry mirroredRegistry = new MirroredYoVariableRegistry(originalRegistry);

        assertTrue(areRegistryVariablesAreEqual(originalRegistry, mirroredRegistry));
    }


    @ContinuousIntegrationTest(estimatedDuration = 0.0)
    @Test(timeout = 30000)
    public void testYoVariableRegistryChildren()
    {
        YoVariableRegistry originalRegistry = createTestRegistry("OriginalRegistry", TEST_VARIABLE_COUNT);
        YoVariableRegistry childRegistry = createTestRegistry("ChildRegistry", TEST_VARIABLE_COUNT);

        originalRegistry.addChild(childRegistry);

        MirroredYoVariableRegistry mirroredRegistry = new MirroredYoVariableRegistry(originalRegistry);

        for (int i = 0; i < mirroredRegistry.getChildren().size(); i++)
        {
            assertTrue(areRegistryVariablesAreEqual(originalRegistry.getChildren().get(i), mirroredRegistry.getChildren().get(i)));
        }

    }

    @ContinuousIntegrationTest(estimatedDuration = 0.0)
    @Test(timeout = 30000)
    public void testChangesArePropagatedFromOriginal()
    {
        YoVariableRegistry originalRegistry = createTestRegistry("OriginalRegistry", TEST_VARIABLE_COUNT);
        MirroredYoVariableRegistry mirroredYoVariableRegistry = new MirroredYoVariableRegistry(originalRegistry);

        for (YoVariable<?> yoVariable : originalRegistry.getAllVariables())
        {
            yoVariable.setValueFromDouble(1.0);
        }

        // Should *not* be equal until updateValuesFromOriginal or updateMirror is called
        assertFalse(areRegistryVariablesAreEqual(originalRegistry, mirroredYoVariableRegistry));

        mirroredYoVariableRegistry.updateValuesFromOriginal();

        assertTrue(areRegistryVariablesAreEqual(originalRegistry, mirroredYoVariableRegistry));
    }

    @ContinuousIntegrationTest(estimatedDuration = 0.0)
    @Test(timeout = 30000)
    public void testChangesArePropagatedFromMirror()
    {
        YoVariableRegistry originalRegistry = createTestRegistry("OriginalRegistry", TEST_VARIABLE_COUNT);
        MirroredYoVariableRegistry mirroredYoVariableRegistry = new MirroredYoVariableRegistry(originalRegistry);

        for (YoVariable<?> yoVariable : mirroredYoVariableRegistry.getAllVariables())
        {
            yoVariable.setValueFromDouble(2.0);
        }

        // Should *not* be equal until updateChangedValues or updateMirror is called
        assertFalse(areRegistryVariablesAreEqual(originalRegistry, mirroredYoVariableRegistry));

        mirroredYoVariableRegistry.updateChangedValues();

        assertTrue(areRegistryVariablesAreEqual(originalRegistry, mirroredYoVariableRegistry));
    }

    @ContinuousIntegrationTest(estimatedDuration = 0.0)
    @Test(timeout = 30000)
    public void testMirrorValuesArePreferredWhenConflict()
    {
        YoVariableRegistry originalRegistry = createTestRegistry("OriginalRegistry", TEST_VARIABLE_COUNT);
        MirroredYoVariableRegistry mirroredYoVariableRegistry = new MirroredYoVariableRegistry(originalRegistry);

        final double newValueForOriginal = 2.0;
        final double newValueForMirror = 3.0;

        for (YoVariable<?> yoVariable : originalRegistry.getAllVariables())
        {
            yoVariable.setValueFromDouble(newValueForOriginal);
        }

        for (YoVariable<?> yoVariable : mirroredYoVariableRegistry.getAllVariables())
        {
            yoVariable.setValueFromDouble(newValueForMirror);
        }

        // Should *not* be equal until updateMirror is called
        assertFalse(areRegistryVariablesAreEqual(originalRegistry, mirroredYoVariableRegistry));

        mirroredYoVariableRegistry.updateMirror();

        assertTrue(areRegistryVariablesAreEqual(originalRegistry, mirroredYoVariableRegistry));

        for (YoVariable<?> yoVariable : mirroredYoVariableRegistry.getAllVariables())
        {
            assertEquals(yoVariable.getValueAsDouble(), newValueForMirror, 1e-10);
        }
    }

    @ContinuousIntegrationTest(estimatedDuration = 0.0)
    @Test(timeout = 30000)
    public void testOriginalListenersAreCalledWhenMirrorChanges()
    {
        YoVariableRegistry originalRegistry = createTestRegistry("OriginalRegistry", TEST_VARIABLE_COUNT);
        MirroredYoVariableRegistry mirroredYoVariableRegistry = new MirroredYoVariableRegistry(originalRegistry);

        ListenerCounter listenerCounter = new ListenerCounter();
        for (YoVariable<?> yoVariable : originalRegistry.getAllVariables())
        {
            yoVariable.addVariableChangedListener(listenerCounter);
        }

        for (YoVariable<?> yoVariable : mirroredYoVariableRegistry.getAllVariables())
        {
            yoVariable.setValueFromDouble(1.0);
        }

        mirroredYoVariableRegistry.updateMirror();

        assertEquals(listenerCounter.callCount, originalRegistry.getAllVariables().size());
    }

    @ContinuousIntegrationTest(estimatedDuration = 0.0)
    @Test(timeout = 30000)
    public void testMirrorListenersAreCalledWhenOriginalChanges()
    {
        YoVariableRegistry originalRegistry = createTestRegistry("OriginalRegistry", TEST_VARIABLE_COUNT);
        MirroredYoVariableRegistry mirroredYoVariableRegistry = new MirroredYoVariableRegistry(originalRegistry);

        ListenerCounter listenerCounter = new ListenerCounter();
        for (YoVariable<?> yoVariable : mirroredYoVariableRegistry.getAllVariables())
        {
            yoVariable.addVariableChangedListener(listenerCounter);
        }

        for (YoVariable<?> yoVariable : originalRegistry.getAllVariables())
        {
            yoVariable.setValueFromDouble(1.0);
        }

        mirroredYoVariableRegistry.updateMirror();

        assertEquals(listenerCounter.callCount, mirroredYoVariableRegistry.getAllVariables().size());
    }

    private static YoVariableRegistry createTestRegistry(String name, int variableCount)
    {
        YoVariableRegistry registry = new YoVariableRegistry(name);
        for (int i = 0; i < variableCount; i++)
        {
            new DoubleYoVariable("Variable" + i, registry);
        }
        return registry;
    }

    private static boolean areRegistryVariablesAreEqual(YoVariableRegistry registry1, YoVariableRegistry registry2)
    {
        if (registry1.getAllVariables().size() != registry2.getAllVariables().size())
            return false;

        for (int i = 0; i < registry1.getAllVariables().size(); i++)
        {
            YoVariable<?> original = registry1.getAllVariables().get(i);
            YoVariable<?> copy = registry2.getAllVariables().get(i);
            if (!areYoVariablesEqual(original, copy))
            {
                return false;
            }
        }
        return true;
    }


    private static boolean areYoVariablesEqual(YoVariable<?> var1, YoVariable<?> var2)
    {
        return StringUtils.equals(var1.getName(), var2.getName())  &&
                var1.getValueAsDouble() == var2.getValueAsDouble();
    }

    private static class ListenerCounter implements VariableChangedListener
    {
        public int callCount = 0;


        @Override
        public void variableChanged(YoVariable<?> v)
        {
            ++callCount;
        }
    }
}