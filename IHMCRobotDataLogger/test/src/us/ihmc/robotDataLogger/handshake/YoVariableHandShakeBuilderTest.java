package us.ihmc.robotDataLogger.handshake;

import static org.junit.Assert.*;

import java.util.List;
import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotDataLogger.Handshake;
import us.ihmc.robotDataLogger.HandshakeFileType;
import us.ihmc.robotDataLogger.dataBuffers.RegistrySendBufferBuilder;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoVariableHandShakeBuilderTest
{
   private static final int MAX_DEPTH = 5;

   private void generateRegistries(int depth, Random random, YoVariableRegistry parent)
   {

      int numberOfChilderen = random.nextInt(10);

      for (int c = 0; c < numberOfChilderen; c++)
      {
         int numberOfVariables = random.nextInt(50);

         YoVariableRegistry registry = new YoVariableRegistry(parent.getName() + "_" + c);
         for (int i = 0; i < numberOfVariables; i++)
         {
            new YoDouble(registry.getName() + "_" + i, registry);
         }
         parent.addChild(registry);

         if (depth < random.nextInt(MAX_DEPTH))
         {
            generateRegistries(depth + 1, random, registry);
         }
      }

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 1200000)
   public void testHandshake()
   {
      Random random = new Random(12451528l);
      YoVariableRegistry root = new YoVariableRegistry("root");
      YoVariableHandShakeBuilder handShakeBuilder = new YoVariableHandShakeBuilder(root.getName(), 0.001);

      
      YoVariableRegistry registries[] = new YoVariableRegistry[5];
      for (int r = 0; r < registries.length; r++)
      {
         registries[r] = new YoVariableRegistry("main_" + r);
         root.addChild(registries[r]);
         generateRegistries(1, random, registries[r]);

         RegistrySendBufferBuilder builder = new RegistrySendBufferBuilder(registries[r], null, null);
         handShakeBuilder.addRegistryBuffer(builder);

      }
      Handshake handshake = handShakeBuilder.getHandShake();

      IDLYoVariableHandshakeParser parser = new IDLYoVariableHandshakeParser(HandshakeFileType.IDL_YAML);
      parser.parseFrom(handshake);

      List<YoVariableRegistry> parsedRegistries = parser.getRootRegistry().getChildren();
      assertEquals(registries.length, parsedRegistries.size());
      for (int i = 0; i < parsedRegistries.size(); i++)
      {
         YoVariableRegistry original = registries[i];
         YoVariableRegistry parsed = parsedRegistries.get(i);
         assertTrue("Registries are not equal", original.areEqual(parsed));
      }

   }
}
