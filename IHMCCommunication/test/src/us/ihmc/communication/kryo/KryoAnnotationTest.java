package us.ihmc.communication.kryo;

import static org.junit.Assert.assertTrue;

import java.io.ByteArrayOutputStream;
import java.util.Random;

import org.junit.Test;

import com.esotericsoftware.kryo.Kryo;
import com.esotericsoftware.kryo.io.Output;
import com.esotericsoftware.kryo.serializers.FieldSerializer.Optional;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class KryoAnnotationTest
{
   @Test(timeout = 30000)
	@ContinuousIntegrationTest(estimatedDuration = 0.1)
   public void testOptionalAnnotation()
   {
      final int ITERATIONS = 1000;
      final boolean DEBUG = false;
      
      Kryo kryo = new Kryo();
//      kryo.getContext().put("testing", null);
      registerClasses(kryo);
      
      ByteArrayOutputStream outStream = new ByteArrayOutputStream();
      Output output = new Output(outStream);
      
      Random random = new Random(4270L);
      
      for(int i = 0; i < ITERATIONS; i++)
      {
         TestPacket packet = new TestPacket(random);
         
         kryo.writeClassAndObject(output, packet);
         output.flush();
         
         int packetLength = outStream.toByteArray().length;
         
         outStream.reset();
         output.clear();
         
         AnnotatedTestPacket annotatedPacket = new AnnotatedTestPacket(random);
         
         kryo.writeClassAndObject(output, annotatedPacket);
         output.flush();
         
         int annotatedPacketLength = outStream.toByteArray().length;
         
         outStream.reset();
         output.clear();
         
         if(DEBUG)
         {
            System.out.println("TestPacket length = " + packetLength);
            System.out.println("AnnotatedTestPacket length = " + annotatedPacketLength);
         }
         
         assertTrue(annotatedPacketLength < packetLength);
      }
   }
   
   private void registerClasses(Kryo kryo)
   {
      kryo.register(TestPacket.class);
      kryo.register(AnnotatedTestPacket.class);
      kryo.register(double[].class);
      kryo.register(long[].class);
   }
   
   public static class TestPacket
   {
      public byte b;
      public double[] doubleArray = new double[100];
      public long[] longArray = new long[100];
      
      public TestPacket()
      {
      }
      
      public TestPacket(Random random)
      {
         b = (byte) random.nextInt();
         
         for(int i = 0; i < doubleArray.length; i++)
         {
            doubleArray[i] = random.nextDouble();
         }
         
         for(int i = 0; i < longArray.length; i++)
         {
            longArray[i] = random.nextLong();
         }
      }
   }

   public static class AnnotatedTestPacket
   {
      public byte b;
      @Optional(value = "testing")
      public double[] doubleArray = new double[100];
      @Optional(value = "testing")
      public long[] longArray = new long[100];
      
      public AnnotatedTestPacket()
      {
      }
      
      public AnnotatedTestPacket(Random random)
      {
         b = (byte) random.nextInt();
         
         for(int i = 0; i < doubleArray.length; i++)
         {
            doubleArray[i] = random.nextDouble();
         }
         
         for(int i = 0; i < longArray.length; i++)
         {
            longArray[i] = random.nextLong();
         }
      }
   }
}


