package us.ihmc.atlas;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.util.ArrayList;
import java.util.Random;

import com.thoughtworks.xstream.XStream;
import com.thoughtworks.xstream.mapper.MapperWrapper;

import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootLoadBearingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanRequestPacket;

public class XStreamTest
{
   public static class TestObject
   {
      int a = 3;
      int b = 1;
      String c = "Kldja;ls";
      int d = 10;
      
      public String toString()
      {
         return a + " " + b + " " + c + " " + d;
      }
   }
   
   
   
   public static void main(String[] args) throws IOException, ClassNotFoundException
   {
      testWritingStuff();
      
      writeObject();
      readObject();
      
   }
   
   private static void readObject() throws IOException, ClassNotFoundException
   {
      File file = new File("ChangedFileTest.xml");
      FileReader reader = new FileReader(file.getAbsoluteFile());
      XStream xStream = new XStream()
      {
         protected MapperWrapper wrapMapper(MapperWrapper next) {
            return new MapperWrapper(next) {

                public boolean shouldSerializeMember(@SuppressWarnings("rawtypes") Class definedIn, String fieldName) {
                    return definedIn != Object.class ? super.shouldSerializeMember(definedIn, fieldName) : false;
                }
                
            };
        }
      };
      ObjectInputStream in = xStream.createObjectInputStream(reader);
      Object object = in.readObject();
      System.out.println(object);
      in.close();
      reader.close();
   }
   
   private static void writeObject() throws IOException
   {
      TestObject testObject = new TestObject();
      File file = new File("ChangedFileTest.xml");
      if(!file.exists())
      {
         file.createNewFile();
      }
      
      FileWriter writer = new FileWriter(file.getAbsolutePath());
           
      XStream xStream = new XStream();
      ObjectOutputStream out = xStream.createObjectOutputStream(writer);
      out.writeObject(testObject);
      out.close();
      writer.close();
   }

   private static void testWritingStuff() throws IOException
   {
      File file = new File("XStreamTest.xml");
      if(!file.exists())
      {
         file.createNewFile();
      }
      
      FileWriter writer = new FileWriter(file.getAbsolutePath());
      
      Random random = new Random(12451L);
      
      XStream xStream = new XStream();
      ObjectOutputStream out = xStream.createObjectOutputStream(writer);
      //XXX: fix hard-coded robot model
      
      Packet<?>[] packets = new Packet<?>[10];
      
      packets[0] = new FootstepPlanRequestPacket(random);
//      packets[1] = new HandTrajectoryMessage(random);
//      packets[2] = new FootTrajectoryMessage(random);
      packets[3] = new FootLoadBearingMessage();
//      packets[4] = new HeadTrajectoryMessage(random);
      
      
      ArrayList<Object> serializedObjects = new ArrayList<Object>();
      for(int i = 0; i < 5; i++)
      {
         out.writeObject(packets[i]);
         serializedObjects.add(packets[i]);
      }
      out.close();
      writer.close();
   }

}
