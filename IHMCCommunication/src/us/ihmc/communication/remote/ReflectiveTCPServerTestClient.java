package us.ihmc.communication.remote;

import java.io.Serializable;
import java.util.Vector;

public class ReflectiveTCPServerTestClient extends RemoteConnection
{
   public void testVoidMethod()
   {
      Vector<Serializable> parameters = new Vector<Serializable>();
      RemoteRequest remoteRequest = new RemoteRequest("voidMethod", parameters);

      try
      {
         SendObject(remoteRequest);
      }
      catch (Exception ex)
      {
         ex.printStackTrace();
      }
   }

   public String testStringMethod()
   {
      Vector<Serializable> parameters = new Vector<Serializable>();
      RemoteRequest remoteRequest = new RemoteRequest("stringMethod", parameters);

      try
      {
         String result = (String) SendRequest(remoteRequest);

         return result;
      }
      catch (Exception ex)
      {
         ex.printStackTrace();
      }

      return null;
   }

   public String testgetPose(String name)
   {
      Vector<Serializable> parameters = new Vector<Serializable>();
      parameters.add(name);
      RemoteRequest remoteRequest = new RemoteRequest("getPose", parameters);

      try
      {
         String result = (String) SendRequest(remoteRequest);

         return result;
      }
      catch (Exception ex)
      {
         ex.printStackTrace();
      }

      return null;
   }

   public static void main(String[] args)
   {
      ReflectiveTCPServerTestClient client = new ReflectiveTCPServerTestClient();
      try
      {
//       client.connect("10.100.0.149");
         client.connect("10.100.0.22");

         long start = System.currentTimeMillis();
         int count = 0;
         while (count < 50)
         {
            client.testVoidMethod();
            count++;
         }

         long now = System.currentTimeMillis();
         System.out.println("voidMethod read " + count + " in " + (now - start) + "(" + (count * 1000 / (now - start)) + ")");
         start = now;
         count = 0;

         while (count < 50)
         {
            client.testStringMethod();
            count++;
         }

         now = System.currentTimeMillis();
         System.out.println("stringMethod read " + count + " in " + (now - start) + "(" + (count * 1000 / (now - start)) + ")");
         start = now;
         count = 0;

         while (count < 50)
         {
            client.testgetPose("bodyname");
            count++;
         }

         now = System.currentTimeMillis();
         System.out.println("getPose read " + count + " in " + (now - start) + "(" + (count * 1000 / (now - start)) + ")");
         start = now;
         count = 0;
      }
      catch (Exception ex1)
      {
         ex1.printStackTrace();
      }
   }
}
