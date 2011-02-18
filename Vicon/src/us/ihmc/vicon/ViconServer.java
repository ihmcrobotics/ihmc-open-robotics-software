package us.ihmc.vicon;

import us.ihmc.utilities.remote.ReflectiveTCPServer;

import java.util.ArrayList;

public class ViconServer extends ViconJavaInterface
{
   protected ReflectiveTCPServer tcpServer;
   protected ArrayList<String> availableModels = new ArrayList<String>();

   public ViconServer(String ip) throws Exception
   {
      if (!ViconConnect(ip))
         throw new Exception("unable to connect to Vicon at " + ip);

      // get list of available bodies
      ViconGetFrame();
      int numBodies = ViconGetNumBodies();
      System.out.println(numBodies + " available models:");

      for (int i = 0; i < numBodies; i++)
      {
         String name = ViconGetBodyName(i);
         availableModels.add(name);
         System.out.println("  " + name);
      }

      // start tcp server
      tcpServer = new ReflectiveTCPServer(this);
      tcpServer.init(7777);

      System.out.println("Vicon server started on " + tcpServer.getHost() + ": " + tcpServer.getPort());
   }

   public ArrayList<String> getAvailableModels()
   {
      return availableModels;
   }

   public Pose getPose(String modelName)
   {
      ViconGetFrame();
      return ViconGetBodyAngleAxis(modelName);
   }

   public static void main(String[] args)
   {
      String ip = "10.4.1.100";
      for (int i = 0; i < args.length - 1; i++)
      {
         if (args[i].equalsIgnoreCase("-ip"))
         {
            ip = args[i + 1];

            break;
         }
      }

      try
      {
         ViconServer viconserver = new ViconServer(ip);

//       ArrayList<String> availableModels = viconserver.getAvailableModels();
//       System.out.println("available models:\n" + availableModels);
//
//       for (String modelName : availableModels)
//       {
//          Pose pose = viconserver.getPose(modelName);
//          System.out.println("\t" + modelName + " \tat\t" + pose);
//       }

         // wait around until terminated
         synchronized (Thread.currentThread())
         {
            try
            {
               Thread.currentThread().wait();
            }
            catch (InterruptedException ex)
            {
               ex.printStackTrace();
            }
         }
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }
}
