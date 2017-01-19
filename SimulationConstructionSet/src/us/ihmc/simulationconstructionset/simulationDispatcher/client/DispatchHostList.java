package us.ihmc.simulationconstructionset.simulationDispatcher.client;

import java.util.ArrayList;

public class DispatchHostList extends Thread
{
   private final ArrayList<DispatchHost> dispatchHostsIn = new ArrayList<DispatchHost>();
   private final ArrayList<DispatchHost> dispatchHostsDead = new ArrayList<DispatchHost>();
   private final ArrayList<DispatchHost> dispatchHostsOut = new ArrayList<DispatchHost>();

   private String password;
   private ArrayList<String> hostNames = new ArrayList<String>();

   private ArrayList<HostsChangedListener> hostsChangedListeners = new ArrayList<HostsChangedListener>();

   public DispatchHostList(String password)
   {
      this.password = password;
      this.start();
   }

   public synchronized void addHostsChangedListener(HostsChangedListener listener)
   {
      this.hostsChangedListeners.add(listener);
   }

   public synchronized Object[] getAllHosts()
   {
      ArrayList<DispatchHost> allDispatchHosts = new ArrayList<DispatchHost>();
      allDispatchHosts.addAll(dispatchHostsIn);
      allDispatchHosts.addAll(dispatchHostsOut);
      allDispatchHosts.addAll(dispatchHostsDead);

      return allDispatchHosts.toArray();
   }

   public synchronized DispatchHost[] getAllAliveHosts()
   {
      ArrayList<DispatchHost> allAliveDispatchHosts = new ArrayList<DispatchHost>();
      allAliveDispatchHosts.addAll(dispatchHostsIn);
      allAliveDispatchHosts.addAll(dispatchHostsOut);

      DispatchHost[] ret = new DispatchHost[allAliveDispatchHosts.size()];

      allAliveDispatchHosts.toArray(ret);

      return ret;
   }

   public synchronized DispatchHost[] getAllDeadHosts()
   {
      DispatchHost[] ret = new DispatchHost[dispatchHostsDead.size()];

      dispatchHostsDead.toArray(ret);

      return ret;
   }

   public synchronized int getNumberOfHosts()
   {
      return (dispatchHostsIn.size() + dispatchHostsOut.size() + dispatchHostsDead.size());
   }

   public synchronized void registerMe(String hostName)
   {
      System.out.println(hostName + " just registered!!!");

      int slashLocation = hostName.indexOf("/");

      // System.out.println(slashLocation);

      if (slashLocation > 0)
         hostName = hostName.substring(0, slashLocation);    // +++JEP!! Need a more robust way to do this!!

      System.out.println("Extracting to " + hostName);

      if (!hostNames.contains(hostName))
      {
         addHost(hostName);
      }

      // printAllHostNames();
   }

   private synchronized void printAllHostNames()
   {
      for (int i = 0; i < hostNames.size(); i++)
      {
         String hostName = (String) hostNames.get(i);
         System.out.println(hostName);
      }

   }

   public synchronized String[] getAllRegistered()
   {
      int size = hostNames.size();

      String[] ret = new String[size];

      for (int i = 0; i < size; i++)
      {
         ret[i] = (String) hostNames.get(i);
      }

      return ret;
   }

   public synchronized void addHost(String hostName)
   {
      hostNames.add(hostName);
      addHost(new DispatchHost(hostName, password));
   }

   private synchronized void notifyListeners()
   {
      for (int i = 0; i < hostsChangedListeners.size(); i++)
      {
         ((HostsChangedListener) hostsChangedListeners.get(i)).hostsChanged();
      }
   }

   private synchronized void addHost(DispatchHost host)
   {
      dispatchHostsIn.add(host);
      notifyListeners();
   }

   public boolean isHostReady()
   {
      return (dispatchHostsIn.size() > 0);
   }

/*   public DispatchHost getNextAvailableDispatchHostAndBlock()
   {
     int index = 0;
     int size;

     while(true)
     {
       synchronized(this)
       {
         size = dispatchHostsIn.size();
         //System.out.println("Size: " + size);

         if (index >= size) index = 0;

         if (size > 0)
         {
           DispatchHost host = (DispatchHost) dispatchHostsIn.get(index);
           if (host.getRemoteSim() != null)
           {
             //System.out.println("Making sure the previous Simulation is Done."); System.out.flush();
             try
             {
               if (host.getRemoteSim().isSimulationDone(pwd))
               {
                 dispatchHostsIn.remove(host);
                 dispatchHostsOut.add(host);
                 return host;
               }
               else
               {
                 System.out.println("That's strange!  There's a simulation running on " + host.getHostName()); System.out.flush();
                 // Return it anyway...
                 dispatchHostsIn.remove(host);
                 dispatchHostsOut.add(host);
                 return host;
               }
             }
             catch(Exception e)
             {
               System.out.println(host.getHostName() + " just crashed.");
               dispatchHostsIn.remove(host);
               dispatchHostsDead.add(host);
               notifyListeners();
               //allAliveDispatchHosts.remove(host);
             }
           }
           else
           {
             System.out.println(host.getHostName() + " has null remote object.");
             dispatchHostsIn.remove(host);
             dispatchHostsDead.add(host);
             notifyListeners();
             //allAliveDispatchHosts.remove(host);
           }
         }
       }

       index++;
       if (index >= size)
       {
         try
         {
           //System.out.println("No Hosts Currently Available.  Blocking for 3 seconds");
           Thread.sleep(3000);
         }
         catch(InterruptedException e){}
       }
     }
   }
*/


   public DispatchHost getNextAvailableDispatchHost()
   {
      int index = 0;
      int size;

      while (true)
      {
         synchronized (this)
         {
            size = dispatchHostsIn.size();

            // System.out.println("Size: " + size);

            if (index >= size)
               return null;
            if (size < 1)
               return null;

            DispatchHost host = (DispatchHost) dispatchHostsIn.get(index);
            if (host.getRemoteSim() != null)
            {
               // System.out.println("Making sure the previous Simulation is Done."); System.out.flush();
               try
               {
                  if (host.getRemoteSim().isSimulationDone(password))
                  {
                     dispatchHostsIn.remove(host);
                     dispatchHostsOut.add(host);

                     return host;
                  }
                  else
                  {
                     System.out.println("That's strange!  There's a simulation running on " + host.getHostName());
                     System.out.flush();

                     // Return it anyway...
                     dispatchHostsIn.remove(host);
                     dispatchHostsOut.add(host);

                     return host;
                  }
               }
               catch (Exception e)
               {
                  System.out.println(host.getHostName() + " just crashed.");
                  dispatchHostsIn.remove(host);
                  dispatchHostsDead.add(host);
                  notifyListeners();
               }
            }
            else
            {
               System.out.println(host.getHostName() + " has null remote object.");
               dispatchHostsIn.remove(host);
               dispatchHostsDead.add(host);
               notifyListeners();
            }
         }

         index++;
      }
   }


   public synchronized void checkInDispatchHost(DispatchHost host)
   {
      // System.out.println("Checking in host: " + host.getHostName());

      dispatchHostsOut.remove(host);
      dispatchHostsIn.add(host);
      host.pingAndOrFind();
   }


   @Override
   public void run()
   {
      int index = 0;
      while (true)
      {
         int size;
         synchronized (this)    // Check on the dead hosts to see if we can revive them.  //checked in hosts to make sure they're good to go...
         {
            size = dispatchHostsDead.size();

            if (size > 0)
            {
               if (index >= size)
                  index = 0;
               DispatchHost host = (DispatchHost) dispatchHostsDead.get(index);

               host.pingAndOrFind();

               if (host.getRemoteSim() != null)
               {
                  System.out.println(host.getHostName() + " just came alive!");
                  dispatchHostsDead.remove(host);
                  dispatchHostsIn.add(host);
                  notifyListeners();
               }

               index++;
            }
         }

         try
         {
            if (size == 0)
               Thread.sleep(8000);    // Sleep for 8 seconds if no hosts.
            else if (size > 0)
               Thread.sleep(3000 / size);    // Sleep for 3 seconds total per all hosts.
         }
         catch (InterruptedException e)
         {
         }

      }

   }


}
