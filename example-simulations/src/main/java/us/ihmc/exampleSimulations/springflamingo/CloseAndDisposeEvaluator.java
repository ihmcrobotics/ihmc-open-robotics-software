package us.ihmc.exampleSimulations.springflamingo;

import java.util.ArrayList;
import java.util.Map;
import java.util.Set;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JPanel;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.commons.thread.ThreadTools;

public class CloseAndDisposeEvaluator
{
   public static void main(String[] args)
   {
//      tryWithSCS();
      
      tryWithSwingComponents();
   }
   
   private static void tryWithSCS()
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("CloseAndDisposeEvaluator"));

      scs.startOnAThread();
      ThreadTools.sleep(1000);

      scs.closeAndDispose();
      scs = null;

      System.gc();
      ThreadTools.sleepForever();
   }
   
   public static void tryWithSwingComponents()
   {
      JFrame frame = new JFrame("");
      frame.setSize(900, 400);
      frame.setVisible(true);
      
//      Dialog foo = new Dialog(frame, "Hello");
//      foo.setSize(600,  600);
//      foo.setVisible(true);
      
//       JFileChooser fileChooser = new JFileChooser(".");
//       fileChooser.setSize(400, 400);
//       fileChooser.setVisible(true);
//       int value = fileChooser.showOpenDialog(null);

      JPanel jPanel = new JPanel();
      
      JButton jButton = new JButton("Push Me");
      jPanel.add(jButton);
      
      frame.getContentPane().add(jPanel);
      
      ThreadTools.sleep(4000);
      
//      ThreadTools.sleepForever();

//      foo.setVisible(false);
//      foo.dispose();
      
//      fileChooser.setVisible(false);
//      fileChooser = null;
      
      frame.setVisible(false);
      frame.dispose();
      
      System.gc();
      ThreadTools.sleep(1000);

      Map<Thread, StackTraceElement[]> allStackTraces = Thread.getAllStackTraces();
      Set<Thread> threads = allStackTraces.keySet();
      
      ArrayList<Thread> liveThreads = new ArrayList<Thread>();
      liveThreads.addAll(threads);
      ArrayList<Thread> deadThreads = new ArrayList<Thread>();
      
      ArrayList<Thread> threadsThatDied = new ArrayList<Thread>();

      while(true)
      {
         threadsThatDied.clear();
         
         System.out.println("\nDead Threads:");
         for(Thread thread : deadThreads)
         {
            System.out.println(thread.getName() + " isDaemon = " + thread.isDaemon());
         }
         
         System.out.println("\nLive Threads:");
         for (Thread thread : liveThreads)
         {
            String name = thread.getName();
            boolean daemon = thread.isDaemon();
            boolean alive = thread.isAlive();
            if (!alive) threadsThatDied.add(thread);
            
            System.out.println(name + " isDaemon = " + daemon + ", isAlive = " + alive);
            
//            if (!name.contains("main"))
//            {
//               thread.interrupt();
//            }
           
         }
         
         for (Thread thread : threadsThatDied)
         {
            liveThreads.remove(thread);
            deadThreads.add(thread);
         }
         
         System.gc();      
         ThreadTools.sleep(1000);
      }
      
//      ThreadTools.sleepForever();
   }
}
