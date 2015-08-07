package us.ihmc.darpaRoboticsChallenge.networking;

import java.io.IOException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.net.AtomicSettableTimestampProvider;
import us.ihmc.communication.net.TimestampListener;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.walking.EndOfScriptCommand;
import us.ihmc.communication.packets.walking.FootstepDataList;
import us.ihmc.humanoidBehaviors.behaviors.scripts.engine.ScriptEngineSettings;
import us.ihmc.humanoidBehaviors.behaviors.scripts.engine.ScriptFileLoader;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.utilities.TimestampProvider;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class CommandPlayer implements TimestampListener
{
   private final ExecutorService threadPool = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory("CommandPlaybackThread"));
   private final TimestampProvider timestampProvider;
   private final PacketCommunicator fieldComputerClient;
   
   private final Object syncObject = new Object();
   
   private boolean playingBack = false;
   private boolean playbackNextPacket = false;
   private final RigidBodyTransform playbackTransform = new RigidBodyTransform();
   private long startTime = Long.MIN_VALUE; 
   private long nextCommandtimestamp = Long.MIN_VALUE;
   
   private ScriptFileLoader loader;
   
   public CommandPlayer(AtomicSettableTimestampProvider timestampProvider, PacketCommunicator fieldComputerClient, IHMCCommunicationKryoNetClassList drcNetClassList)
   {
      this.timestampProvider = timestampProvider;
      this.fieldComputerClient = fieldComputerClient;
      timestampProvider.attachListener(this);
   }
   
   public void startPlayback(String filename, RigidBodyTransform playbackTransform)
   {
      synchronized (syncObject)
      {
         if(playingBack)
         {
            System.err.println("Already playing back, ignoring command");
            return;
         }
      }
      try
      {
         String fullpath = ScriptEngineSettings.scriptLoadingDirectory + filename + ScriptEngineSettings.extension;
         
         loader = new ScriptFileLoader(fullpath);
         startTime = timestampProvider.getTimestamp();
         this.playbackTransform.set(playbackTransform);
                  
         synchronized (syncObject)
         {
            nextCommandtimestamp = loader.getTimestamp();
            playingBack = true;
            playbackNextPacket = true;
         }
         System.out.println("Started playback of " + filename);
      }
      catch(IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public void timestampChanged(final long newTimestamp)
   {
      synchronized (syncObject)
      {
         if(playingBack)
         {
            if(((newTimestamp - startTime) >= nextCommandtimestamp) && playbackNextPacket)
            {
               playbackNextPacket = false;
               threadPool.execute(new Runnable()
               {
                  
                  public void run()
                  {
                     executeNewCommand(newTimestamp);
                  }
               });
            }
         }
      }
   }
   
   public void executeNewCommand(long timestamp)
   {
      try
      {
         Packet object = (Packet) loader.getObject(playbackTransform);

         boolean consumeObject = true;

         if(object instanceof EndOfScriptCommand)
         {
           consumeObject = false;
         }

         else if (object instanceof FootstepDataList)
         {
            FootstepDataList footstepDataList = (FootstepDataList) object;
         }


         if (consumeObject)
            fieldComputerClient.send(object);
         
         synchronized (syncObject)
         {
            nextCommandtimestamp = loader.getTimestamp();
            playbackNextPacket = true;
         }
       
         
      }
      catch (IOException e)
      {
         System.out.println("End of inputstream reached, stopping playback");
         synchronized (syncObject)
         {
            playingBack = false;
            playbackNextPacket = false;
         }
         
         loader.close();
      }
   }
}
