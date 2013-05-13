package us.ihmc.darpaRoboticsChallenge.networking;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import javax.media.j3d.Transform3D;

import us.ihmc.darpaRoboticsChallenge.configuration.DRCNetClassList;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.net.AtomicSettableTimestampProvider;
import us.ihmc.utilities.net.KryoStreamSerializer;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.utilities.net.TimestampListener;
import us.ihmc.utilities.net.TimestampProvider;
import us.ihmc.utilities.net.TransformableDataObject;

public class CommandPlayer implements TimestampListener
{
   private final KryoStreamSerializer serializer = new KryoStreamSerializer(1048576, 1048576);
   private final ExecutorService threadPool = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory("CommandPlaybackThread"));
   private final TimestampProvider timestampProvider;
   private final ObjectCommunicator fieldComputerClient;
   
   private Object syncObject = new Object();
   
   private boolean playingBack = false;
   private FileInputStream inputStream;
   private Transform3D playbackTransform = new Transform3D();
   private long startTime = Long.MIN_VALUE; 
   private long nextCommandtimestamp = Long.MIN_VALUE;
   
   public CommandPlayer(AtomicSettableTimestampProvider timestampProvider, ObjectCommunicator fieldComputerClient, DRCNetClassList drcNetClassList)
   {
      this.timestampProvider = timestampProvider;
      this.fieldComputerClient = fieldComputerClient;
      timestampProvider.attachListener(this);
      
      serializer.registerClasses(drcNetClassList);
      serializer.registerClass(TimestampPacket.class);
   }
   
   public void startPlayback(String filename, Transform3D playbackTransform)
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
         File file = new File(CommandRecorder.directory + "/" + filename + ".script");
         
         if(!file.exists())
         {
            throw new RuntimeException("Script " + filename + " does not exist");
         }
         
         inputStream = new FileInputStream(file);
         startTime = timestampProvider.getTimestamp();
         this.playbackTransform.set(playbackTransform);
         
         getNextCommandTimestamp();
         
         synchronized (syncObject)
         {
            playingBack = true;
         }
         System.out.println("Started playback of " + filename);
      }
      catch(IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   private void getNextCommandTimestamp() throws IOException
   {
      Object timestampPacket = serializer.read(inputStream);
      if(timestampPacket instanceof TimestampPacket)
      {
         synchronized (syncObject)
         {
            nextCommandtimestamp = (((TimestampPacket) timestampPacket).getTimestamp());
         }
      }
      else
      {
         throw new RuntimeException("Invalid packet, expected TimestampPacket, got " + timestampPacket.getClass().getSimpleName());
      }
   }

   public void timestampChanged(final long newTimestamp)
   {
      synchronized (syncObject)
      {
         if(playingBack)
         {
            if(((newTimestamp - startTime) >= nextCommandtimestamp))
            {
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
         Object object = serializer.read(inputStream);
         
         Object objectToSend;
         if(object instanceof TransformableDataObject)
         {
            objectToSend = ObjectTransformationHelper.transform(playbackTransform, (TransformableDataObject) object);         
         }
         else
         {
            objectToSend = object;
         }
         
         fieldComputerClient.consumeObject(objectToSend);
         
         if(inputStream.available() > 0)
         {
            getNextCommandTimestamp();
         }
         else
         {
            System.out.println("End of inputstream reached, stopping playback");
            synchronized (syncObject)
            {
               playingBack = false;
            }
            
            inputStream.close();
         }
         
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }
}
