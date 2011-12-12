package us.ihmc.vicon;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;

import javax.media.j3d.Transform3D;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class ViconFrames
{
   protected static ViconFrames viconFramesSingleton;
   protected static final String worldFrameName = "ViconWorld";
   protected ViconClient viconClient;
   protected static HashMap<String, ViconReferenceFrame> referenceFrames;
   protected static ReferenceFrame viconWorldFrame;

   protected ViconFrames() throws Exception
   {
      initialize();
   }

   protected void initialize() throws Exception
   {
      viconWorldFrame = ReferenceFrame.constructAWorldFrame(worldFrameName);
      referenceFrames = new HashMap<String, ViconReferenceFrame>();

      viconClient = ViconClient.getInstance();
      ArrayList<String> modelNames = viconClient.getAvailableModels();

      for (String modelName : modelNames)
      {
         final String bodyName = modelName;
         System.out.println("adding frame for " + modelName);
         Transform3D transform3d = new Transform3D();
         ViconReferenceFrame referenceFrame = new ViconReferenceFrame(modelName, viconWorldFrame, transform3d, viconClient);
         referenceFrames.put(bodyName, referenceFrame);
      }

      viconClient.attachViconFrames(this);
   }

   public static ViconFrames getInstance() throws Exception
   {
      if (viconFramesSingleton == null)
      {
         viconFramesSingleton = new ViconFrames();
      }

      return viconFramesSingleton;
   }

   public ArrayList<String> getAvailableModels()
   {
      return viconClient.getAvailableModels();
   }

   protected String replaceColonWithUnderscore(String string)
   {
      return string.replace(":", "_");
   }

   public synchronized ReferenceFrame getViconWorldFrame()
   {
      return viconWorldFrame;

   }

   public void updateTransformToParent(String name)
   {
      ReferenceFrame referenceFrame = referenceFrames.get(name);
      if (referenceFrame != null)
      {
         referenceFrame.update();
      }

   }

   public boolean isDataValid(String name)
   {
      if (referenceFrames.get(name) != null)
         return referenceFrames.get(name).isDataValid();

      return false;
   }

   public synchronized ViconReferenceFrame getBodyFrame(String name)
   {
      return referenceFrames.get(name);
   }

   public Collection<ViconReferenceFrame> getFrames()
   {
      return referenceFrames.values();
   }

   public boolean isConnected()
   {
      return viconClient.isConnected();
   }

   public static void main(String[] args)
   {
      try
      {
         ViconFrames viconFrames = ViconFrames.getInstance();
         Thread.sleep(3000);
         ArrayList<String> modelNames = viconFrames.getAvailableModels();

         while (true)
         {
            ViconReferenceFrame drone = viconFrames.getBodyFrame(modelNames.get(0));

            FramePose point = new FramePose(drone);

            System.out.println(drone.isDataValid());
            System.out.println(point.changeFrameCopy(viconFrames.getViconWorldFrame()));
         }

      }
      catch (Exception e)
      {
         e.printStackTrace();
      }

   }

   public long getTimeStamp(String name)
   {
      return referenceFrames.get(name).getLastUpdateTimeStamp();
   }
}
