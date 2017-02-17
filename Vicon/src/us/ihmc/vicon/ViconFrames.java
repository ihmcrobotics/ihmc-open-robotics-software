package us.ihmc.vicon;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class ViconFrames
{
   protected static ViconFrames viconFramesSingleton;
   protected static final String worldFrameName = "ViconWorld";
   protected ViconClient viconClient;
   protected static HashMap<String, ViconReferenceFrame> referenceFrames;
   protected static ReferenceFrame viconWorldFrame;

   protected ViconFrames() throws Exception
   {
      initialize(null);
   }

   protected ViconFrames(ReferenceFrame parentReferenceFrame) throws Exception
   {
      initialize(parentReferenceFrame);
   }

   protected void initialize(ReferenceFrame parentReferenceFrame) throws Exception
   {
      if (parentReferenceFrame == null)
         viconWorldFrame = ReferenceFrame.constructAWorldFrame(worldFrameName);
      else
      {
         viconWorldFrame = new ReferenceFrame(worldFrameName, parentReferenceFrame, false, true, false)
         {
            /**
             * 
             */
            private static final long serialVersionUID = 8016047182514159403L;

            protected void updateTransformToParent(RigidBodyTransform transformToParent)
            {
               setTransformToParent(transformToParent);
            }
         };
      }
         
      referenceFrames = new HashMap<String, ViconReferenceFrame>();

      viconClient = ViconClient.getInstance();
      ArrayList<String> modelNames = viconClient.getAvailableModels();

      for (String modelName : modelNames)
      {
         final String bodyName = modelName;
         System.out.println("adding frame for " + modelName);
         RigidBodyTransform transform3d = new RigidBodyTransform();
         ViconReferenceFrame referenceFrame = new ViconReferenceFrame(modelName, viconWorldFrame, transform3d, viconClient);
         referenceFrames.put(bodyName, referenceFrame);
      }

      viconClient.attachViconFrames(this);
   }

   public static ViconFrames getInstance() throws Exception
   {
      return getInstance(null);
   }
   
   public static ViconFrames getInstance(ReferenceFrame parentReferenceFrame) throws Exception
   {
      if (viconFramesSingleton == null)
      {
         viconFramesSingleton = new ViconFrames(parentReferenceFrame);
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
         ViconFrames viconFrames = ViconFrames.getInstance(null);
//         Thread.sleep(3000);
         ArrayList<String> modelNames = viconFrames.getAvailableModels();

         while (true)
         {
            for(String name : modelNames)
            {
               System.out.println("\n\n ****************************");  
               System.out.println("Model: " + name);
               ViconReferenceFrame drone = viconFrames.getBodyFrame(name);

               FramePose point = new FramePose(drone);
               point.changeFrame(viconFrames.getViconWorldFrame());

               System.out.println(drone.isDataValid());
               System.out.println(point);
            }
            
          
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
