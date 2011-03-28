package us.ihmc.vicon;

import java.util.HashMap;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class ViconFrames
{

   private static ViconFrames viconFrames = new ViconFrames();
   private static ReferenceFrame worldFrame;
   private static HashMap<String, ReferenceFrame> bodyFrames = new HashMap<String, ReferenceFrame>();

   public ViconFrames()
   {
      worldFrame = ReferenceFrame.getWorldFrame();
   }

   public static ReferenceFrame getWorldFrame()
   {
      return worldFrame;
   }

   public static void updateTransformToParent(String name, Transform3D transform3d)
   {
      if (bodyFrames.containsKey(name))
      {
         //update it
         bodyFrames.get(name).setTransformToParent(transform3d);
      } else
      {
         //Create

         bodyFrames.put(name, new ReferenceFrame(name, worldFrame, transform3d, false, false, false)
         {
            private static final long serialVersionUID = -9160732749609839626L;

            public void updateTransformToParent(Transform3D transformToParent)
            {
               throw new RuntimeException("do not call the update method on vicon frames");
            }
         });
      }
   }

   public static ReferenceFrame getBodyFrame(String name)
   {
      return bodyFrames.get(name);
   }
}
