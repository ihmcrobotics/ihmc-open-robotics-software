package us.ihmc.vicon;

import java.util.HashMap;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class ViconFrames
{
   private static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static HashMap<String, ReferenceFrame> bodyFrames = new HashMap<String, ReferenceFrame>();
   private static HashMap<String, ReferenceFrame> cameraFrames = new HashMap<String, ReferenceFrame>();

   public ViconFrames()
   {
   }

   public static ReferenceFrame getWorldFrame()
   {
      return worldFrame;
   }

   /**
    * Note: The name parameter has "_body" concatenated to it within this method. There is also a camera frame that is created and given the name: name+"_body_camera".
    * @param name - the name of the reference frame that is being updated
    * @param transform3d - the transform from the parent to the named reference frame
    */
   public static void updateTransformToParent(String name, Transform3D transform3d)
   {
      name.concat("_body");
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

         name.concat("_camera");
         Transform3D cameraTransform3d = new Transform3D();
         cameraTransform3d.setEuler(new Vector3d(0, 0, 0));
         cameraTransform3d.setTranslation(new Vector3d(.195, 0, .005));
         cameraFrames.put(name, new ReferenceFrame(name, bodyFrames.get(name), cameraTransform3d, false, false, false)
         {
            private static final long serialVersionUID = -9160732749609839626L;

            public void updateTransformToParent(Transform3D transformToParent)
            {
               throw new RuntimeException("do not call the update method on vicon frames");
            }
         });
      }
   }

   /**
    * Note: The name parameter has "_body" concatenated to it within this method. 
    * @param name - the name of the reference frame that will be returned
    * @return
    */
   public static ReferenceFrame getBodyFrame(String name)
   {
      return bodyFrames.get(name+"_body");
   }

   /**
    * Note: The name parameter has "_body_camera" concatenated to it within this method. 
    * @param name - the name of the reference frame that will be returned
    * @return
    */
   public static ReferenceFrame getCameraFrame(String name)
   {
      return bodyFrames.get(name+"_body_camera");
   }
}
