package us.ihmc.vicon;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicReferenceFrame;
import sim.DroneRobot;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class ViconFrames
{
   private static ViconFrames viconFrames = new ViconFrames();
   private ViconClient viconClient;
   private static HashMap<String, ReferenceFrame> referenceFrames;
   private static Transform3D bodyToWorldTransform;
   private static Vector3d euler;
   private static Vector3d translation;

   private ViconFrames()
   {
      try
      {
         euler = new Vector3d();
         translation = new Vector3d();
         bodyToWorldTransform = new Transform3D();
         ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
         referenceFrames = new HashMap<String, ReferenceFrame>();
         referenceFrames.put("World", worldFrame);
         viconClient = ViconClient.getInstance();
         ArrayList<String> modelNames = viconClient.getAvailableModels();
         // wait for vicon to be ready
         while(viconClient.getPose(modelNames.get(0)) == null)
         {
            System.out.println("waiting...");
            Thread.sleep(500);
         }

         for (String modelName : modelNames)
         {
            final String bodyName = modelName;
            System.out.println("adding frame for " + modelName);
            Transform3D transform3d = new Transform3D();
            ReferenceFrame referenceFrame = new ReferenceFrame(replaceColonWithUnderscore(bodyName), worldFrame, transform3d, false, false, false)
            {
               private static final long serialVersionUID = -9160732749609839626L;

               public void updateTransformToParent(Transform3D transformToParent)
               {
                  Pose pose = viconClient.getPose(bodyName);
                  System.out.println("updating frame for " + bodyName + " at " + pose);
//                  euler.set(Math.toRadians(pose.yAxisRotation), Math.toRadians(pose.xAxisRotation), Math.toRadians(pose.zAxisRotation));
                  euler.set(0.0, 0.0, Math.toRadians(pose.zAxisRotation));
                  bodyToWorldTransform.setEuler(euler);
                  translation.set(pose.xPosition/1000.0, pose.yPosition/1000.0, pose.zPosition/1000.0);
                  bodyToWorldTransform.setTranslation(translation);
                  transformToParent.set(bodyToWorldTransform);
               }
            };
            referenceFrame.update();
            referenceFrames.put(bodyName, referenceFrame);
         }
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   private String replaceColonWithUnderscore(String string)
   {
      return string.replace(":", "_");
   }

   public static ReferenceFrame getWorldFrame()
   {
      return referenceFrames.get("World");
   }

   /**
    * Note: The name parameter has "_body" concatenated to it within this method. There is also a camera frame that is created and given the name: name+"_body_camera".
    * @param name - the name of the reference frame that is being updated
    * @param transform3d - the transform from the parent to the named reference frame
    */
   public static void updateTransformToParent(String name, Transform3D transform3d)
   {
      System.err.println("Who is calling this?");
//      name.concat("_body");
//
//      if (referenceFrames.containsKey(name))
//      {
//         // update it
//         referenceFrames.get(name).setTransformToParent(transform3d);
//      }
//      else
//      {
//         // Create
//         referenceFrames.put(name, new ReferenceFrame(name, worldFrame, transform3d, false, false, false)
//         {
//            private static final long serialVersionUID = -9160732749609839626L;
//
//            public void updateTransformToParent(Transform3D transformToParent)
//            {
//               throw new RuntimeException("do not call the update method on vicon frames");
//            }
//         });
//
//         name.concat("_camera");
//         Transform3D cameraTransform3d = new Transform3D();
//         cameraTransform3d.setEuler(new Vector3d(0, 0, 0));
//         cameraTransform3d.setTranslation(new Vector3d(.195, 0, .005));
//         cameraFrames.put(name, new ReferenceFrame(name, referenceFrames.get(name), cameraTransform3d, false, false, false)
//         {
//            private static final long serialVersionUID = -9160732749609839626L;
//
//            public void updateTransformToParent(Transform3D transformToParent)
//            {
//               throw new RuntimeException("do not call the update method on vicon frames");
//            }
//         });
//      }
   }

   /**
    * Note: The name parameter has "_body" concatenated to it within this method.
    * @param name - the name of the reference frame that will be returned
    * @return
    */
   public static ReferenceFrame getBodyFrame(String name)
   {
      return referenceFrames.get(name + "_body");
   }

   /**
    * Note: The name parameter has "_body_camera" concatenated to it within this method.
    * @param name - the name of the reference frame that will be returned
    * @return
    */
   public static ReferenceFrame getCameraFrame(String name)
   {
      return referenceFrames.get(name + "_body_camera");
   }

   public static Collection<ReferenceFrame> getFrames()
   {
      return referenceFrames.values();
   }

   public static void main(String[] args)
   {
      YoVariableRegistry registry = new YoVariableRegistry("ViconFrames");
      DroneRobot robot = new DroneRobot("drone", true);

      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();

      for (ReferenceFrame referenceFrame : ViconFrames.getFrames())
      {
         DynamicGraphicReferenceFrame dynamicGraphicReferenceFrame = new DynamicGraphicReferenceFrame(referenceFrame, registry, 1.0);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject(referenceFrame.getName(), dynamicGraphicReferenceFrame);
         dynamicGraphicReferenceFrame.update();
      }

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.addVarList(registry.createVarList());
      dynamicGraphicObjectsListRegistry.addDynamicGraphicsObjectListsToSimulationConstructionSet(scs);

      Thread scsThread = new Thread(scs);
      scsThread.start();
   }
}
