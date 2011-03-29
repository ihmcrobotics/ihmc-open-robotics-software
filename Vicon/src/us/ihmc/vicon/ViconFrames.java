package us.ihmc.vicon;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicReferenceFrame;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class ViconFrames
{
   private static ViconFrames viconFrames = new ViconFrames();
   private ViconClient viconClient;
   private static HashMap<String, ReferenceFrame> referenceFrames;
   private static HashMap<String, DynamicGraphicReferenceFrame> dynamicReferenceFrames;
   private static Transform3D bodyToWorldTransform;
   private static Vector3d euler;
   private static Vector3d translation;
   private static DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry;
   private YoVariableRegistry registry;

   private ViconFrames()
   {
      euler = new Vector3d();
      translation = new Vector3d();
      bodyToWorldTransform = new Transform3D();
      dynamicGraphicObjectsListRegistry  = new DynamicGraphicObjectsListRegistry();
      registry = new YoVariableRegistry("ViconFrames");

      try
      {
         ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
         referenceFrames = new HashMap<String, ReferenceFrame>();
         dynamicReferenceFrames = new HashMap<String, DynamicGraphicReferenceFrame>();
         DynamicGraphicReferenceFrame dynamicGraphicReferenceFrame = new DynamicGraphicReferenceFrame(worldFrame, registry, 1.0);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("World", dynamicGraphicReferenceFrame);
         dynamicReferenceFrames.put("World", dynamicGraphicReferenceFrame);
         referenceFrames.put("World", worldFrame);
         viconClient = ViconClient.getInstance();
         ArrayList<String> modelNames = viconClient.getAvailableModels();

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
                  if(pose == null)
                  {
                     pose = new Pose(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                  }
                  euler.set(pose.xAxisRotation, pose.yAxisRotation, pose.zAxisRotation);
                  bodyToWorldTransform.setEuler(euler);
                  translation.set(pose.xPosition, pose.yPosition, pose.zPosition);
                  bodyToWorldTransform.setTranslation(translation);
                  transformToParent.set(bodyToWorldTransform);
                  dynamicReferenceFrames.get(bodyName).update();
               }
            };
            dynamicGraphicReferenceFrame = new DynamicGraphicReferenceFrame(referenceFrame, registry, 1.0);
            dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject(referenceFrame.getName(), dynamicGraphicReferenceFrame);
            referenceFrames.put(bodyName, referenceFrame);
            dynamicReferenceFrames.put(bodyName, dynamicGraphicReferenceFrame);
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
      ReferenceFrame referenceFrame = referenceFrames.get(name);
      referenceFrame.update();
   }

   /**
    * Note: The name parameter has "_body" concatenated to it within this method.
    * @param name - the name of the reference frame that will be returned
    * @return
    */
   public static ReferenceFrame getBodyFrame(String name)
   {
      return referenceFrames.get(name);
   }

   public static Collection<ReferenceFrame> getFrames()
   {
      return referenceFrames.values();
   }

   public static DynamicGraphicObjectsListRegistry getDynamicGraphicObjectsListRegistry()
   {
      return dynamicGraphicObjectsListRegistry;
   }

   public static void main(String[] args)
   {
      Collection<ReferenceFrame> frames = ViconFrames.getFrames();
      System.out.println("size " + frames.size());

      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("dummy"));
      ViconFrames.getDynamicGraphicObjectsListRegistry().addDynamicGraphicsObjectListsToSimulationConstructionSet(scs);

      Thread scsThread = new Thread(scs);
      scsThread.start();

      try
      {
         Thread.sleep(5000);
         ViconClient viconClient = ViconClient.getInstance();
         ArrayList<String> modelNames = viconClient.getAvailableModels();
         ReferenceFrame drone = ViconFrames.getBodyFrame(modelNames.get(0));

         FramePoint point = new FramePoint(drone);
         System.out.println(point.changeFrameCopy(ViconFrames.getWorldFrame()));

         point = new FramePoint(drone, new Point3d(1.0, 0.0, 0.0));
         System.out.println(point.changeFrameCopy(ViconFrames.getWorldFrame()));
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }

   }
}
