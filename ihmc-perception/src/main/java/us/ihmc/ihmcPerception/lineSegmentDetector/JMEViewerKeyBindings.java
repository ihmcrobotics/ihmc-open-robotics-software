package us.ihmc.ihmcPerception.lineSegmentDetector;

import com.jme3.input.KeyInput;
import com.jme3.input.controls.*;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.input.InputManager;
import com.jme3.input.MouseInput;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import us.ihmc.euclid.tuple3D.Point3D;

import java.awt.*;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Paths;

public class JMEViewerKeyBindings
{

   private boolean camNavEnabled;
   private boolean camNavShiftEnabled;
   private boolean camNavLiftEnabled;
   private Node camParentNode;
   private Node camGrandNode;
   private Node camNode;
   private Geometry camBoxGeom;
   private JMEPointCloudViewer viewer;

   public void setNodes(Node camParentNode, Node camGrandNode, Node camNode, Geometry camBoxGeom)
   {
      this.camBoxGeom = camBoxGeom;
      this.camGrandNode = camGrandNode;
      this.camParentNode = camParentNode;
      this.camNode = camNode;
   }

   public void setViewer(JMEPointCloudViewer viewer)
   {
      this.viewer = viewer;
   }

   public void setKeyBindings(InputManager inputManager)
   {
      inputManager.addMapping("SavePCD", new KeyTrigger(KeyInput.KEY_S));

      inputManager.addMapping("CamNav", new MouseButtonTrigger(MouseInput.BUTTON_LEFT));
      inputManager.addMapping("CamNavShift", new MouseButtonTrigger(MouseInput.BUTTON_RIGHT));
      inputManager.addMapping("CamNavLift", new MouseButtonTrigger(MouseInput.BUTTON_MIDDLE));

      inputManager.addMapping("X+", new MouseAxisTrigger(MouseInput.AXIS_X, true));
      inputManager.addMapping("X-", new MouseAxisTrigger(MouseInput.AXIS_X, false));
      inputManager.addMapping("Y+", new MouseAxisTrigger(MouseInput.AXIS_Y, true));
      inputManager.addMapping("Y-", new MouseAxisTrigger(MouseInput.AXIS_Y, false));
      inputManager.addMapping("Zoom+", new MouseAxisTrigger(MouseInput.AXIS_WHEEL, true));
      inputManager.addMapping("Zoom-", new MouseAxisTrigger(MouseInput.AXIS_WHEEL, false));

      inputManager.addListener((AnalogListener) (name, value, tpf) ->
      {
         if (camNavEnabled)
         {
            switch (name)
            {
               case "X+":
               {
                  Quaternion q = new Quaternion().fromAngleAxis(0.008f, Vector3f.UNIT_Y);
                  Quaternion qNew = camParentNode.getLocalRotation().mult(q);
                  camGrandNode.rotate(0, 0.008f, 0);
                  break;
               }
               case "X-":
               {
                  Quaternion q = new Quaternion().fromAngleAxis(-0.008f, Vector3f.UNIT_Y);
                  Quaternion qNew = camParentNode.getLocalRotation().mult(q);
                  camGrandNode.rotate(0, -0.008f, 0);
                  break;
               }
               case "Y+":
               {
                  Quaternion q = new Quaternion().fromAngleAxis(0.008f, Vector3f.UNIT_X);
                  Quaternion qNew = camParentNode.getLocalRotation().mult(q);
                  camParentNode.setLocalRotation(qNew);
                  break;
               }
               case "Y-":
               {
                  Quaternion q = new Quaternion().fromAngleAxis(-0.008f, Vector3f.UNIT_X);
                  Quaternion qNew = camParentNode.getLocalRotation().mult(q);
                  camParentNode.setLocalRotation(qNew);
                  break;
               }
            }
         }
         if (camNavShiftEnabled)
         {
            switch (name)
            {
               case "X+":
               {
                  Vector3f dir = camGrandNode.getLocalRotation().mult(Vector3f.UNIT_X);
                  camGrandNode.move(dir.scaleAdd(-0.01f, Vector3f.ZERO));
                  camBoxGeom.setLocalTranslation(camGrandNode.getLocalTranslation());
                  break;
               }
               case "X-":
               {
                  Vector3f dir = camGrandNode.getLocalRotation().mult(Vector3f.UNIT_X);
                  camGrandNode.move(dir.scaleAdd(0.01f, Vector3f.ZERO));
                  camBoxGeom.setLocalTranslation(camGrandNode.getLocalTranslation());
                  break;
               }
               case "Y+":
               {
                  Vector3f dir = camGrandNode.getLocalRotation().mult(Vector3f.UNIT_Z);
                  camGrandNode.move(dir.scaleAdd(0.01f, Vector3f.ZERO));
                  camBoxGeom.setLocalTranslation(camGrandNode.getLocalTranslation());
                  break;
               }
               case "Y-":
               {
                  Vector3f dir = camGrandNode.getLocalRotation().mult(Vector3f.UNIT_Z);
                  camGrandNode.move(dir.scaleAdd(-0.01f, Vector3f.ZERO));
                  camBoxGeom.setLocalTranslation(camGrandNode.getLocalTranslation());
                  break;
               }
            }
         }
         if (camNavLiftEnabled)
         {
            switch (name)
            {
               case "Y+":
               {
                  // System.out.println("Up");
                  Vector3f dir = camGrandNode.getLocalRotation().mult(Vector3f.UNIT_Y);
                  camGrandNode.move(dir.scaleAdd(0.02f, Vector3f.ZERO));
                  camBoxGeom.setLocalTranslation(camGrandNode.getLocalTranslation());
                  break;
               }
               case "Y-":
               {
                  // System.out.println("Down");
                  Vector3f dir = camGrandNode.getLocalRotation().mult(Vector3f.UNIT_Y);
                  camGrandNode.move(dir.scaleAdd(-0.02f, Vector3f.ZERO));
                  camBoxGeom.setLocalTranslation(camGrandNode.getLocalTranslation());
                  break;
               }
            }
         }

         if (name.equals("Zoom+"))
         {
            Vector3f vec = camNode.getLocalTranslation();
            camNode.setLocalTranslation(0, 0, vec.z - 0.4f);
         }
         else if (name.equals("Zoom-"))
         {
            Vector3f vec = camNode.getLocalTranslation();
            camNode.setLocalTranslation(0, 0, vec.z + 0.4f);
         }
      }, "X+", "X-", "Y+", "Y-", "Zoom+", "Zoom-");
      inputManager.addListener((ActionListener) (name, isPressed, tpf) ->
      {
         switch (name)
         {
            case "CamNav":
            {
               // System.out.println("Button Left Clicked");
               camNavEnabled = isPressed;
               break;
            }
            case "CamNavShift":
            {
               // System.out.println("Button Right Clicked");
               camNavShiftEnabled = isPressed;
               break;
            }
            case "CamNavLift":
            {
               // System.out.println("Button Middle Clicked");
               camNavLiftEnabled = isPressed;
               break;
            }
            case "SavePCD":
            {
               if (isPressed)
               {
                  System.out.println("Saving Current Point Cloud as PCD: " + viewer.points3D.length);
                  String path1 = Paths.get(
                        "./ihmc-open-robotics-software/ihmc-perception/src/main/java/us/ihmc/ihmcPerception/lineSegmentDetector/pointcloud1.pcd")
                                      .toAbsolutePath()
                                      .normalize()
                                      .toString();
                  String path2 = Paths.get(
                        "./ihmc-open-robotics-software/ihmc-perception/src/main/java/us/ihmc/ihmcPerception/lineSegmentDetector/pointcloud2.pcd")
                                      .toAbsolutePath()
                                      .normalize()
                                      .toString();

                  File pcdFile;
                  if (!viewer.firstSaved)
                  {
                     pcdFile = new File(path1);
                     viewer.firstSaved = true;
                  }
                  else
                  {
                     pcdFile = new File(path2);
                  }
                  try
                  {
                     pcdFile.createNewFile();
                     FileWriter writer = new FileWriter(pcdFile);
                     writer.write("DATA\n");
                     for (int i = 0; i < viewer.points3D.length; i++)
                     {
                        Point3D point = viewer.points3D[i];
                        Color color = viewer.pointColors[i];
                        writer.write(point.getX32() + "\t" + point.getY32() + "\t" + point.getZ32() + "\t");
                        writer.write(color.getRed() + "\t" + color.getGreen() + "\t" + color.getBlue() + "\n");
                     }
                     writer.close();
                  }
                  catch (IOException e)
                  {
                     e.printStackTrace();
                  }
               }
            }
         }
      }, "CamNav", "CamNavShift", "CamNavLift", "SavePCD");
   }
}