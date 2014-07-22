package us.ihmc.graphics3DAdapter;

import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.camera.ClassicCameraController;
import us.ihmc.graphics3DAdapter.camera.ViewportAdapter;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;

public class Graphics3DWorld
{
   private final Graphics3DAdapter graphics3dAdapter;
   private Graphics3DNode rootNode;
   private final String worldName;
   private ClassicCameraController cameraController;
   private ViewportAdapter viewportAdapter;

   public Graphics3DWorld(String worldName, Graphics3DAdapter graphics3dAdapter)
   {
      this.graphics3dAdapter = graphics3dAdapter;
      this.worldName = worldName;

      init();
   }

   public Graphics3DWorld(Graphics3DAdapter graphics3dAdapter)
   {
      this("DefaultWorld", graphics3dAdapter);
   }

   private void init()
   {
      rootNode = new Graphics3DNode(worldName + "RootNode");
   }

   private void start()
   {
      graphics3dAdapter.addRootNode(rootNode);
   }

   public void fixCameraOnNode(Graphics3DNode node)
   {
      checkCameraIsNotNull();

      cameraController.setFixPosition(node.getTranslation().x, node.getTranslation().y, node.getTranslation().z);
   }

   public void setCameraPosition(double x, double y, double z)
   {
      checkCameraIsNotNull();

      cameraController.setCameraPosition(x, y, z);
   }

   private void checkCameraIsNotNull()
   {
      if (cameraController == null)
      {
         try
         {
            throw new Exception("Camera controller is null. Start gui before moving camera.");
         }
         catch (Exception e)
         {
            e.printStackTrace();
         }
      }
   }

   public void startWithGui(double camX, double camY, double camZ, int windowWidth, int windowHeight)
   {
      start();

      viewportAdapter = Graphics3DAdapterTools.createViewport(graphics3dAdapter);
      cameraController = Graphics3DAdapterTools.createNewWindow(graphics3dAdapter, viewportAdapter, worldName, windowWidth, windowHeight,
              new Vector3d(camX, camY, camZ));
   }

   public void addFrameListener(Graphics3DFrameListener frameListener)
   {
      viewportAdapter.addFrameListener(frameListener);
   }

   public void startWithGui(int windowWidth, int windowHeight)
   {
      startWithGui(ClassicCameraController.CAMERA_START_X, ClassicCameraController.CAMERA_START_Y, ClassicCameraController.CAMERA_START_Z, windowWidth,
                   windowHeight);
   }

   public void startWithGui(double camX, double camY, double camZ)
   {
      startWithGui(camX, camY, camZ, 800, 600);
   }

   public void startWithGui()
   {
      startWithGui(ClassicCameraController.CAMERA_START_X, ClassicCameraController.CAMERA_START_Y, ClassicCameraController.CAMERA_START_Z, 800, 600);
   }

   public void startWithoutGui()
   {
      start();
   }

   public void addChild(Graphics3DNode child)
   {
      rootNode.addChild(child);
   }

   public void addAllChildren(Graphics3DNode... children)
   {
      for (Graphics3DNode child : children)
      {
         addChild(child);
      }
   }

   public Graphics3DNode getRootNode()
   {
      return rootNode;
   }

   public void setRootNode(Graphics3DNode rootNode)
   {
      this.rootNode = rootNode;
   }

   public Graphics3DAdapter getGraphics3dAdapter()
   {
      return graphics3dAdapter;
   }
}
