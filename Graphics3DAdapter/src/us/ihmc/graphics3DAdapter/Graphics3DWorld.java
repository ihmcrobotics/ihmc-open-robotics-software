package us.ihmc.graphics3DAdapter;

import java.awt.Container;
import java.util.concurrent.ConcurrentLinkedQueue;

import javax.swing.JFrame;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.camera.ClassicCameraController;
import us.ihmc.graphics3DAdapter.camera.ViewportAdapter;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.tools.thread.ThreadTools;

public class Graphics3DWorld implements Graphics3DFrameListener
{
   private final Graphics3DAdapter graphics3dAdapter;
   private Graphics3DNode rootNode;
   private final String worldName;
   private ClassicCameraController cameraController;
   protected ViewportAdapter viewportAdapter;
   private JFrame jFrame;
   private ConcurrentLinkedQueue<Graphics3DNode> graphics3DNodesToAddPostFrame = new ConcurrentLinkedQueue<>();

   public Graphics3DWorld(String worldName, Graphics3DAdapter graphics3dAdapter)
   {
      this.graphics3dAdapter = graphics3dAdapter;
      this.worldName = worldName;

      init();
   }

   public Graphics3DWorld(Graphics3DAdapter graphics3dAdapter)
   {
      this(Graphics3DWorld.class.getSimpleName(), graphics3dAdapter);
   }

   private void init()
   {
      rootNode = new Graphics3DNode(worldName + "RootNode");
   }

   protected void start()
   {
      graphics3dAdapter.addRootNode(rootNode);

      viewportAdapter = Graphics3DAdapterTools.createViewport(graphics3dAdapter);
      
      addFrameListener(this);
   }

   public void fixCameraOnNode(Graphics3DNode node)
   {
      checkCameraIsNotNull();

      cameraController.setFixPosition(node.getTranslation().getX(), node.getTranslation().getY(), node.getTranslation().getZ());
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

   private void checkViewportIsNotNull()
   {
      if (viewportAdapter == null)
      {
         try
         {
            throw new Exception("Viewport adapter is null. Call startWithGui or startWithoutGui first.");
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

      cameraController = Graphics3DAdapterTools.createCameraController(graphics3dAdapter, viewportAdapter, new Vector3d(camX, camY, camZ));
      addChild(cameraController.getFixPointNode());
      jFrame = Graphics3DAdapterTools.createNewWindow(viewportAdapter, worldName, windowWidth, windowHeight, cameraController);
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
   
   public void keepAlive(double time)
   {
      ThreadTools.sleepSeconds(time);
   }

   public void stop()
   {
      graphics3dAdapter.closeViewport(viewportAdapter);
      graphics3dAdapter.closeAndDispose();

      if (jFrame != null)
      {
         jFrame.dispose();
      }
   }

   public void addFrameListener(Graphics3DFrameListener frameListener)
   {
      checkViewportIsNotNull();
   
      viewportAdapter.addFrameListener(frameListener);
   }

   public void addChild(Graphics3DNode child)
   {
      if (viewportAdapter == null)
      {
         rootNode.addChild(child);
      }
      else
      {
         graphics3DNodesToAddPostFrame.add(child);
      }
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

   public Graphics3DAdapter getGraphics3DAdapter()
   {
      return graphics3dAdapter;
   }

   @Override
   public void postFrame(double timePerFrame)
   {
      while (!graphics3DNodesToAddPostFrame.isEmpty())
      {
         graphics3dAdapter.addRootNode(graphics3DNodesToAddPostFrame.poll());
      }
   }
   
   public Container getContentPane()
   {
      return jFrame.getContentPane();
   }
}
