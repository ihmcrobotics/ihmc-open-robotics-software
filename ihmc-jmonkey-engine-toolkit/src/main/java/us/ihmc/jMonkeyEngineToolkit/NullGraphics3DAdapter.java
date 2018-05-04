package us.ihmc.jMonkeyEngineToolkit;

import java.awt.Canvas;
import java.awt.Color;
import java.awt.GraphicsDevice;
import java.awt.image.BufferedImage;
import java.io.File;
import java.net.URL;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DSpotLight;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.color.MutableColor;
import us.ihmc.graphicsDescription.input.SelectedListener;
import us.ihmc.graphicsDescription.input.keyboard.KeyListener;
import us.ihmc.graphicsDescription.input.mouse.Mouse3DListener;
import us.ihmc.graphicsDescription.input.mouse.MouseListener;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraController;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraStreamer;
import us.ihmc.jMonkeyEngineToolkit.camera.CaptureDevice;
import us.ihmc.jMonkeyEngineToolkit.camera.RGBDStreamer;
import us.ihmc.jMonkeyEngineToolkit.camera.ViewportAdapter;
import us.ihmc.robotics.lidar.LidarScanParameters;

public class NullGraphics3DAdapter implements Graphics3DAdapter
{
   private final Object graphicsConch = new Object();

   public void setupSky()
   {
      
   }
   
   public void addRootNode(Graphics3DNode rootNode)
   {
   }

   public void removeRootNode(Graphics3DNode rootNode)
   {
   }

   public ViewportAdapter createNewViewport(GraphicsDevice graphicsDevice, boolean isMainViewport, boolean isOffScreen)
   {
      return new ViewportAdapter()
      {
         public void setupOffscreenView(int width, int height)
         {
         }

         public void setCameraController(CameraController cameraController)
         {
         }

         public double[][] getZBuffer()
         {
            return new double[1][1];
         }

         public Point3D getWorldCoordinatesFromScreenCoordinates(float f, float g, double z)
         {
            return new Point3D();
         }

         public double getPhysicalWidth()
         {
            return 1;
         }

         public double getPhysicalHeight()
         {
            return 1;
         }

         public double getFieldOfView()
         {
            return Math.PI;
         }

         public CaptureDevice getCaptureDevice()
         {
            return new CaptureDevice()
            {
               public void streamTo(CameraStreamer cameraStreamer, int framesPerSecond)
               {
               }

               public void setSize(int width, int height)
               {
               }

               public int getWidth()
               {
                  return 1;
               }

               public int getHeight()
               {
                  return 1;
               }

               public BufferedImage exportSnapshotAsBufferedImage()
               {
                  return new BufferedImage(1, 1, BufferedImage.TYPE_3BYTE_BGR);
               }

               public void exportSnapshot(File snapshotFile)
               {
               }

               @Override
               public void streamTo(RGBDStreamer cameraStreamer, int framesPerSecond)
               {
                  
               }
            };
         }

         public Canvas getCanvas()
         {
            return new Canvas();
         }

         public CameraController getCameraController()
         {
            return new CameraController()
            {
               public double getHorizontalFieldOfViewInRadians()
               {
                  return Math.PI;
               }

               public double getClipNear()
               {
                  return 0.1;
               }

               public double getClipFar()
               {
                  return 100.0;
               }

               public void computeTransform(RigidBodyTransform cameraTransform)
               {
               }

               public void closeAndDispose()
               {                  
               }
            };
         }

         public CameraAdapter getCamera()
         {
            return new CameraAdapter()
            {
               public float getHorizontalFovInRadians()
               {
                  return (float) Math.PI;
               }

               public Quaternion getCameraRotation()
               {
                  return new Quaternion();
               }

               public Point3D getCameraPosition()
               {
                  return new Point3D();
               }
            };
         }

         public void addContextSwitchedListener(ContextSwitchedListener contextSwitchedListener)
         {
         }
      };
   }

   public void closeViewport(ViewportAdapter viewport)
   {
   }

   public void setHeightMap(HeightMap heightMap)
   {
   }

   public Object getGraphicsConch()
   {
      return graphicsConch;
   }

   public void setGroundVisible(boolean isVisible)
   {
   }

   public void addSelectedListener(SelectedListener selectedListener)
   {
   }

   public void addKeyListener(KeyListener keyListener)
   {
   }

   public void addMouseListener(MouseListener mouseListener)
   {
   }

   @Override
   public void addMouse3DListener(Mouse3DListener mouse3dListener)
   {
   }

   public void closeAndDispose()
   {
   }

   public void setBackgroundColor(MutableColor color)
   {
   }

   public void setBackgroundImage(URL fileURL, Graphics3DBackgroundScaleMode backgroundScaleMode)
   {
   }

   public void setGroundAppearance(AppearanceDefinition app)
   {
   }

   public void freezeFrame(Graphics3DNode rootJoint)
   {
   }

   public ContextManager getContextManager()
   {
      return new ContextManager()
      {
         public ViewportAdapter getCurrentViewport()
         {
            return null;
         }
      };
   }

   @Override
   public GPULidar createGPULidar(int pointsPerSweep, int scanHeight, double fieldOfView, double minRange, double maxRange)
   {
      return null;
   }

   @Override
   public GPULidar createGPULidar(GPULidarListener listener, int pointsPerSweep, int scanHeight, double fieldOfView, double minRange, double maxRange)
   {
      return null;
   }
   
   @Override
   public GPULidar createGPULidar(GPULidarListener listener, LidarScanParameters lidarScanParameters)
   {
      return null;
   }

   @Override
   public GPULidar createGPULidar(LidarScanParameters lidarScanParameters)
   {
      return null;
   }
	
	@Override
	public void play() {
		
		
	}
	
	@Override
	public void pause() {
		
	}

   @Override
   public void setupSky(String skyBox)
   {
      
   }
   
   @Override
   public void setupSky(String west, String east, String north, String south, String up, String down)
   {
      
   }

   @Override
   public void addDirectionalLight(Color color, Vector3D direction)
   {
      
   }

   @Override
   public void clearDirectionalLights()
   {
      
   }

   @Override
   public void setAmbientLight(Color color)
   {
      
   }

   @Override
   public void addSpotLight(Graphics3DSpotLight spotLight)
   {
      
   }

   @Override
   public void removeSpotLight(Graphics3DSpotLight spotLight)
   {
      
   }
}
