package us.ihmc.jMonkeyEngineToolkit.jme.lidar;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.jMonkeyEngineToolkit.GPULidar;
import us.ihmc.jMonkeyEngineToolkit.GPULidarListener;
import us.ihmc.jMonkeyEngineToolkit.jme.JMERenderer;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JMEDataTypeUtils;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JMEGeometryUtils;
import us.ihmc.robotics.geometry.RigidBodyTransform;

import com.jme3.app.Application;
import com.jme3.app.state.AppState;
import com.jme3.app.state.AppStateManager;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.RenderManager;
import com.jme3.scene.Node;

public class JMEGPULidar implements GPULidar, AppState
{
   private static final Quaternion scsToJMECameraRotation = new Quaternion();

   static
   {
      scsToJMECameraRotation.fromRotationMatrix(0, 0, 1, -1, 0, 0, 0, -1, 0);
   }

   private final int numberOfCameras;
   private final LidarSceneViewPort[] lidarSceneViewPort;
   private final LidarDistortionProcessor lidarDistortionProcessor;
   private final JMERenderer jmeRenderer;

   private final Quaternion[] cameraRotations;

   private final Vector3d j3dPosition = new Vector3d();
   private final Quat4d j3dOrientation = new Quat4d();

   private final Vector3f jmePosition = new Vector3f();
   private final Quaternion jmeOrientation = new Quaternion();
   private final Quaternion jmeCameraOrientation = new Quaternion();

   private final RigidBodyTransform lidarTransform = new RigidBodyTransform();
   private double time = Double.NaN;
   
   public JMEGPULidar(JMERenderer jmeRenderer, int scansPerSweep, int scanHeight, double fieldOfView, double minRange, double maxRange)
   {
      this.jmeRenderer = jmeRenderer;
      
      // Figure out how many cameras to use. Use a small delta to avoid rounding errors of pi
      if (fieldOfView - 1e-4 <= Math.PI / 2)
      {
         numberOfCameras = 1;
      }
      else if (fieldOfView - 1e-4 <= Math.PI)
      {
         numberOfCameras = 2;
      }
      else if (fieldOfView - 1e-4 <= Math.PI * 3.0 / 2.0)
      {
         numberOfCameras = 3;
      }
      else if (fieldOfView - 1e-4 <= Math.PI * 2.0)
      {
         numberOfCameras = 4;
      }
      else
      {
         throw new RuntimeException("Field of view for LIDAR can not be more than 2 pi radians.");
      }

      if (scansPerSweep != (scansPerSweep / numberOfCameras) * numberOfCameras)
      {
         throw new RuntimeException("scansPerSweep is not integer divisable by number of cameras " + numberOfCameras);
      }

      this.cameraRotations = createCameraRotations(numberOfCameras, (float) fieldOfView);

      float startAngle = -((float) fieldOfView) / 2.0f;

      lidarSceneViewPort = new LidarSceneViewPort[numberOfCameras];
      LidarMaterial lidarMaterial = new LidarMaterial(jmeRenderer.getAssetManager());
      for (int i = 0; i < numberOfCameras; i++)
      {
         lidarSceneViewPort[i] = new LidarSceneViewPort(jmeRenderer, lidarMaterial, numberOfCameras, scansPerSweep, scanHeight,
               (float) fieldOfView, (float) minRange, (float) maxRange);
      }

      lidarDistortionProcessor = new LidarDistortionProcessor(jmeRenderer, scansPerSweep, scanHeight,
            numberOfCameras, startAngle, (float) fieldOfView, lidarSceneViewPort);

      jmeRenderer.getStateManager().attach(this);
   }
   
   public void addGPULidarListener(GPULidarListener listener)
   {
      lidarDistortionProcessor.addGPULidarListener(listener);
   }

   private static Quaternion[] createCameraRotations(int numberOfCameras, float fieldOfView)
   {
      Quaternion[] cameraRotations = new Quaternion[numberOfCameras];
      if (numberOfCameras == 1)
      {
         cameraRotations[0] = null;
      }
      else if (numberOfCameras == 2)
      {
         cameraRotations[0] = new Quaternion();
         cameraRotations[0].fromAngles(0.0f, fieldOfView / 4.0f, 0.0f);

         cameraRotations[1] = new Quaternion();
         cameraRotations[1].fromAngles(0.0f, -fieldOfView / 4.0f, 0.0f);
      }
      else if (numberOfCameras == 3)
      {
         cameraRotations[0] = new Quaternion();
         cameraRotations[0].fromAngles(0.0f, fieldOfView / 3.0f, 0.0f);

         cameraRotations[1] = null;

         cameraRotations[2] = new Quaternion();
         cameraRotations[2].fromAngles(0.0f, -fieldOfView / 3.0f, 0.0f);
      }
      else if (numberOfCameras == 4)
      {
         cameraRotations[0] = new Quaternion();
         cameraRotations[0].fromAngles(0.0f, 3.0f * fieldOfView / 8.0f, 0.0f);

         cameraRotations[1] = new Quaternion();
         cameraRotations[1].fromAngles(0.0f, fieldOfView / 8.0f, 0.0f);

         cameraRotations[2] = new Quaternion();
         cameraRotations[2].fromAngles(0.0f, -fieldOfView / 8.0f, 0.0f);

         cameraRotations[3] = new Quaternion();
         cameraRotations[3].fromAngles(0.0f, -3.0f * fieldOfView / 8.0f, 0.0f);
      }
      return cameraRotations;
   }

   public void setTransformFromWorld(RigidBodyTransform transformFromWorld, double time)
   {
      synchronized (lidarTransform)
      {
         this.lidarTransform.set(transformFromWorld);
         this.time = time;
      }
   }
   
   public void updateViewPortScenes()
   {
      Node sceneToUse;
      if (JMERenderer.USE_GPU_LIDAR_PARALLEL_SCENE)
      {
         sceneToUse = jmeRenderer.cloneSceneWithoutVisualizations();
      }
      else
      {
         sceneToUse = jmeRenderer.getZUpNode();
      }
      
      for (LidarSceneViewPort viewPort : lidarSceneViewPort)
      {
         viewPort.updateScene(sceneToUse);
      }
   }

   @Override
   public void update(float tpf)
   {
      synchronized (lidarTransform)
      {
         lidarTransform.get(j3dOrientation, j3dPosition);
         lidarDistortionProcessor.setTransform(lidarTransform, time);
      }

      JMEDataTypeUtils.packVecMathTuple3dInJMEVector3f(j3dPosition, jmePosition);
      JMEDataTypeUtils.packVectMathQuat4dInJMEQuaternion(j3dOrientation, jmeOrientation);
      JMEGeometryUtils.transformFromZupToJMECoordinates(jmePosition);

      jmeOrientation.multLocal(scsToJMECameraRotation);

      JMEGeometryUtils.transformFromZupToJMECoordinates(jmeOrientation);

      for (int i = 0; i < numberOfCameras; i++)
      {
         lidarSceneViewPort[i].setLocation(jmePosition);
         if (cameraRotations[i] == null)
         {
            lidarSceneViewPort[i].setRotation(jmeOrientation);
         }
         else
         {
            jmeOrientation.mult(cameraRotations[i], jmeCameraOrientation);
            lidarSceneViewPort[i].setRotation(jmeCameraOrientation);
         }
      }

   }

   @Override
   public void initialize(AppStateManager stateManager, Application app)
   {
   }

   @Override
   public boolean isInitialized()
   {
      return true;
   }

   @Override
   public void setEnabled(boolean active)
   {
   }

   @Override
   public boolean isEnabled()
   {
      return true;
   }

   @Override
   public void stateAttached(AppStateManager stateManager)
   {
   }

   @Override
   public void stateDetached(AppStateManager stateManager)
   {
   }

   @Override
   public void render(RenderManager rm)
   {
   }

   @Override
   public void postRender()
   {
   }

   @Override
   public void cleanup()
   {
   }
}
