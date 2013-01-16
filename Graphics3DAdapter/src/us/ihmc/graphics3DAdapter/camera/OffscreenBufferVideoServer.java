package us.ihmc.graphics3DAdapter.camera;

import us.ihmc.graphics3DAdapter.Graphics3DAdapter;

public class OffscreenBufferVideoServer
{
   
   private final Graphics3DStreamingVideoTCPServer graphics3dStreamingVideoTCPServer;
   
   public OffscreenBufferVideoServer(Graphics3DAdapter adapter, CameraMountList mountList, CameraConfiguration cameraConfiguration, int width, int height, int port)
   {
      this(adapter, mountList, cameraConfiguration, new SimpleCameraTrackingAndDollyPositionHolder(), width, height, port);
   }
   
   public OffscreenBufferVideoServer(Graphics3DAdapter adapter, CameraMountList mountList, CameraConfiguration cameraConfiguration, CameraTrackingAndDollyPositionHolder cameraTrackingAndDollyPositionHolder, int width, int height, int port)
   {  
      ViewportAdapter viewport = adapter.createNewViewport(null, false, true);
      viewport.setupOffscreenView(width, height);
      
      ClassicCameraController cameraController = new ClassicCameraController(adapter, viewport, cameraTrackingAndDollyPositionHolder);
      cameraController.setConfiguration(cameraConfiguration, mountList);
      viewport.setCameraController(cameraController);
      
      graphics3dStreamingVideoTCPServer = new Graphics3DStreamingVideoTCPServer(viewport.getCaptureDevice(), port);
      
      Runtime.getRuntime().addShutdownHook(new Thread(new ShutdownServerListener()));
   }
   
   public void close()
   {
      graphics3dStreamingVideoTCPServer.close();
   }
   
   private class ShutdownServerListener implements Runnable
   {

      public void run()
      {
         graphics3dStreamingVideoTCPServer.close();
      }
      
   }
}
