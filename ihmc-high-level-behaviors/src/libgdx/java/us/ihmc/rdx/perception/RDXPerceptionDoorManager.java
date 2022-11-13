package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.perception.OpenCVArUcoMarkerDetection;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.object.objects.door.RDXArUcoVirtualDoorFrame;
import us.ihmc.rdx.simulation.environment.object.objects.door.RDXArUcoVirtualDoorPanel;

import java.util.Set;

public class RDXPerceptionDoorManager
{
   private RDXArUcoVirtualDoorPanel virtualDoorPanel;
   private RDXArUcoVirtualDoorFrame virtualDoorFrame;
   private boolean doorDetected = false;
   private boolean doorDetectedOnce = false;
   private boolean doorFrameLockedIn = false;

   public void create(int arUcoMarkerID, String name)
   {
      virtualDoorPanel = new RDXArUcoVirtualDoorPanel(arUcoMarkerID, name);
      virtualDoorFrame = new RDXArUcoVirtualDoorFrame(arUcoMarkerID, name);
   }

   public void update(OpenCVArUcoMarkerDetection arUcoMarkerDetection,
                      ReferenceFrame sensorFrame,
                      FramePose3D cameraPose)
   {
      doorDetected = arUcoMarkerDetection.isDetected(virtualDoorPanel.getArUcoMarker());
      if (doorDetected)
      {
         doorDetectedOnce = true;

         FramePose3DBasics panelMarkerPose = arUcoMarkerDetection.getPose(virtualDoorPanel.getArUcoMarker());
         panelMarkerPose.changeFrame(ReferenceFrame.getWorldFrame());
         panelMarkerPose.get(virtualDoorPanel.getMarkerToWorld());

         // Hack, once we see the door panel up close, lock in the frame pose, because after a while or the panel moves
         // we won't know where it is anymore
         if (!doorFrameLockedIn)
         {
            FramePose3DBasics markerPose = arUcoMarkerDetection.getPose(virtualDoorFrame.getArUcoMarker());
            markerPose.changeFrame(ReferenceFrame.getWorldFrame());
            markerPose.get(virtualDoorFrame.getMarkerToWorld());

            cameraPose.setToZero(sensorFrame);
            cameraPose.changeFrame(ReferenceFrame.getWorldFrame());
            double distanceToMarker = markerPose.getPosition().distance(cameraPose.getPosition());
            if (distanceToMarker < 0.9)
            {
               doorFrameLockedIn = true;
            }
         }
      }
      virtualDoorPanel.update();
      virtualDoorFrame.update();
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (doorDetected)
      {
         virtualDoorPanel.getRenderables(renderables, pool, sceneLevels);
      }
      if (doorDetectedOnce)
      {
         virtualDoorFrame.getRenderables(renderables, pool, sceneLevels);
      }
   }

   public RDXArUcoVirtualDoorPanel getVirtualDoorPanel()
   {
      return virtualDoorPanel;
   }

   public RDXArUcoVirtualDoorFrame getVirtualDoorFrame()
   {
      return virtualDoorFrame;
   }
}
