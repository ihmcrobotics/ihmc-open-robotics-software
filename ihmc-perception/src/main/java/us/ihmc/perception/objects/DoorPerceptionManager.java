package us.ihmc.perception.objects;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class DoorPerceptionManager
{
   private final long markerID;
   private final ReferenceFrame cameraFrame;
   private final ArUcoMarkerObject doorPanel;
   private final ArUcoMarkerObject doorFrame;
   private final FramePose3D cameraPose = new FramePose3D();
   private final FramePose3D markerPose = new FramePose3D();

   private boolean doorFrameLockedIn = false;

   public DoorPerceptionManager(long markerID, String name, ReferenceFrame cameraFrame)
   {
      this.markerID = markerID;
      this.cameraFrame = cameraFrame;
      doorPanel = new ArUcoMarkerObject(markerID, String.format("%sDoor%dPanel", name, markerID));
      doorFrame = new ArUcoMarkerObject(markerID, String.format("%sDoor%dFrame", name, markerID));
   }

   public void updateMarkerTransform(Tuple3DReadOnly position, QuaternionReadOnly orientation)
   {
      doorPanel.updateMarkerTransform(position, orientation);

      // Hack, once we see the door panel up close, lock in the frame pose, because after a while or the panel moves
      // we won't know where it is anymore
      if (!doorFrameLockedIn)
      {
         doorFrame.updateMarkerTransform(position, orientation);

         cameraPose.setToZero(cameraFrame);
         cameraPose.changeFrame(ReferenceFrame.getWorldFrame());

         markerPose.setIncludingFrame(ReferenceFrame.getWorldFrame(), position, orientation);

         double distanceToMarker = cameraPose.getPosition().distance(markerPose.getPosition());
         if (distanceToMarker < 3.0)
         {
            doorFrameLockedIn = true;
         }
      }
   }

   public ArUcoMarkerObject getDoorPanel()
   {
      return doorPanel;
   }

   public ArUcoMarkerObject getDoorFrame()
   {
      return doorFrame;
   }
}
