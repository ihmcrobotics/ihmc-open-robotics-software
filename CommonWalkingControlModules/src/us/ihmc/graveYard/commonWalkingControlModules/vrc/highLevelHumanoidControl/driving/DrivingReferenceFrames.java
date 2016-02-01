package us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.driving;

import java.util.EnumMap;

import us.ihmc.tools.FormattingTools;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;

/**
 * @author twan
 *         Date: 6/4/13
 */
public class DrivingReferenceFrames
{
   private final PoseReferenceFrame vehicleFrame;
   private final EnumMap<VehicleObject, ReferenceFrame> vehicleObjectFrames = new EnumMap<VehicleObject, ReferenceFrame>(VehicleObject.class);

   public DrivingReferenceFrames(VehicleModelObjects vehicleModelObjects)
   {
      vehicleFrame = new PoseReferenceFrame("vehicle", ReferenceFrame.getWorldFrame());
      for (VehicleObject vehicleObject : VehicleObject.values())
      {
         String name = FormattingTools.underscoredToCamelCase(vehicleObject.toString(), false);
         RigidBodyTransform transform = vehicleModelObjects.getTransform(vehicleObject);
         ReferenceFrame carObjectFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(name, vehicleFrame, transform);
         vehicleObjectFrames.put(vehicleObject, carObjectFrame);
      }
   }

   public void setVehicleFramePoseInWorld(FramePose pose)
   {
      vehicleFrame.setPoseAndUpdate(pose);
   }

   public ReferenceFrame getVehicleFrame()
   {
      return vehicleFrame;
   }

   public ReferenceFrame getObjectFrame(VehicleObject object)
   {
      return vehicleObjectFrames.get(object);
   }
}
