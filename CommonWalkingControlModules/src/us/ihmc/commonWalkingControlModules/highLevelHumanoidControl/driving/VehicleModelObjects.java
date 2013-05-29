package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.driving;

import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import javax.media.j3d.Transform3D;

/**
 * @author twan
 *         Date: 5/29/13
 */
public interface VehicleModelObjects
{
   Transform3D getTransform(VehicleObject vehicleObject);

   FramePose getFramePose(ReferenceFrame vehicleReferenceFrame, VehicleObject vehicleObject);
}
