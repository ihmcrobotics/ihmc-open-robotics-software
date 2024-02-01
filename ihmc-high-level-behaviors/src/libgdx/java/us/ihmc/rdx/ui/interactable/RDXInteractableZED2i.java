package us.ihmc.rdx.ui.interactable;

import com.badlogic.gdx.graphics.g3d.model.data.ModelData;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.affordances.RDXInteractableFrameModel;
import us.ihmc.robotics.interaction.CylinderRayIntersection;

public class RDXInteractableZED2i
{
   private static final double LENGTH = 0.172;
   private static final double RADIUS = 0.042 / 2.0;

   private final RDXInteractableFrameModel interactableFrameModel = new RDXInteractableFrameModel();
   private final CylinderRayIntersection cylinderRayIntersection = new CylinderRayIntersection();
   private final Point3D offset = new Point3D(-RADIUS, 0.0, 0.0);

   public RDXInteractableZED2i(RDX3DPanel panel3D, ReferenceFrame referenceFrameToRepresent, RigidBodyTransform transformToParentToModify)
   {
      ModelData zed2iSensorModel = RDXModelLoader.loadModelData("environmentObjects/ZED2i/ZED2i.g3dj");
      interactableFrameModel.create(referenceFrameToRepresent, transformToParentToModify, panel3D, zed2iSensorModel, this::calculateClosestCollision);
   }

   private double calculateClosestCollision(Line3DReadOnly mousePickRay)
   {
      cylinderRayIntersection.update(LENGTH, RADIUS, offset, Axis3D.Y, interactableFrameModel.getReferenceFrame());
      return cylinderRayIntersection.intersect(mousePickRay);
   }

   public RDXInteractableFrameModel getInteractableFrameModel()
   {
      return interactableFrameModel;
   }
}
