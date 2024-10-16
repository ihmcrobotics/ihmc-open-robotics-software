package us.ihmc.robotics.geometry;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameRamp3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameConvexPolytope3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameFace3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameVertex3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.commons.referenceFrames.PoseReferenceFrame;

public class PlanerRegionBuilderTools
{
   public static PlanarRegionsList createRegionsFromBox(FrameBox3DReadOnly box)
   {
      return createRegionsFromPolytope(box, box.asConvexPolytope());
   }

   public static PlanarRegionsList createRegionsFromRamp(FrameRamp3DReadOnly ramp)
   {
      return createRegionsFromPolytope(ramp, ramp.asConvexPolytope());
   }

   public static PlanarRegionsList createRegionsFromPolytope(ReferenceFrameHolder referenceFrameHolder,
                                                             FrameConvexPolytope3DReadOnly frameConvexPolytope3DReadOnly)
   {
      PlanarRegionsList planarRegions = new PlanarRegionsList();
      PoseReferenceFrame faceFrame = new PoseReferenceFrame("faceFrame", referenceFrameHolder.getReferenceFrame());
      for (FrameFace3DReadOnly face : frameConvexPolytope3DReadOnly.getFaces())
      {
         Quaternion faceOrientation = new Quaternion();
         EuclidGeometryTools.orientation3DFromZUpToVector3D(face.getNormal(), faceOrientation);
         faceFrame.setPoseAndUpdate(face.getCentroid(), faceOrientation);

         FrameConvexPolygon2D faceAsPolygon = new FrameConvexPolygon2D(faceFrame);

         for (FrameVertex3DReadOnly vertex : face.getVertices())
         {
            FramePoint3D faceFrameVertex = new FramePoint3D(vertex);
            faceFrameVertex.changeFrame(faceFrame);
            faceAsPolygon.addVertex(faceFrameVertex);
         }

         faceAsPolygon.update();
         planarRegions.addPlanarRegion(new PlanarRegion(faceFrame.getTransformToWorldFrame(), faceAsPolygon));
      }
      return planarRegions;
   }

   public static void setRegionsIds(PlanarRegionsList planarRegions)
   {
      setRegionsIds(0, planarRegions);
   }

   public static void setRegionsIds(int startId, PlanarRegionsList planarRegions)
   {
      for (int i = 0; i < planarRegions.getNumberOfPlanarRegions(); i++)
      {
         planarRegions.getPlanarRegion(i).setRegionId(startId + i);
      }
   }
}
