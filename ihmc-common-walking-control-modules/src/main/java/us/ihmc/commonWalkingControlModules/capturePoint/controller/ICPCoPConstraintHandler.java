package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPolygons;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class ICPCoPConstraintHandler
{
   private final ICPControlPolygons icpControlPolygons;

   private final boolean hasICPControlPolygons;
   private final BooleanProvider useICPControlPolygons;
   private final YoBoolean keepCoPInsideSupportPolygon;

   private final FrameConvexPolygon2D copConstraint = new FrameConvexPolygon2D();

   private int numberOfVertices = 0;
   private boolean hasSupportPolygonChanged;

   public ICPCoPConstraintHandler(ICPControlPolygons icpControlPolygons,
                                  BooleanProvider useICPControlPolygons,
                                  boolean hasICPControlPolygons,
                                  YoRegistry parentRegistry)
   {
      this.icpControlPolygons = icpControlPolygons;
      this.useICPControlPolygons = useICPControlPolygons;
      this.hasICPControlPolygons = hasICPControlPolygons;

      keepCoPInsideSupportPolygon = new YoBoolean("keepCoPInsideSupportPolygon", parentRegistry);
      keepCoPInsideSupportPolygon.set(true);
   }

   /**
    * <p>
    * Updates the CoP and CMP constraint polygons for the optimization controller. This polygon is used
    * to constrain both the location of the CoP and the CMP. It is either the vertical projection of the
    * support feet, or the projection through the CoM (to account for
    * multi and out of plane contact).
    * </p>
    *
    * <p>
    *    NOTE: You MUST call {@link ICPOptimizationQPSolver#resetCoPLocationConstraint()} before calling
    *    this method!
    * </p>
    */
   public FrameConvexPolygon2DReadOnly updateCoPConstraint(FrameConvexPolygon2DReadOnly supportPolygonInWorld)
   {
      if (keepCoPInsideSupportPolygon.getBooleanValue())
      {
         FrameConvexPolygon2DReadOnly supportPolygon;
         if (useICPControlPolygons.getValue() && icpControlPolygons != null && hasICPControlPolygons)
            supportPolygon = icpControlPolygons.getControlPolygonInWorldFrame();
         else
            supportPolygon = supportPolygonInWorld;

         // this is a really simplistic way of checking if the support polygon has changed.
         if (supportPolygon.getNumberOfVertices() != numberOfVertices)
         {
            hasSupportPolygonChanged = true;
            numberOfVertices = supportPolygon.getNumberOfVertices();
         }
         else
         {
            hasSupportPolygonChanged = false;
         }

         copConstraint.setIncludingFrame(supportPolygon);
         return supportPolygon;
      }

      return null;
   }

   public FrameConvexPolygon2DReadOnly getCoPConstraint()
   {
      if (keepCoPInsideSupportPolygon.getBooleanValue())
         return copConstraint;
      else
         return null;
   }

   /**
    * Returns whether or not the support polygon has changed. This currently uses a very simplistic way of
    * determining a change, only monitoring the number of vertices in the support polygon constraint.
    */
   public boolean hasSupportPolygonChanged()
   {
      return hasSupportPolygonChanged;
   }

   public void setKeepCoPInsideSupportPolygon(boolean keepCoPInsideSupportPolygon)
   {
      this.keepCoPInsideSupportPolygon.set(keepCoPInsideSupportPolygon);
   }
}
