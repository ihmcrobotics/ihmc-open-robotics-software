package us.ihmc.commonWalkingControlModules.polygonWiggling;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

/**
 * Input to {@link GradientDescentStepConstraintSolver}. The solver can perform the following checks depending on the input:
 * - Transform a polygon such that the constraints in {@link #wiggleParameters} are met
 * - Check for collisions with the leg
 * - Check for collisions with another polygon (e.g. stance foot)
 */
public class GradientDescentStepConstraintInput
{
   /**
    * Step polygon to be constrained. Expressed in the planar region's "local frame", note this is different than the "snap frame"
    */
   private final ConvexPolygon2D initialStepPolygon = new ConvexPolygon2D();

   /**
    * Constraint region, i.e. planar region perimeter. Expressed in planar region's "local frame"
    */
   private Vertex2DSupplier polygonToWiggleInto = null;

   /**
    * Parameters for how far to be inside the region
    */
   private final WiggleParameters wiggleParameters = new WiggleParameters();

   /**
    * Stance foot polygon, provided to avoid overlapping step. This is optional
    */
   private final ConvexPolygon2D stanceFootPolygon = new ConvexPolygon2D();

   /**
    * Foot transform in region frame. Note this is different from the "snap frame", which the step polygon is expressed in. This is optional
    */
   private final RigidBodyTransform footstepInRegionFrame = new RigidBodyTransform();

   /**
    * Planar region's transform to world, this is optional
    */
   private final RigidBodyTransform localToWorld = new RigidBodyTransform();

   /**
    * Regions to check for leg collision, this is optional
    */
   private PlanarRegionsList planarRegionsList = null;


   public GradientDescentStepConstraintInput()
   {
      clear();
   }

   public void setInitialStepPolygon(ConvexPolygon2DReadOnly initialStepPolygon)
   {
      this.initialStepPolygon.set(initialStepPolygon);
   }

   public void setPolygonToWiggleInto(Vertex2DSupplier polygonToWiggleInto)
   {
      this.polygonToWiggleInto = polygonToWiggleInto;
   }

   public void setWiggleParameters(WiggleParameters wiggleParameters)
   {
      this.wiggleParameters.set(wiggleParameters);
   }

   public void setStanceFootPolygon(ConvexPolygon2D stanceFootPolygon)
   {
      this.stanceFootPolygon.set(stanceFootPolygon);
   }

   public void setFootstepInRegionFrame(RigidBodyTransformReadOnly footstepInRegionFrame)
   {
      this.footstepInRegionFrame.set(footstepInRegionFrame);
   }

   public void setLocalToWorld(RigidBodyTransformReadOnly localToWorld)
   {
      this.localToWorld.set(localToWorld);
   }

   public void setPlanarRegion(PlanarRegion planarRegion)
   {
      this.localToWorld.set(planarRegion.getTransformToWorld());
      this.polygonToWiggleInto = Vertex2DSupplier.asVertex2DSupplier(planarRegion.getConcaveHull());
   }

   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
   }

   public ConvexPolygon2D getInitialStepPolygon()
   {
      return initialStepPolygon;
   }

   public Vertex2DSupplier getPolygonToWiggleInto()
   {
      return polygonToWiggleInto;
   }

   public WiggleParameters getWiggleParameters()
   {
      return wiggleParameters;
   }

   public ConvexPolygon2D getStanceFootPolygon()
   {
      return stanceFootPolygon;
   }

   public RigidBodyTransform getFootstepInRegionFrame()
   {
      return footstepInRegionFrame;
   }

   public RigidBodyTransform getLocalToWorld()
   {
      return localToWorld;
   }

   public PlanarRegionsList getPlanarRegionsList()
   {
      return planarRegionsList;
   }

   public void clear()
   {
      initialStepPolygon.clear();
      polygonToWiggleInto = null;
      stanceFootPolygon.clear();
      footstepInRegionFrame.setToNaN();
      localToWorld.setToNaN();
      planarRegionsList = null;
   }

   void checkInputs()
   {
      if (initialStepPolygon.isEmpty())
      {
         throw new RuntimeException("Initial step polygon not set");
      }

      if (polygonToWiggleInto == null)
      {
         throw new RuntimeException("Polygon to wiggle into is not set");
      }
   }

   boolean containsInputForLegCollisionCheck()
   {
      if (footstepInRegionFrame.containsNaN())
      {
         return false;
      }

      if (localToWorld.containsNaN())
      {
         return false;
      }

      if (planarRegionsList == null)
      {
         return false;
      }

      return true;
   }

   boolean containsInputForStanceFootCheck()
   {
      return !stanceFootPolygon.isEmpty();
   }
}
