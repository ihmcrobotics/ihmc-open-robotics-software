package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

import java.util.ArrayList;
import java.util.List;

/**
 * Input to the solver {@link StaticSupportRegionSolver}
 *
 * {@see http://lall.stanford.edu/papers/bretl_eqmcut_ieee_tro_projection_2008_08_01_01/pubdata/entry.pdf}
 */
public class StaticEquilibriumSolverInput
{
   public static final int maxContactPoints = 50;
   private static final double defaultGravityMagnitude = 9.80665;
   private static final double defaultCoefficientOfFriction = 0.9;

   /**
    * The vector r in the paper above
    */
   private final RecyclingArrayList<FramePoint3D> contactPointPositions = new RecyclingArrayList<>(20, FramePoint3D::new);

   /**
    * The vector v in the paper above
    */
   private final RecyclingArrayList<FrameVector3D> surfaceNormals = new RecyclingArrayList<>(20, FrameVector3D::new);

   private double gravityMagnitude = defaultGravityMagnitude;

   private double coefficientOfFriction = defaultCoefficientOfFriction;

   public void clear()
   {
      contactPointPositions.clear();
      surfaceNormals.clear();
      gravityMagnitude = defaultGravityMagnitude;
      coefficientOfFriction = defaultCoefficientOfFriction;
   }

   public void addContactPoint(Point3DReadOnly contactPointPosition, Vector3DReadOnly surfaceNormal)
   {
      this.contactPointPositions.add().set(contactPointPosition);
      this.surfaceNormals.add().set(surfaceNormal);
   }

   public void setGravityMagnitude(double gravityMagnitude)
   {
      this.gravityMagnitude = gravityMagnitude;
   }

   public int getNumberOfContacts()
   {
      return contactPointPositions.size();
   }

   public void setCoefficientOfFriction(double coefficientOfFriction)
   {
      this.coefficientOfFriction = coefficientOfFriction;
   }

   public List<FramePoint3D> getContactPointPositions()
   {
      return contactPointPositions;
   }

   public List<FrameVector3D> getSurfaceNormals()
   {
      return surfaceNormals;
   }

   public double getGravityMagnitude()
   {
      return gravityMagnitude;
   }

   public double getCoefficientOfFriction()
   {
      return coefficientOfFriction;
   }
}
