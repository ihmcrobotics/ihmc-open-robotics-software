package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

import java.util.List;

/**
 * Input to the solver {@link MultiContactSupportRegionSolver}
 *
 * {@see http://lall.stanford.edu/papers/bretl_eqmcut_ieee_tro_projection_2008_08_01_01/pubdata/entry.pdf}
 */
public class MultiContactSupportRegionSolverInput
{
   public static final int maxContactPoints = 50;
   private static final double defaultCoefficientOfFriction = 0.9;
   private static final double defaultMaxNormalForce = 2.0;

   /**
    * Coefficient of friction
    */
   private double coefficientOfFriction = defaultCoefficientOfFriction;

   /**
    * The vector r in the paper above
    */
   private final RecyclingArrayList<FramePoint3D> contactPointPositions = new RecyclingArrayList<>(20, FramePoint3D::new);

   /**
    * The vector v in the paper above
    */
   private final RecyclingArrayList<FrameVector3D> surfaceNormals = new RecyclingArrayList<>(20, FrameVector3D::new);

   /**
    * Constraints on rho. Can either be fixed maximum or actuation-based polytope
    */
   private final RecyclingArrayList<ContactPointActuationConstraint> actuationConstraints = new RecyclingArrayList<>(20, ContactPointActuationConstraint::new);

   public void clear()
   {
      contactPointPositions.clear();
      surfaceNormals.clear();
      actuationConstraints.clear();
      coefficientOfFriction = defaultCoefficientOfFriction;
   }

   public void addContactPoint(Point3DReadOnly contactPointPosition, Vector3DReadOnly surfaceNormal)
   {
      addContactPoint(contactPointPosition, surfaceNormal, defaultMaxNormalForce);
   }

   public void addContactPoint(Point3DReadOnly contactPointPosition, Vector3DReadOnly surfaceNormal, double maxNormalForce)
   {
      this.contactPointPositions.add().set(contactPointPosition);
      this.surfaceNormals.add().set(surfaceNormal);

      ContactPointActuationConstraint forceConstraint = this.actuationConstraints.add();
      forceConstraint.setToMaxNormalForce(maxNormalForce);
   }

   public void addContactPoint(Point3DReadOnly contactPointPosition, Vector3DReadOnly surfaceNormal, ConvexPolytope3DReadOnly forcePolytope)
   {
      this.contactPointPositions.add().set(contactPointPosition);
      this.surfaceNormals.add().set(surfaceNormal);

      ContactPointActuationConstraint forceConstraint = this.actuationConstraints.add();
      forceConstraint.setToPolytopeConstraint(forcePolytope);
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

   public RecyclingArrayList<ContactPointActuationConstraint> getActuationConstraints()
   {
      return actuationConstraints;
   }

   public double getCoefficientOfFriction()
   {
      return coefficientOfFriction;
   }
}
