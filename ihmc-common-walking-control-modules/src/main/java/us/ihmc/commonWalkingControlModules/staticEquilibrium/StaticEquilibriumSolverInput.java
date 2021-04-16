package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.log.LogTools;

import java.util.ArrayList;
import java.util.List;

/**
 * Input to the solver {@link StaticEquilibriumSolver}
 *
 * {@see http://lall.stanford.edu/papers/bretl_eqmcut_ieee_tro_projection_2008_08_01_01/pubdata/entry.pdf}
 */
public class StaticEquilibriumSolverInput
{
   public static final int maxContactPoints = 50;
   private static final double defaultGravityMagnitude = 9.80665;
   private static final double defaultCoefficientOfFriction = 0.8;

   /**
    * The vector r in the paper above
    */
   private final List<FramePoint3D> contactPointPositions = new ArrayList<>();

   /**
    * The vector v in the paper above
    */
   private final List<FrameVector3D> surfaceNormals = new ArrayList<>();

   private double gravityMagnitude = defaultGravityMagnitude;

   private double coefficientOfFriction = defaultCoefficientOfFriction;

   public void clear()
   {
      contactPointPositions.clear();
      surfaceNormals.clear();
      gravityMagnitude = defaultGravityMagnitude;
      coefficientOfFriction = defaultCoefficientOfFriction;
   }

   public void addContactPoint(FramePoint3D contactPointPosition, FrameVector3D surfaceNormal)
   {
      this.contactPointPositions.add(contactPointPosition);
      this.surfaceNormals.add(surfaceNormal);
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
