package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import controller_msgs.msg.dds.MultiContactBalanceStatus;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

import java.io.*;
import java.util.Arrays;
import java.util.List;

/**
 * Input to the solver {@link MultiContactFrictionBasedSupportRegionSolver}
 *
 * {@see http://lall.stanford.edu/papers/bretl_eqmcut_ieee_tro_projection_2008_08_01_01/pubdata/entry.pdf}
 */
public class MultiContactFrictionBasedSupportRegionSolverInput
{
   public static final int maxContactPoints = 50;
   private static final double defaultCoefficientOfFriction = 0.7;
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

   public void setFromMessage(MultiContactBalanceStatus multiContactBalanceStatus)
   {
      clear();

      for (int i = 0; i < multiContactBalanceStatus.getContactPointsInWorld().size(); i++)
      {
         addContactPoint(multiContactBalanceStatus.getContactPointsInWorld().get(i), multiContactBalanceStatus.getSurfaceNormalsInWorld().get(i));
      }
   }

   public void writeToFile(Writer writer) throws IOException
   {
      writer.write("Coefficient of friction:" + coefficientOfFriction + "\n");
      writer.write("Number of points:" + contactPointPositions.size() + "\n");
      for (int i = 0; i < contactPointPositions.size(); i++)
      {
         FramePoint3D position = contactPointPositions.get(i);
         FrameVector3D normal = surfaceNormals.get(i);

         writer.write(position.getX() + "," + position.getY() + "," + position.getZ() + "," + normal.getX() + "," + normal.getY() + "," + normal.getZ() + ",");

         ContactPointActuationConstraint actuationConstraint = actuationConstraints.get(i);
         if (actuationConstraint.isMaxNormalForceConstraint())
         {
            writer.write(Double.toString(actuationConstraint.getMaxNormalForce()));
         }
         else
         {
            ConvexPolytope3D polytopeConstraint = actuationConstraint.getPolytopeConstraint();
            for (int j = 0; j < polytopeConstraint.getNumberOfVertices(); j++)
            {
               writer.write(polytopeConstraint.getVertex(j).getX() + "," + polytopeConstraint.getVertex(j).getY() + "," + polytopeConstraint.getVertex(j).getZ());
               if (j < polytopeConstraint.getNumberOfVertices() - 1)
                  writer.write(",");
            }
         }

         writer.write("\n");
      }
   }

   public static MultiContactFrictionBasedSupportRegionSolverInput loadFromFile(BufferedReader reader) throws IOException
   {
      MultiContactFrictionBasedSupportRegionSolverInput input = new MultiContactFrictionBasedSupportRegionSolverInput();
      input.setCoefficientOfFriction(Double.parseDouble(reader.readLine().split(":")[1]));
      int numberOfPoints = Integer.parseInt(reader.readLine().split(":")[1]);

      for (int i = 0; i < numberOfPoints; i++)
      {
         String[] dataStrings = reader.readLine().split(",");
         double[] data = Arrays.stream(dataStrings).mapToDouble(Double::parseDouble).toArray();

         Point3D point = new Point3D(data[0], data[1], data[2]);
         Vector3D normal = new Vector3D(data[3], data[4], data[5]);

         if (data.length == 7)
         {
            double maxForce = data[6];
            input.addContactPoint(point, normal, maxForce);
         }
         else
         {
            int numberOfVertices = (data.length - 6) / 3;
            ConvexPolytope3D polytope = new ConvexPolytope3D();
            for (int j = 0; j < numberOfVertices; j++)
            {
               polytope.addVertex(new Point3D(data[6 + 3 * j], data[6 + 3 * j + 1], data[6 + 3 * j + 2]));
            }
            input.addContactPoint(point, normal, polytope);
         }
      }

      return input;
   }
}
