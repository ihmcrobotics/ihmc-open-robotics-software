package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/* package-private */
class ContactPoint
{
   public static final double basisVectorGraphicScale = 0.1;
   public static final double forceVectorGraphicScale = 0.2;
   public static final int basisVectorsPerContactPoint = 4;

   private final int contactPointIndex;

   private final YoFrameVector3D[] polyhedraFrameBasisVectors = new YoFrameVector3D[basisVectorsPerContactPoint];
   private final YoFramePose3D surfacePose;
   private final YoFrameVector3D force;
   private final PoseReferenceFrame contactPointFrame;
   private final YoDouble[] rhoValues = new YoDouble[basisVectorsPerContactPoint];

   public ContactPoint(int contactPointIndex, YoRegistry registry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.contactPointIndex = contactPointIndex;
      this.contactPointFrame = new PoseReferenceFrame("cpFrame" + contactPointIndex, ReferenceFrame.getWorldFrame());
      this.surfacePose = new YoFramePose3D("cpPose" + contactPointIndex, ReferenceFrame.getWorldFrame(), registry);
      this.force = new YoFrameVector3D("cpForce" + contactPointIndex, ReferenceFrame.getWorldFrame(), registry);

      for (int i = 0; i < polyhedraFrameBasisVectors.length; i++)
      {
         polyhedraFrameBasisVectors[i] = new YoFrameVector3D("beta" + contactPointIndex + "_" + i, ReferenceFrame.getWorldFrame(), registry);
         YoGraphicVector basisVector = new YoGraphicVector("betaGraphic" + contactPointIndex + "_" + i, surfacePose.getPosition(), polyhedraFrameBasisVectors[i], basisVectorGraphicScale, YoAppearance.Black(), true, 0.008);
         graphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), basisVector);
      }

      YoGraphicVector forceVector = new YoGraphicVector("forceGraphic" + contactPointIndex, surfacePose.getPosition(), force, forceVectorGraphicScale, YoAppearance.Red(), true, 0.008);
      graphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), forceVector);

      for (int i = 0; i < rhoValues.length; i++)
      {
         rhoValues[i] = new YoDouble("rho_cp" + contactPointIndex + "_" + i, registry);
      }
   }

   public void initialize(MultiContactFrictionBasedSupportRegionSolverInput input)
   {
      FramePoint3D contactPointPosition = input.getContactPointPositions().get(contactPointIndex);
      FrameVector3D surfaceNormal = input.getSurfaceNormals().get(contactPointIndex);

      contactPointPosition.changeFrame(ReferenceFrame.getWorldFrame());
      surfaceNormal.changeFrame(ReferenceFrame.getWorldFrame());

      surfacePose.getPosition().set(contactPointPosition);
      EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(Axis3D.Z, surfaceNormal, surfacePose.getOrientation());
      contactPointFrame.setPoseAndUpdate(surfacePose);

      double angleFromNormal = Math.atan(input.getCoefficientOfFriction());
      for (int i = 0; i < polyhedraFrameBasisVectors.length; i++)
      {
         Vector3D basisVector = new Vector3D();
         basisVector.set(Axis3D.Z);

         double angleToRotateAroundInXY = i * 2.0 * Math.PI / basisVectorsPerContactPoint;
         AxisAngle basisVectorRotation = new AxisAngle(Math.cos(angleToRotateAroundInXY), Math.sin(angleToRotateAroundInXY), 0.0, angleFromNormal);
         basisVectorRotation.transform(basisVector);

         FrameVector3D frameBasisVector = new FrameVector3D(contactPointFrame, basisVector);
         frameBasisVector.changeFrame(ReferenceFrame.getWorldFrame());
         polyhedraFrameBasisVectors[i].set(frameBasisVector);
      }
   }

   public YoFrameVector3D getBasisVector(int index)
   {
      return polyhedraFrameBasisVectors[index];
   }

   public void clear()
   {
      surfacePose.setToNaN();
      for (int i = 0; i < polyhedraFrameBasisVectors.length; i++)
      {
         polyhedraFrameBasisVectors[i].setToNaN();
      }
   }

   public void setResolvedForce(double[] solution)
   {
      force.setToZero();

      for (int i = 0; i < basisVectorsPerContactPoint; i++)
      {
         Vector3D scaledBasisVector = new Vector3D(polyhedraFrameBasisVectors[i]);
         double rho = solution[basisVectorsPerContactPoint * contactPointIndex + i];
         scaledBasisVector.scale(rho);
         force.add(scaledBasisVector);
         rhoValues[i].set(rho);

         if (Double.isNaN(rho))
         {
            force.setToNaN();
            return;
         }
      }
   }

   public Tuple3DReadOnly getResolvedForce()
   {
      return force;
   }
}
