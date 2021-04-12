package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

/* package-private */ class StaticEquilibriumContactPoint
{
   private final FrameVector3D[] polyhedraFrameBasisVectors = new FrameVector3D[4];
   private final FramePose3D surfacePose = new FramePose3D();
   private final PoseReferenceFrame contactPointFrame;

   public StaticEquilibriumContactPoint(int contactPointIndex, StaticEquilibriumSolverInput input)
   {
      FramePoint3D contactPointPosition = input.getContactPointPositions().get(contactPointIndex);
      FrameVector3D surfaceNormal = input.getSurfaceNormals().get(contactPointIndex);
      contactPointFrame = new PoseReferenceFrame("cpFrame" + contactPointIndex, ReferenceFrame.getWorldFrame());

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

         double angleToRotateAroundInXY = i * Math.PI / 2.0;
         AxisAngle basisVectorRotation = new AxisAngle(Math.cos(angleToRotateAroundInXY), Math.sin(angleToRotateAroundInXY), 0.0, angleFromNormal);
         basisVectorRotation.transform(basisVector);

         FrameVector3D frameBasisVector = new FrameVector3D(contactPointFrame, basisVector);
         frameBasisVector.changeFrame(ReferenceFrame.getWorldFrame());
         polyhedraFrameBasisVectors[i] = frameBasisVector;
      }
   }

   public FrameVector3D getBasisVector(int index)
   {
      return polyhedraFrameBasisVectors[index];
   }
}
