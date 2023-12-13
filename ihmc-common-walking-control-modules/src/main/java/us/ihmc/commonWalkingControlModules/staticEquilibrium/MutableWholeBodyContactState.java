package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

import java.util.ArrayList;
import java.util.List;

public class MutableWholeBodyContactState implements WholeBodyContactStateInterface
{
   private final List<PoseReferenceFrame> contactFrames = new ArrayList<>();

   private final DMatrixRMaj actuationMatrix = new DMatrixRMaj(0);
   private final DMatrixRMaj actuationVector = new DMatrixRMaj(0);

   public void addContactPoint(Tuple3DReadOnly contactPoint, Vector3DReadOnly surfaceNormal)
   {
      PoseReferenceFrame contactFrame = new PoseReferenceFrame("contactFrame" + contactFrames.size(), ReferenceFrame.getWorldFrame());
      FramePose3D contactPose = new FramePose3D();
      contactPose.getPosition().set(contactPoint);
      EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(Axis3D.Z, surfaceNormal, contactPose.getOrientation());
      contactFrame.setPoseAndUpdate(contactPose);
      contactFrames.add(contactFrame);
   }

   public void clear()
   {
      contactFrames.clear();
   }

   @Override
   public int getNumberOfContactPoints()
   {
      return contactFrames.size();
   }

   @Override
   public ReferenceFrame getContactFrame(int contactPointIndex)
   {
      return contactFrames.get(contactPointIndex);
   }

   @Override
   public DMatrixRMaj getActuationConstraintMatrix()
   {
      return actuationMatrix;
   }

   @Override
   public DMatrixRMaj getActuationConstraintVector()
   {
      return actuationVector;
   }
}
