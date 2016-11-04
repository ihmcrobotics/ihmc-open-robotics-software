package optiTrack;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class IHMCMocapDataClient extends MocapDataClient
{
   private ReferenceFrame mocapOriginFrame;
   private ReferenceFrame mocapRbFrame;
   private ReferenceFrame mocapRbZUpFrame;

   private RigidBodyTransform mocapRbToMocapOrigin;

   public IHMCMocapDataClient()
   {
      super();

      setupReferenceFrames();
   }

   public void setupReferenceFrames()
   {
      mocapOriginFrame = new ReferenceFrame("mocapOrigin", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.setRotationEulerAndZeroTranslation(Math.toRadians(-90), Math.toRadians(0), Math.toRadians(0));
         }
      };
      mocapOriginFrame.update();

      mocapRbFrame = new ReferenceFrame("mocapRB", mocapOriginFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(mocapRbToMocapOrigin);
         }
      };

      mocapRbZUpFrame = new ReferenceFrame("mocapRbZUpFrame", mocapRbFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.setRotationEulerAndZeroTranslation(Math.toRadians(90), 0, 0);
         }
      };
      mocapRbZUpFrame.update();
   }

   @Override
   protected void updateListeners(ArrayList<MocapRigidBody> lisftOfRigidbodies)
   {
      ArrayList<MocapRigidBody> convertedListOfMocapRigidBodies = new ArrayList<MocapRigidBody>();
      for (MocapRigidBody mocapRigidBody : lisftOfRigidbodies)
      {
         mocapRbToMocapOrigin = new RigidBodyTransform(new Quat4d(mocapRigidBody.qx, mocapRigidBody.qy, mocapRigidBody.qz, mocapRigidBody.qw),
                 new Vector3d(mocapRigidBody.xPosition, mocapRigidBody.yPosition, mocapRigidBody.zPosition));
         mocapRbFrame.update();
         mocapOriginFrame.update();
         mocapRbZUpFrame.update();
         
         FramePose pose = new FramePose(mocapRbZUpFrame, new Point3d(),
                                        new Quat4d());
         pose.changeFrame(ReferenceFrame.getWorldFrame());

         RigidBodyTransform r = new RigidBodyTransform();
         pose.getRigidBodyTransform(r);

         Vector3d position = new Vector3d();
         r.getTranslation(position);

         Quat4d rotation = new Quat4d();
         r.getRotation(rotation);

         convertedListOfMocapRigidBodies.add(new MocapRigidBody(mocapRigidBody.getId(), position, rotation, mocapRigidBody.getListOfAssociatedMarkers(),
                 mocapRigidBody.dataValid));
      }

      updateRigidBodiesListeners(convertedListOfMocapRigidBodies);
   }

   public static void main(String args[])
   {
      IHMCMocapDataClient udpMulticastClient = new IHMCMocapDataClient();
   }
}


//~ Formatted by Jindent --- http://www.jindent.com
