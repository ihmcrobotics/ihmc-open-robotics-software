package us.ihmc.rdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;

public class RDXFocusBasedCameraDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private RDXFocusBasedCamera focusBasedCamera;
   private final Vector3D tempVector = new Vector3D();
   private RDXReferenceFrameGraphic cameraFrameGraphic;
   private RDXReferenceFrameGraphic cameraPoseGraphic;
   private RDXReferenceFrameGraphic focusPointPoseGraphic;

   public RDXFocusBasedCameraDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            focusBasedCamera = baseUI.getPrimary3DPanel().getCamera3D();
            focusBasedCamera.changeCameraPosition(5.0, 5.0, 5.0);

            baseUI.getPrimaryScene().addModelInstance(new ModelInstance(RDXModelBuilder.createCoordinateFrame(0.3)));

            cameraFrameGraphic = new RDXReferenceFrameGraphic(0.3, Color.SKY);
            baseUI.getPrimaryScene().addRenderableProvider(cameraFrameGraphic);
            cameraPoseGraphic = new RDXReferenceFrameGraphic(0.25, Color.OLIVE);
            baseUI.getPrimaryScene().addRenderableProvider(cameraPoseGraphic);
            focusPointPoseGraphic = new RDXReferenceFrameGraphic(0.25, Color.SALMON);
            baseUI.getPrimaryScene().addRenderableProvider(focusPointPoseGraphic);
         }

         @Override
         public void render()
         {
            focusPointPoseGraphic.getFramePose3D().setIncludingFrame(focusBasedCamera.getFocusPointPose());
            focusPointPoseGraphic.getFramePose3D().changeFrame(ReferenceFrame.getWorldFrame());
            focusPointPoseGraphic.updateFromFramePose();

            cameraPoseGraphic.getFramePose3D().setIncludingFrame(focusBasedCamera.getCameraPose());
            cameraPoseGraphic.getFramePose3D().changeFrame(ReferenceFrame.getWorldFrame());
            tempVector.sub(cameraPoseGraphic.getFramePose3D().getPosition(), focusPointPoseGraphic.getFramePose3D().getPosition());
            tempVector.scale(1.5);
            cameraPoseGraphic.getFramePose3D().getPosition().set(focusPointPoseGraphic.getFramePose3D().getPosition());
            cameraPoseGraphic.getFramePose3D().getPosition().add(tempVector);
            cameraPoseGraphic.updateFromFramePose();

            cameraFrameGraphic.getFramePose3D().setToZero(focusBasedCamera.getCameraFrame());
            cameraFrameGraphic.getFramePose3D().changeFrame(ReferenceFrame.getWorldFrame());
            tempVector.sub(cameraFrameGraphic.getFramePose3D().getPosition(), focusPointPoseGraphic.getFramePose3D().getPosition());
            tempVector.scale(0.5);
            cameraFrameGraphic.getFramePose3D().getPosition().set(focusPointPoseGraphic.getFramePose3D().getPosition());
            cameraFrameGraphic.getFramePose3D().getPosition().add(tempVector);
            cameraFrameGraphic.updateFromFramePose();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXFocusBasedCameraDemo();
   }
}
