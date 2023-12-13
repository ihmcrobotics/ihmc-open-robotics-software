package us.ihmc.rdx.ui.affordances;

import com.badlogic.gdx.graphics.Color;
import us.ihmc.euclid.referenceFrame.FrameSphere3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.scs2.simulation.collision.Collidable;

public class RDXInteractableRobotLinkDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI("Interactable Robot Link Demo");
   private RDXInteractableRobotLink interactableRobotLink;
   private RDXRobotCollidable robotRobotCollidable;

   public RDXInteractableRobotLinkDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            RigidBody rootJoint = new RigidBody("root", ReferenceFrame.getWorldFrame());
            SixDoFJoint elevator = new SixDoFJoint("elevator", rootJoint);
            RigidBody rigidBody = new RigidBody("sphereBody", elevator, 0.1, 0.1, 0.1, 1.0, new Point3D());
            FrameSphere3D frameShape = new FrameSphere3D(ReferenceFrame.getWorldFrame(), 0.03);
            Collidable collidable = new Collidable(rigidBody, 0, 0, frameShape);

            robotRobotCollidable = new RDXRobotCollidable(collidable, 0, Color.GRAY);
            baseUI.getVRManager().getContext().addVRPickCalculator(robotRobotCollidable::calculateVRPick);
            baseUI.getVRManager().getContext().addVRInputProcessor(robotRobotCollidable::processVRInput);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(robotRobotCollidable::calculatePick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(robotRobotCollidable::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(robotRobotCollidable, RDXSceneLevel.VIRTUAL);

            MutableReferenceFrame controlFrame = new MutableReferenceFrame(ReferenceFrame.getWorldFrame());
            interactableRobotLink = new RDXInteractableRobotLink();
            interactableRobotLink.create(robotRobotCollidable,
                                         controlFrame.getReferenceFrame(),
                                         "environmentObjects/l515Sensor/L515Sensor.g3dj",
                                         baseUI.getPrimary3DPanel());

            baseUI.getPrimaryScene().addRenderableProvider(interactableRobotLink::getVirtualRenderables, RDXSceneLevel.VIRTUAL);
            baseUI.getImGuiPanelManager().addPanel("Live Robot Part Interactable", interactableRobotLink::renderImGuiWidgets);
            baseUI.getVRManager().getContext().addVRInputProcessor(interactableRobotLink::processVRInput);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(interactableRobotLink::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(interactableRobotLink::process3DViewInput);
         }

         @Override
         public void render()
         {
            robotRobotCollidable.update();
            interactableRobotLink.update();

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
      new RDXInteractableRobotLinkDemo();
   }
}
