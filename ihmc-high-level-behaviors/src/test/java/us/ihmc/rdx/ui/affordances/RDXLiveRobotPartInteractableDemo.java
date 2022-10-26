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
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.scs2.simulation.collision.Collidable;

public class RDXLiveRobotPartInteractableDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(),
                                                  "ihmc-open-robotics-software",
                                                  "ihmc-high-level-behaviors/src/test/resources",
                                                  "Live Robot Part Interactable Demo");
   private RDXLiveRobotPartInteractable liveRobotPartInteractable;
   private RDXRobotCollidable robotRobotCollidable;

   public RDXLiveRobotPartInteractableDemo()
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

            robotRobotCollidable = new RDXRobotCollidable(collidable, Color.GRAY);
            baseUI.getVRManager().getContext().addVRPickCalculator(robotRobotCollidable::calculateVRPick);
            baseUI.getVRManager().getContext().addVRInputProcessor(robotRobotCollidable::processVRInput);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(robotRobotCollidable::calculatePick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(robotRobotCollidable::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(robotRobotCollidable, RDXSceneLevel.VIRTUAL);

            ModifiableReferenceFrame controlFrame = new ModifiableReferenceFrame(ReferenceFrame.getWorldFrame());
            liveRobotPartInteractable = new RDXLiveRobotPartInteractable();
            liveRobotPartInteractable.create(robotRobotCollidable,
                                             controlFrame.getReferenceFrame(),
                                             "environmentObjects/l515Sensor/L515Sensor.g3dj",
                                             baseUI.getPrimary3DPanel());

            baseUI.getPrimaryScene().addRenderableProvider(liveRobotPartInteractable::getVirtualRenderables, RDXSceneLevel.VIRTUAL);
            baseUI.getImGuiPanelManager().addPanel("Live Robot Part Interactable", liveRobotPartInteractable::renderImGuiWidgets);
            baseUI.getVRManager().getContext().addVRInputProcessor(liveRobotPartInteractable::processVRInput);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(liveRobotPartInteractable::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(liveRobotPartInteractable::process3DViewInput);
         }

         @Override
         public void render()
         {
            robotRobotCollidable.update();
            liveRobotPartInteractable.update();

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
      new RDXLiveRobotPartInteractableDemo();
   }
}
