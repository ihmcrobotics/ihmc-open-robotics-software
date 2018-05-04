package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.text.WordUtils;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.FootSwitchInterface;


public class CenterOfPressureVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final Map<RigidBody, YoFramePoint3D> footRawCoPPositionsInWorld = new HashMap<>();
   private final YoFramePoint3D overallRawCoPPositionInWorld;
   private final FramePoint2D tempRawCoP2d = new FramePoint2D();
   private final FramePoint3D tempRawCoP = new FramePoint3D();
   private final Wrench tempWrench = new Wrench();
   private final Map<RigidBody, FootSwitchInterface> footSwitches;
   private final Collection<RigidBody> footRigidBodies;
   private final List<RigidBody> footList = new ArrayList<>();

   public CenterOfPressureVisualizer(Map<RigidBody, FootSwitchInterface> footSwitches,
         YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.footSwitches = footSwitches;
      footRigidBodies = footSwitches.keySet();

      for (RigidBody rigidBody : footRigidBodies)
      {
         String rigidBodyName = rigidBody.getName();
         rigidBodyName = WordUtils.capitalize(rigidBodyName);

         YoFramePoint3D rawCoPPositionInWorld = new YoFramePoint3D("raw" + rigidBodyName + "CoPPositionsInWorld", worldFrame, registry);
         footRawCoPPositionsInWorld.put(rigidBody, rawCoPPositionInWorld);

         YoGraphicPosition copYoGraphic = new YoGraphicPosition("Meas " + rigidBodyName + "CoP", rawCoPPositionInWorld, 0.008, YoAppearance.DarkRed(), GraphicType.DIAMOND);
         YoArtifactPosition copArtifact = copYoGraphic.createArtifact();
         yoGraphicsListRegistry.registerArtifact("StateEstimator", copArtifact);

         footList.add(rigidBody);
      }

      overallRawCoPPositionInWorld = new YoFramePoint3D("overallRawCoPPositionInWorld", worldFrame, registry);
      YoGraphicPosition overallRawCoPYoGraphic = new YoGraphicPosition("Meas CoP", overallRawCoPPositionInWorld, 0.015, YoAppearance.DarkRed(), GraphicType.DIAMOND);
      YoArtifactPosition overallRawCoPArtifact = overallRawCoPYoGraphic.createArtifact();
      overallRawCoPArtifact.setVisible(false);
      yoGraphicsListRegistry.registerArtifact("StateEstimator", overallRawCoPArtifact);

      parentRegistry.addChild(registry);
   }

   public void update()
   {
      if (footRawCoPPositionsInWorld != null)
      {
         overallRawCoPPositionInWorld.setToZero();
         double totalFootForce = 0.0;

         for (int i = 0; i < footList.size(); i++)
         {
            RigidBody rigidBody = footList.get(i);

            footSwitches.get(rigidBody).computeAndPackCoP(tempRawCoP2d);
            tempRawCoP.setIncludingFrame(tempRawCoP2d.getReferenceFrame(), tempRawCoP2d.getX(), tempRawCoP2d.getY(), 0.0);
            tempRawCoP.changeFrame(worldFrame);
            footRawCoPPositionsInWorld.get(rigidBody).set(tempRawCoP);

            footSwitches.get(rigidBody).computeAndPackFootWrench(tempWrench);
            double singleFootForce = tempWrench.getLinearPartZ();
            totalFootForce += singleFootForce;
            tempRawCoP.scale(singleFootForce);
            overallRawCoPPositionInWorld.add(tempRawCoP);
         }

         overallRawCoPPositionInWorld.scale(1.0 / totalFootForce);
      }
   }

   public void hide()
   {
      for (RigidBody rigidBody : footRigidBodies)
      {
         footRawCoPPositionsInWorld.get(rigidBody).setToNaN();
      }
      overallRawCoPPositionInWorld.setToNaN();
   }
}
