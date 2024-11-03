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
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.DefaultPoint2DGraphic;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class CenterOfPressureVisualizer implements SCS2YoGraphicHolder
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final Map<RigidBodyBasics, YoFramePoint3D> footRawCoPPositionsInWorld = new HashMap<>();
   private final YoFramePoint3D overallRawCoPPositionInWorld;
   private final FramePoint2D tempRawCoP2d = new FramePoint2D();
   private final FramePoint3D tempRawCoP = new FramePoint3D();
   private final Wrench tempWrench = new Wrench();
   private final Map<RigidBodyBasics, FootSwitchInterface> footSwitches;
   private final Collection<RigidBodyBasics> footRigidBodies;
   private final List<RigidBodyBasics> footList = new ArrayList<>();

   public CenterOfPressureVisualizer(Map<RigidBodyBasics, FootSwitchInterface> footSwitches,
                                     YoGraphicsListRegistry yoGraphicsListRegistry,
                                     YoRegistry parentRegistry)
   {
      this.footSwitches = footSwitches;
      footRigidBodies = footSwitches.keySet();

      for (RigidBodyBasics rigidBody : footRigidBodies)
      {
         String rigidBodyName = rigidBody.getName();
         rigidBodyName = WordUtils.capitalize(rigidBodyName);

         YoFramePoint3D rawCoPPositionInWorld = new YoFramePoint3D("raw" + rigidBodyName + "CoPPositionsInWorld", worldFrame, registry);
         footRawCoPPositionsInWorld.put(rigidBody, rawCoPPositionInWorld);

         YoGraphicPosition copYoGraphic = new YoGraphicPosition("Meas " + rigidBodyName
               + "CoP", rawCoPPositionInWorld, 0.008, YoAppearance.DarkRed(), GraphicType.DIAMOND);
         YoArtifactPosition copArtifact = copYoGraphic.createArtifact();
         yoGraphicsListRegistry.registerArtifact("StateEstimator", copArtifact);

         footList.add(rigidBody);
      }

      overallRawCoPPositionInWorld = new YoFramePoint3D("overallRawCoPPositionInWorld", worldFrame, registry);
      YoGraphicPosition overallRawCoPYoGraphic = new YoGraphicPosition("Meas CoP",
                                                                       overallRawCoPPositionInWorld,
                                                                       0.015,
                                                                       YoAppearance.DarkRed(),
                                                                       GraphicType.DIAMOND);
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
            RigidBodyBasics rigidBody = footList.get(i);

            footSwitches.get(rigidBody).getCenterOfPressure(tempRawCoP2d);
            tempRawCoP.setIncludingFrame(tempRawCoP2d.getReferenceFrame(), tempRawCoP2d.getX(), tempRawCoP2d.getY(), 0.0);
            tempRawCoP.changeFrame(worldFrame);
            footRawCoPPositionsInWorld.get(rigidBody).set(tempRawCoP);

            footSwitches.get(rigidBody).getMeasuredWrench(tempWrench);
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
      for (RigidBodyBasics rigidBody : footRigidBodies)
      {
         footRawCoPPositionsInWorld.get(rigidBody).setToNaN();
      }
      overallRawCoPPositionInWorld.setToNaN();
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      for (RigidBodyBasics rigidBody : footRigidBodies)
      {
         group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint2D("Meas " + rigidBody.getName()
               + "CoP", footRawCoPPositionsInWorld.get(rigidBody), 0.016, ColorDefinitions.DarkRed(), DefaultPoint2DGraphic.DIAMOND));
      }
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint2D("Meas CoP",
                                                                    overallRawCoPPositionInWorld,
                                                                    0.03,
                                                                    ColorDefinitions.DarkRed(),
                                                                    DefaultPoint2DGraphic.DIAMOND));
      return group;
   }
}
