package us.ihmc.commonWalkingControlModules.captureRegion;

import java.awt.Color;

import javax.media.j3d.Transform3D;
import javax.vecmath.Color3f;

import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.math.frames.YoFramePoint2d;
import us.ihmc.yoUtilities.math.frames.YoFrameVector2d;

import com.yobotics.simulationconstructionset.plotting.DynamicGraphicYoPolygonArtifact;
import com.yobotics.simulationconstructionset.plotting.DynamicGraphicYoVectorArtifact;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameConvexPolygon2d;

public class OrientationStateVisualizer 
{
	private static final Color REDUCED_SUPPORT_POLYGON_COLOR = Color.ORANGE;
	private static final Color3f PELVIS_X_AXIS_COLOR = new Color3f(Color.RED);
	private static final Color3f PELVIS_Y_AXIS_COLOR = new Color3f(Color.WHITE);
	private static final double PELVIS_SYS_SCALING = 0.3;
	
	private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
	   
	private final String name = getClass().getSimpleName();
	private final YoVariableRegistry registry = new YoVariableRegistry(name + "Registry");
	
	private final DynamicGraphicYoPolygonArtifact reducedSupportPolygonArtifact;  
	private YoFrameConvexPolygon2d yoreducedSupportPolygon;
	private FrameConvexPolygon2d reducedSupportPolygon;
	

	private final DynamicGraphicYoVectorArtifact pelvisXaxisArtifact;
	private final YoFrameVector2d yoPelvisXaxis;
	private final FrameVector pelvisXaxis;
	private final YoFramePoint2d yoPelvisXaxisBase;
	private final FramePoint pelvisXaxisBase;
	
	private final DynamicGraphicYoVectorArtifact pelvisYaxisArtifact;
	private final YoFrameVector2d yoPelvisYaxis;
	private final FrameVector pelvisYaxis;
	private final YoFramePoint2d yoPelvisYaxisBase;
	private final FramePoint pelvisYaxisBase;
	
	public OrientationStateVisualizer(DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, YoVariableRegistry parentRegistry)
   {
      String reducedSupportPolygonCaption = "ReducedSupportPolygon";
      yoreducedSupportPolygon = new YoFrameConvexPolygon2d(reducedSupportPolygonCaption, "", worldFrame, 8, registry);
      reducedSupportPolygon = new FrameConvexPolygon2d(worldFrame);
      reducedSupportPolygonArtifact = new DynamicGraphicYoPolygonArtifact(reducedSupportPolygonCaption, yoreducedSupportPolygon, REDUCED_SUPPORT_POLYGON_COLOR,
            false);

      String pelvisXaxisCaption = "PelvisXaxis";
      yoPelvisXaxis = new YoFrameVector2d(pelvisXaxisCaption, worldFrame, registry);
      pelvisXaxis = new FrameVector(worldFrame, PELVIS_SYS_SCALING, 0, 0);
      yoPelvisXaxisBase = new YoFramePoint2d(pelvisXaxisCaption + "Base", worldFrame, registry);
      pelvisXaxisBase = new FramePoint(worldFrame, 0, 0, 0);
      pelvisXaxisArtifact = new DynamicGraphicYoVectorArtifact(pelvisXaxisCaption, yoPelvisXaxisBase, yoPelvisXaxis, PELVIS_X_AXIS_COLOR);

      String pelvisYaxisCaption = "PelvisYaxis";
      yoPelvisYaxis = new YoFrameVector2d(pelvisYaxisCaption, worldFrame, registry);
      pelvisYaxis = new FrameVector(worldFrame, 0, PELVIS_SYS_SCALING, 0);
      yoPelvisYaxisBase = new YoFramePoint2d(pelvisYaxisCaption + "Base", worldFrame, registry);
      pelvisYaxisBase = new FramePoint(worldFrame, 0, 0, 0);
      pelvisYaxisArtifact = new DynamicGraphicYoVectorArtifact(pelvisYaxisCaption, yoPelvisYaxisBase, yoPelvisYaxis, PELVIS_Y_AXIS_COLOR);

      if (dynamicGraphicObjectsListRegistry != null)
      {
         dynamicGraphicObjectsListRegistry.registerArtifact(reducedSupportPolygonCaption, reducedSupportPolygonArtifact);
         dynamicGraphicObjectsListRegistry.registerArtifact(pelvisXaxisCaption, pelvisXaxisArtifact);
         dynamicGraphicObjectsListRegistry.registerArtifact(pelvisYaxisCaption, pelvisYaxisArtifact);
      }

      parentRegistry.addChild(registry);
   }
	
	public void updateReducedSupportPolygon(FrameConvexPolygon2d newReducedSupportPolygon)
	{
		reducedSupportPolygon = new FrameConvexPolygon2d(newReducedSupportPolygon);
		reducedSupportPolygon.changeFrame(worldFrame);
		
		try
		{
			yoreducedSupportPolygon.setFrameConvexPolygon2d(reducedSupportPolygon);
		}
		catch (Exception e)
	    {
	       e.printStackTrace();
	    }
	}
	
	public void updatePelvisReferenceFrame(Transform3D fromWoldToPelvis)
	{
		pelvisXaxis.applyTransform(fromWoldToPelvis);
		pelvisXaxisBase.applyTransform(fromWoldToPelvis);
		pelvisYaxis.applyTransform(fromWoldToPelvis);
		pelvisYaxisBase.applyTransform(fromWoldToPelvis);
		
		try
		{
			yoPelvisXaxis.set(pelvisXaxis.getReferenceFrame(), pelvisXaxis.getX(), pelvisXaxis.getY());
			yoPelvisXaxisBase.set(pelvisXaxisBase.getReferenceFrame(),pelvisXaxisBase.getX(), pelvisXaxisBase.getY());
			yoPelvisYaxis.set(pelvisYaxis.getReferenceFrame(), pelvisYaxis.getX(), pelvisYaxis.getY());
			yoPelvisYaxisBase.set(pelvisYaxisBase.getReferenceFrame(),pelvisYaxisBase.getX(), pelvisYaxisBase.getY());
		}
		catch (Exception e)
	    {
	       e.printStackTrace();
	    }
		
		// reset vectors, we do this way to avoid the creation of new objects every time the method is called
		pelvisXaxis.set(PELVIS_SYS_SCALING, 0, 0);
		pelvisXaxisBase.set(0, 0, 0);
		pelvisYaxis.set(0, PELVIS_SYS_SCALING, 0);
		pelvisYaxisBase.set(0, 0, 0);
	}

}
