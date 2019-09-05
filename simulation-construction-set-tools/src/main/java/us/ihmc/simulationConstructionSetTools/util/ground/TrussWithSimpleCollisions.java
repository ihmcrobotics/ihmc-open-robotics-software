package us.ihmc.simulationConstructionSetTools.util.ground;

import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;

public class TrussWithSimpleCollisions extends RotatableBoxTerrainObject
{
	public TrussWithSimpleCollisions(Box3D box, AppearanceDefinition appearance)
	{
		super(box, appearance);
	}

	public TrussWithSimpleCollisions(double[] newPoint, double trussLength,
			double trussSide, double courseAngleDeg, AppearanceDefinition color)
	{
		this(new Box3D(TransformTools.yawPitchDegreesTransform(new Vector3D(newPoint[0], newPoint[1], trussSide/2), courseAngleDeg, 0),
				trussSide, trussLength, trussSide), color);
	}
//	// TODO Auto-generated constructor stub
//	AppearanceDefinition overrideColor = YoAppearance.White();	//color;
//	overrideColor.setTransparency(1.0);
//	setUpSlopedBox(newPoint[0], newPoint[1], trussSide / 2.0, trussLength,
//			trussSide, trussSide, 0.0, courseAngleDeg - 45, overrideColor);
//	///
////	////setupsloped box
////	setUpSlopedBox(double xCenter, double yCenter, double zCenter,
////			double xLength, double yLength, double zLength,
////			double slopeRadians, double yawDegrees, AppearanceDefinition app) 
////	Transform3D location = new Transform3D();
////	location.rotZ(Math.toRadians(yawDegrees));
////
////	Transform3D tilt = new Transform3D();
////	tilt.setToPitchMatrix(-slopeRadians);
////	location.mul(tilt);
////
////	location.setTranslation(new Vector3d(xCenter, yCenter, zCenter));
////	RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(
////			new Box3d(location, xLength, yLength, zLength), app);
////	combinedTerrainObject.addTerrainObject(newBox);
////	////////////////
//
//	///
//	/////
//	//		Transform3D location = new Transform3D();
//	//		location.rotZ(Math.toRadians(yawDegrees));
//	//
//	//		Transform3D tilt = new Transform3D();
//	//		tilt.setToPitchMatrix(-slopeRadians);
//	//		location.mul(tilt);
//	//
//	//		location.setTranslation(new Vector3d(xCenter, yCenter, zCenter));
//	//		RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(
//	//				new Box3d(location, xLength, yLength, zLength), app);
//	//		combinedTerrainObject.addTerrainObject(newBox);
//	//
//	//		///
//	Vector3d translation = new Vector3d(newPoint[0], newPoint[1], trussSide / 2.0);
//
//	Graphics3DObject linkGraphics = new Graphics3DObject();
//
//	linkGraphics.translate(translation);
//	linkGraphics.rotate(Math.toRadians(courseAngleDeg + 45), Axis.Z);
//
//	linkGraphics.addModelFile(getClass().getResource("truss.dae"));
//	combinedTerrainObject.addStaticLinkGraphics(linkGraphics);


	@Override
   protected void addGraphics()
	{
		RigidBodyTransform transformCenterConventionToBottomConvention = new RigidBodyTransform(box.getPose());
//		transformCenterConventionToBottomConvention = TransformTools.transformLocalZ(transformCenterConventionToBottomConvention, -box.getDimension(Direction.Z) / 2.0);
		
		double graphicSide=.291;
		double graphicLength=1.524;
		Vector3D vector = new Vector3D(box.getSizeX()/graphicSide, box.getSizeY()/graphicLength, box.getSizeZ()/graphicSide);

		linkGraphics = new Graphics3DObject();
		linkGraphics.transform(transformCenterConventionToBottomConvention);
		linkGraphics.scale(vector);
		linkGraphics.addModelFile("models/truss.dae");
	}

}
