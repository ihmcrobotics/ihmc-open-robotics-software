/*
 * Copyright (c) 2013-2014, Peter Abeles. All Rights Reserved.
 *
 * This file is part of Project BUBO.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package us.ihmc.sensorProcessing.bubo.gui.d3;

import java.util.List;

import javax.swing.JPanel;

import georegression.struct.line.LineSegment3D_F64;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;

/**
 * Class for displaying 3D point clouds in a swing panel.
 *
 * @author Peter Abeles
 */
public abstract class PointCloudPanel extends JPanel {

	public abstract void setFov( double angle );

	public abstract void setCamera( Se3_F64 camera );

	public abstract void setShowAxis( boolean show );

	public abstract boolean getShowAxis();

	public abstract void addPoints( List<Point3D_F64> points , int argb , float size );

	public abstract void addPoints( List<Point3D_F64> points , int[] argb , float size );

	public abstract void addBox( double sizeX , double sizeY , double sizeZ ,
								 Se3_F64 boxToWorld ,
								 int argb );

	/**
	 * Draws 2D mesh on a plane
	 *
	 * @param meshToWorld Transform from the plane to world coordinate
	 * @param vertexes Ordered (CW or CCW) set of vertexes which define the outside hull of the mesh
	 */
	public abstract void addMesh2D( Se3_F64 meshToWorld , List<Point2D_F64> vertexes , int argb );

	public abstract void addVectors( List<Point3D_F64> location , List<Vector3D_F64> direction , int argb );

	public abstract void addSphere( double x , double y , double z, double radius , int argb );

	/**
	 * Renders arrows pointing along the local x,y,z axis in the global frame
	 * @param localToWorld transform from local to global reference frames
	 * @param armLength Length of the arrows
	 * @param armRadius Radius/thickness of the arrows
	 */
	public abstract void addAxis( Se3_F64 localToWorld , double armLength , double armRadius );

	public abstract void addLines( List<LineSegment3D_F64> lines , double radius , int argb );

	public abstract void shutdownVisualize();

}
