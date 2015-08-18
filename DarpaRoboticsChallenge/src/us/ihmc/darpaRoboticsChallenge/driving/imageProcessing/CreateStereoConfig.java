package us.ihmc.darpaRoboticsChallenge.driving.imageProcessing;

import georegression.struct.se.Se3_F64;
import boofcv.io.UtilIO;
import boofcv.struct.calib.IntrinsicParameters;
import boofcv.struct.calib.StereoParameters;

/**
 * Creates a StereoParameters config file for DARPA robotics challenge simulation.
 *
 * @author Peter Abeles
 */
public class CreateStereoConfig {
	public static final int width = 800;
	public static final int height = 800;
	public static final double hfov = 1.396263; // radians
   public static final double vfov = 1.396263; // radians
	public static final double baseline = 0.07; // meters

	private static IntrinsicParameters createIntrinsic() {
		IntrinsicParameters param = new IntrinsicParameters();
		param.width = width;
		param.height = height;
		param.cx = width/2;
		param.cy = height/2;
		param.fx =  (float)((width/2)/ Math.tan(hfov / 2.0));
		param.fy =  (float)((height/2)/ Math.tan(vfov / 2.0));
		param.skew = 0;
		param.radial = new double[0];
		param.flipY = false;
		return param;
	}

	public static void main( String args[] ) {

		Se3_F64 rightToLeft = new Se3_F64();
		rightToLeft.getT().x = baseline;

		StereoParameters stereo = new StereoParameters();

		stereo.left = createIntrinsic();
		stereo.right = createIntrinsic();

		stereo.setRightToLeft(rightToLeft);

      UtilIO.saveXML(stereo, "stereo_simulation.xml");
	}
}
