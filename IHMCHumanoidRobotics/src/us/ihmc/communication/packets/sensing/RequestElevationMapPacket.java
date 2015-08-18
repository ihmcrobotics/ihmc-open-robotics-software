package us.ihmc.communication.packets.sensing;

import us.ihmc.communication.packets.Packet;

public class RequestElevationMapPacket extends Packet<RequestElevationMapPacket> {

	public double x;
	public double y;
	public double xLength;
	public double yLength;
	
	public RequestElevationMapPacket()
	{
		// for serialization
	}
	
	public RequestElevationMapPacket(double x, double y, double xLength, double yLength)
	{
		this.x = x;
		this.y = y;
		this.xLength = xLength;
		this.yLength = yLength;
	}
	
	public double[] getData()
	{
		return new double[]{x, y, xLength, yLength};
	}
	
	@Override
	public boolean epsilonEquals(RequestElevationMapPacket other, double epsilon) {
		boolean ret = true;
		double[] thisData = this.getData();
		double[] otherData = other.getData();
		
		for (int i = 0; (i < thisData.length && ret); i++){
			ret = Math.abs(thisData[i] - otherData[i]) < epsilon;
		}
		return ret;
	}

}
