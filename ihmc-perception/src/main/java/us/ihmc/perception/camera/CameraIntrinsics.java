package us.ihmc.perception.camera;

public class CameraIntrinsics {
    private double fx;
    private double fy;
    private double cx;
    private double cy;

    public CameraIntrinsics() {
    }

    public CameraIntrinsics(double fx, double fy, double cx, double cy) {
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
    }

    public double getFx() {
        return fx;
    }

    public double getFy() {
        return fy;
    }

    public double getCx() {
        return cx;
    }

    public double getCy() {
        return cy;
    }

    public void setFx(double fx) {
        this.fx = fx;
    }

    public void setFy(double fy) {
        this.fy = fy;
    }

    public void setCx(double cx) {
        this.cx = cx;
    }

    public void setCy(double cy) {
        this.cy = cy;
    }
}
