package us.ihmc.valkyrie.torquespeedcurve;

public class TorqueSpeedTestResult {
    public String testInfo;
    public String testSuffix;
    public boolean testSucceeded;
    
    public TorqueSpeedTestResult(String info, String suffix, boolean result) {
    	this.testInfo = info;
    	this.testSuffix = suffix;
    	this.testSucceeded = result;
    }
}
