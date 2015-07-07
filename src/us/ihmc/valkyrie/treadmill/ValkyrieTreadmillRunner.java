package us.ihmc.valkyrie.treadmill;

public class ValkyrieTreadmillRunner {


	public ValkyrieTreadmillRunner() {
	
	}

	public static void main(String[] args) {
		ValkyrieTreadmill myTreadmill = new ValkyrieTreadmill(true); //Show GUI
		Thread myThread = new Thread(myTreadmill);
		myThread.start();
	}
}
